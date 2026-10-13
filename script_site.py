"""Local web controls for the unchanged AllTests.py motor workflow.

The browser never talks to an ODrive directly. Each test runs AllTests.py in its
own process, so its USB connection, configuration, monitor and stop sequence
remain the authority for motor operation.
"""
import argparse
import io
import json
import re
import subprocess
import sys
import threading
from collections import deque
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

ROOT = Path(__file__).resolve().parent
SCRIPT = ROOT / 'AllTests.py'
RESULTS = ROOT / 'results'
STOP_REQUEST = ROOT / 'site-estop.request'
ASSETS = {'/': ('script_site.html', 'text/html; charset=utf-8'),
          '/script_site.js': ('script_site.js', 'text/javascript; charset=utf-8'),
          '/script_site.css': ('script_site.css', 'text/css; charset=utf-8')}


def point_args(data):
    """Validate UI input before handing it to the script's own Point validator."""
    mode = data.get('mode')
    if mode not in ('sensored', 'sensorless'):
        raise ValueError('Choose sensored or sensorless feedback.')
    try:
        rpm, amps = float(data['rpm']), float(data['amps'])
    except (TypeError, ValueError, KeyError):
        raise ValueError('Enter a numeric speed and opposing current.') from None
    if not 0 < rpm <= 1800 or not 0 <= amps <= 50:
        raise ValueError('The script accepts speed above 0 through 1800 rpm and current 0 through 50 A.')
    bandwidth = str(data.get('bandwidth', 'keep'))
    if bandwidth not in ('keep', '250', '500', '1000'):
        raise ValueError('Choose a listed feedback bandwidth.')
    return mode, rpm, amps, bandwidth


def matrix_args(data):
    modes = data.get('modes', 'sensored')
    if modes not in ('sensored', 'sensorless', 'sensored,sensorless'):
        raise ValueError('Choose a listed feedback group.')
    def numbers(name, low, high):
        parts = [p.strip() for p in str(data.get(name, '')).split(',')]
        if not 1 <= len(parts) <= 100 or any(not p for p in parts):
            raise ValueError(f'Enter 1 to 100 comma-separated {name}.')
        try:
            values = [float(p) for p in parts]
        except ValueError:
            raise ValueError(f'{name} must contain numbers.') from None
        if any(not low <= n <= high for n in values):
            raise ValueError(f'{name} must be between {low:g} and {high:g}.')
        return ','.join(format(n, 'g') for n in values)
    rpms = numbers('rpms', .001, 1800)
    amps = numbers('loads', 0, 50)
    bandwidth = str(data.get('bandwidth', 'keep'))
    if bandwidth not in ('keep', '250', '500', '1000'):
        raise ValueError('Choose a listed feedback bandwidth.')
    return ['--rpms', rpms, '--loads', amps, '--modes', modes, '--bandwidth', bandwidth]


class ScriptSession:
    def __init__(self):
        self.lock = threading.RLock()
        self.process = None
        self.kind = None
        self.started = None
        self.exit_code = None
        self.lines = deque(maxlen=600)
        self.partial = ''
        self.prompt = ''
        self.reader = None

    def status(self):
        with self.lock:
            running = self.process is not None and self.process.poll() is None
            return dict(running=running, kind=self.kind, started=self.started,
                        exit_code=self.exit_code, lines=list(self.lines),
                        partial=self.partial, prompt=self.prompt)

    def _read_output(self, process):
        # Read characters so prompts without a trailing newline appear in the browser.
        stream = io.TextIOWrapper(process.stdout, encoding='utf-8', errors='replace', newline='')
        try:
            while True:
                char = stream.read(1)
                if not char:
                    break
                with self.lock:
                    if char == '\n':
                        self.lines.append(self.partial.rstrip('\r'))
                        self.partial, self.prompt = '', ''
                    else:
                        self.partial += char
                        if len(self.partial) > 3000:
                            self.lines.append(self.partial[:3000])
                            self.partial = self.partial[3000:]
                        if self.partial.endswith(': ') and (
                            'Press Enter' in self.partial or 'type ready' in self.partial
                            or 'Test motor feedback [' in self.partial
                            or 'Target RPM (' in self.partial
                            or 'Test motor opposing Iq' in self.partial
                            or 'Test feedback bandwidth' in self.partial
                        ):
                            self.prompt = self.partial
        finally:
            code = process.wait()
            stream.close()
            if process.stdin:
                process.stdin.close()
            with self.lock:
                if self.partial:
                    self.lines.append(self.partial)
                    self.partial = ''
                self.prompt = ''
                self.exit_code = code
                self.lines.append(f'Process finished with exit code {code}.')

    def start(self, kind, data):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                raise ValueError('A script workflow is already running.')
            if not SCRIPT.is_file():
                raise ValueError('Bundled AllTests.py is missing.')
            if STOP_REQUEST.exists():
                STOP_REQUEST.unlink()
            initial = None
            if kind == 'point':
                mode, rpm, amps, bandwidth = point_args(data)
                args = ['--internal-stage', 'interactive', '0', '--run', '--no-live']
                initial = f'{mode}\n{rpm:g}\n{amps:g}\n{bandwidth}\n'
            elif kind == 'matrix':
                args = ['--internal-stage', 'matrix', '0', '--run', '--no-live'] + matrix_args(data)
            elif kind == 'sequence':
                args = ['--run', '--no-show']
            else:
                raise ValueError('Unknown script workflow.')
            RESULTS.mkdir(exist_ok=True)
            self.lines.clear()
            self.partial = self.prompt = ''
            self.exit_code = None
            self.kind = kind
            self.started = datetime.now(timezone.utc).isoformat()
            self.process = subprocess.Popen([sys.executable, '-u', str(SCRIPT), *args],
                cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, bufsize=0)
            if initial:
                self.process.stdin.write(initial.encode('utf-8'))
                self.process.stdin.flush()
            self.reader = threading.Thread(target=self._read_output, args=(self.process,), daemon=True)
            self.reader.start()
            return self.status()

    def answer(self, value):
        with self.lock:
            if not self.process or self.process.poll() is not None or not self.prompt:
                raise ValueError('The script is not waiting for an operator answer.')
            if value not in ('yes', 'no', 'ready'):
                raise ValueError('Choose Continue, Cancel, or Ready after GUI handoff.')
            self.process.stdin.write((value + '\n').encode('utf-8'))
            self.process.stdin.flush()
            self.lines.append(self.partial + value)
            self.partial = self.prompt = ''
            return self.status()

    def stop(self):
        with self.lock:
            if not self.process or self.process.poll() is not None:
                return self.status()
            STOP_REQUEST.write_text('operator stop requested\n', encoding='utf-8')
            if self.prompt:
                self.process.stdin.write(b'no\n')
                self.process.stdin.flush()
                self.lines.append(self.partial + 'no')
                self.partial = self.prompt = ''
            return self.status()


def listed_results():
    if not RESULTS.exists():
        return []
    files = []
    for path in RESULTS.rglob('*'):
        if path.is_file() and path.suffix.lower() in ('.csv', '.svg', '.json', '.jsonl'):
            rel = path.relative_to(RESULTS).as_posix()
            files.append(dict(path=rel, size=path.stat().st_size,
                              modified=datetime.fromtimestamp(path.stat().st_mtime, timezone.utc).isoformat()))
    return sorted(files, key=lambda item: item['modified'], reverse=True)[:500]


def live_samples(since=None):
    """Read the script's latest host-monitor tail for display only."""
    if not RESULTS.exists():
        return dict(source=None, rows=[])
    paths = (p for p in RESULTS.rglob('monitor.jsonl') if since is None or p.stat().st_mtime >= since - 1)
    path = max(paths, key=lambda p: p.stat().st_mtime, default=None)
    if path is None:
        return dict(source=None, rows=[])
    with path.open('rb') as stream:
        size = path.stat().st_size
        if size > 300_000:
            stream.seek(size - 300_000)
            stream.readline()  # discard a partial line
        lines = deque(stream, maxlen=240)
    rows = []
    for raw in lines:
        try:
            row = json.loads(raw)
            if isinstance(row, dict) and 'boards' in row:
                rows.append(row)
        except (ValueError, UnicodeDecodeError):
            pass  # a write still in progress is picked up on the next poll
    return dict(source=path.relative_to(RESULTS).as_posix(), rows=rows)


def result_file(relative):
    if not relative or '\\' in relative or '..' in Path(relative).parts:
        raise ValueError('Invalid result path.')
    path = (RESULTS / relative).resolve()
    if not path.is_relative_to(RESULTS.resolve()) or not path.is_file():
        raise ValueError('Result file not found.')
    if path.suffix.lower() not in ('.csv', '.svg', '.json', '.jsonl'):
        raise ValueError('This file type is not served.')
    return path


def make_handler(session):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def send(self, code, payload, content_type='application/json', filename=None):
            if not isinstance(payload, bytes):
                payload = json.dumps(payload, allow_nan=False).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; style-src 'self'; script-src 'self'; img-src 'self' data:")
            if filename:
                self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            try:
                url = urlsplit(self.path)
                if url.path in ASSETS:
                    name, ctype = ASSETS[url.path]
                    return self.send(200, (ROOT / name).read_bytes(), ctype)
                if url.path == '/api/status':
                    return self.send(200, session.status())
                if url.path == '/api/results':
                    return self.send(200, listed_results())
                if url.path == '/api/live':
                    state = session.status()
                    since = datetime.fromisoformat(state['started']).timestamp() if state['running'] else None
                    return self.send(200, live_samples(since))
                if url.path == '/api/info':
                    source = SCRIPT.read_text(encoding='utf-8')
                    serials = re.search(r"^SERIALS = \{'test': '([^']+)', 'load': '([^']+)'\}", source, re.M)
                    return self.send(200, dict(serials={'test': serials.group(1), 'load': serials.group(2)} if serials else {},
                                               script='AllTests.py', assignment='Load motor controls velocity; test motor applies opposing torque.'))
                if url.path == '/result':
                    relative = parse_qs(url.query).get('path', [''])[0]
                    path = result_file(relative)
                    ctype = {'.svg': 'image/svg+xml', '.csv': 'text/csv; charset=utf-8',
                             '.json': 'application/json', '.jsonl': 'application/x-ndjson'}[path.suffix.lower()]
                    return self.send(200, path.read_bytes(), ctype, path.name if path.suffix.lower() != '.svg' else None)
                self.send(404, {'error': 'Not found.'})
            except (ValueError, OSError) as exc:
                self.send(400, {'error': str(exc)})

        def do_POST(self):
            host = self.headers.get('Host', '')
            if (host not in (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}')
                    or self.headers.get('Origin') != 'http://' + host
                    or self.headers.get('Content-Type') != 'application/json'):
                return self.send(403, {'error': 'Use the local dashboard.'})
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 10000:
                    raise ValueError('Invalid request size.')
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError('Expected an object.')
                action = data.get('action')
                if action == 'start':
                    result = session.start(data.get('kind'), data)
                elif action == 'answer':
                    result = session.answer(data.get('value'))
                elif action == 'stop':
                    result = session.stop()
                else:
                    raise ValueError('Unknown action.')
                self.send(200, result)
            except (ValueError, TypeError, KeyError, OSError, BrokenPipeError) as exc:
                self.send(400, {'error': str(exc)})

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--rate', type=int, default=20)  # kept for existing start.ps1 calls
    parser.add_argument('--open-browser', action='store_true', help='Open the local site in the default browser.')
    args = parser.parse_args()
    session = ScriptSession()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(session))
    address = f'http://127.0.0.1:{args.port}'
    print(f'ODrive script workbench: {address}', flush=True)
    try:
        if args.open_browser:
            import webbrowser
            try:
                opened = webbrowser.open(address)
            except OSError:
                opened = False
            if not opened:
                print(f'Could not open the browser automatically. Visit {address}', flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        session.stop()
        if session.process and session.process.poll() is None:
            try:
                session.process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                print('Script has not exited after Stop; use the physical stop and inspect both boards.', flush=True)
        server.server_close()


if __name__ == '__main__':
    main()
