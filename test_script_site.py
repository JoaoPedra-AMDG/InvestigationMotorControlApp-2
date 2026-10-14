"""Offline checks for the browser-to-script bridge; no ODrive access."""
import tempfile
import time
import json
import threading
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

import script_site
from motor_connections import MotorConnections


class FakeConnectionDriver:
    SERIALS = {'test': '01', 'load': '02'}

    def __init__(self):
        self.events = []

    async def connect(self, serial):
        self.events.append(('connect', serial))
        return {'serial': serial}

    async def read(self, board, path):
        return 1 if path == 'axis0.current_state' else False

    def release_connection(self, board):
        self.events.append(('release', board['serial']))

    async def emergency_idle(self, boards):
        self.events.append(('stop', tuple(boards)))
        return []


class ScriptSiteTests(unittest.TestCase):
    def test_http_connect_and_disconnect_controls(self):
        driver = FakeConnectionDriver()
        connections = MotorConnections(driver)
        session = script_site.ScriptSession(connections)
        server = ThreadingHTTPServer(('127.0.0.1', 0), script_site.make_handler(session))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f'http://127.0.0.1:{server.server_port}'

        def post(action):
            request = urllib.request.Request(base + '/api/command', json.dumps({'action': action}).encode(),
                headers={'Content-Type': 'application/json', 'Origin': base})
            with urllib.request.urlopen(request, timeout=5) as response:
                return json.load(response)

        try:
            self.assertIn(post('connect')['connections']['test']['state'], ('connecting', 'connected'))
            connections.operation.result(timeout=5)
            with urllib.request.urlopen(base + '/api/status', timeout=5) as response:
                self.assertEqual(json.load(response)['connections']['load']['state'], 'connected')
            self.assertEqual(post('disconnect')['connections']['test']['state'], 'disconnected')
        finally:
            server.shutdown()
            server.server_close()

    def test_connect_both_stays_open_until_released(self):
        driver = FakeConnectionDriver()
        connections = MotorConnections(driver)
        self.assertEqual(connections.connect_both()['test']['state'], 'connecting')
        connections.operation.result(timeout=5)
        self.assertEqual(connections.status()['test']['state'], 'connected')
        self.assertEqual(connections.status()['load']['state'], 'connected')
        self.assertEqual(driver.events[:2], [('connect', '01'), ('connect', '02')])
        connections.stop_connected()
        self.assertIn(('stop', ('test', 'load')), driver.events)
        connections.release_all()
        self.assertEqual(connections.status()['test']['state'], 'disconnected')
        self.assertEqual(driver.events[-2:], [('release', '01'), ('release', '02')])

    def test_test_workflow_releases_idle_usb_connections(self):
        driver = FakeConnectionDriver()
        connections = MotorConnections(driver)
        connections.connect_both()
        connections.operation.result(timeout=5)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fake = root / 'fake_script.py'
            fake.write_text('print("started")\n', encoding='utf-8')
            with patch.multiple(script_site, ROOT=root, SCRIPT=fake,
                                RESULTS=root / 'results', STOP_REQUEST=root / 'site-estop.request'):
                session = script_site.ScriptSession(connections)
                session.start('matrix', dict(modes='sensored', rpms='600', loads='1', bandwidth='keep'))
                session.process.wait(timeout=5)
        self.assertEqual(connections.status()['test']['state'], 'disconnected')
        self.assertIn(('release', '02'), driver.events)

    def test_point_and_matrix_validation(self):
        self.assertEqual(script_site.point_args(dict(mode='sensored', rpm='600', amps='0.5', bandwidth='keep')),
                         ('sensored', 600., .5, 'keep'))
        self.assertEqual(script_site.matrix_args(dict(modes='sensored', rpms='600,800', loads='0,1', bandwidth='keep')),
                         ['--rpms', '600,800', '--loads', '0,1', '--modes', 'sensored', '--bandwidth', 'keep'])
        with self.assertRaises(ValueError):
            script_site.point_args(dict(mode='sensored', rpm='nan', amps='1'))
        with self.assertRaises(ValueError):
            script_site.matrix_args(dict(modes='sensored', rpms='600,broken', loads='1'))

    def test_point_prompt_is_visible_and_answered(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fake = root / 'fake_script.py'
            fake.write_text('import sys\n'
                'for i in range(4): input()\n'
                'sys.stdout.write("Ready to start the test? Press Enter or type yes; any other answer stops: ")\n'
                'sys.stdout.flush()\n'
                'print("ANSWER=" + input())\n', encoding='utf-8')
            with patch.multiple(script_site, ROOT=root, SCRIPT=fake,
                                RESULTS=root / 'results', STOP_REQUEST=root / 'site-estop.request'):
                session = script_site.ScriptSession()
                try:
                    session.start('point', dict(mode='sensored', rpm=600, amps=.5, bandwidth='keep'))
                    deadline = time.monotonic() + 5
                    while not session.status()['prompt'] and time.monotonic() < deadline:
                        time.sleep(.01)
                    self.assertIn('Ready to start the test?', session.status()['prompt'])
                    session.answer('yes')
                    while session.status()['running'] and time.monotonic() < deadline:
                        time.sleep(.01)
                    status = session.status()
                    self.assertEqual(status['exit_code'], 0)
                    self.assertIn('ANSWER=yes', '\n'.join(status['lines']))
                finally:
                    if session.process and session.process.poll() is None:
                        session.process.terminate()
                        session.process.wait(timeout=5)

    def test_live_trends_use_script_monitor_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            folder = root / 'session_1'
            folder.mkdir()
            row = {'host_monotonic_s': 4.2, 'stage': 'capturing',
                   'boards': {'test': {'position': 1, 'rpm': 600, 'phase_a': .1},
                              'load': {'position': 1, 'rpm': 601, 'phase_a': -.1}}}
            (folder / 'monitor.jsonl').write_text(json.dumps(row) + '\n', encoding='utf-8')
            with patch.object(script_site, 'RESULTS', root):
                result = script_site.live_samples()
            self.assertEqual(result['source'], 'session_1/monitor.jsonl')
            self.assertEqual(result['rows'][0]['boards']['load']['rpm'], 601)


if __name__ == '__main__':
    unittest.main()
