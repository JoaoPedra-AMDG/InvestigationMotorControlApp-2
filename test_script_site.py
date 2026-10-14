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
        return {'serial': serial, 'state': 1, 'armed': False, 'errors': 0, 'disarm': 0,
                'watchdog_enabled': False}

    async def read(self, board, path):
        return {'axis0.current_state': board['state'], 'axis0.is_armed': board['armed'],
                'axis0.active_errors': board['errors'], 'axis0.disarm_reason': board['disarm'],
                'axis0.config.enable_watchdog': board['watchdog_enabled']}[path]

    async def checked_write(self, board, path, value):
        self.events.append(('write', board['serial'], path, value))
        if path == 'axis0.config.enable_watchdog':
            board['watchdog_enabled'] = value

    async def sample(self, board):
        return {'state': board['state'], 'armed': board['armed'], 'errors': board['errors'],
                'disarm': board['disarm'], 'position': 1.25, 'rpm': 600.0,
                'iq': .5, 'id': 0.0, 'phase_a': .2, 'phase_b': -.1, 'phase_c': -.1,
                'vbus': 24.0, 'ibus': .3, 'fet_C': 30.0}

    async def call(self, board, path):
        self.events.append(('call', board['serial'], path))
        if path == 'axis0.watchdog_feed' and not board['watchdog_enabled']:
            board['errors'] &= ~0x1000000
        if path == 'clear_errors':
            board['disarm'] = 0

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
            with urllib.request.urlopen(base + '/api/live', timeout=5) as response:
                self.assertEqual(json.load(response)['rows'][-1]['boards']['test']['rpm'], 600.0)
            self.assertIn('No active or recorded disarm errors remain.', post('clear_errors')['connection_notice'])
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
        self.assertEqual(connections.live()['rows'][-1]['boards']['test']['rpm'], 600.0)
        self.assertEqual(driver.events[:2], [('connect', '01'), ('connect', '02')])
        connections.stop_connected()
        self.assertIn(('stop', ('test', 'load')), driver.events)
        connections.release_all()
        self.assertEqual(connections.status()['test']['state'], 'disconnected')
        self.assertEqual(driver.events[-2:], [('release', '01'), ('release', '02')])

    def test_clear_errors_reports_active_faults_and_requires_idle(self):
        driver = FakeConnectionDriver()
        connections = MotorConnections(driver)
        connections.connect_both()
        connections.operation.result(timeout=5)
        try:
            connections.boards['test']['errors'] = 0x10
            connections.boards['test']['disarm'] = 0x20
            session = script_site.ScriptSession(connections)
            result = session.clear_errors()
            self.assertIn('active 0x10', result['connection_notice'])
            self.assertEqual(connections.boards['test']['disarm'], 0)
            connections.boards['load']['armed'] = True
            calls_before = len([event for event in driver.events if event[0] == 'call'])
            with self.assertRaisesRegex(ValueError, 'IDLE and disarmed'):
                session.clear_errors()
            self.assertEqual(len([event for event in driver.events if event[0] == 'call']), calls_before)
        finally:
            connections.release_all()

    def test_watchdog_recovery_requires_idle_and_clears_leftover_timeout(self):
        driver = FakeConnectionDriver()
        connections = MotorConnections(driver)
        connections.connect_both()
        connections.operation.result(timeout=5)
        try:
            test_board = connections.boards['test']
            load_board = connections.boards['load']
            for board in (test_board, load_board):
                board['errors'] = 0x1000000
                board['disarm'] = 0x1000000
                board['watchdog_enabled'] = True
            session = script_site.ScriptSession(connections)
            load_board['armed'] = True
            with self.assertRaisesRegex(ValueError, 'IDLE and disarmed'):
                session.recover_watchdogs()
            self.assertTrue(test_board['watchdog_enabled'])
            load_board['armed'] = False
            result = session.recover_watchdogs()
            self.assertIn('Idle watchdog disabled on: test, load', result['connection_notice'])
            self.assertIn('no active or recorded disarm errors', result['connection_notice'])
            self.assertFalse(test_board['watchdog_enabled'])
            self.assertFalse(load_board['watchdog_enabled'])
            self.assertIn(('write', '01', 'axis0.config.enable_watchdog', False), driver.events)
        finally:
            connections.release_all()

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

    def test_recent_manual_spectra_excludes_matrix_and_incomplete_captures(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name, finished in [('point_manual_old', 10), ('point_manual_new', 30),
                                   ('point_matrix', 40), ('point_incomplete', 50)]:
                folder = root / name
                folder.mkdir()
                (folder / 'capture.json').write_text(json.dumps({
                    'status': 'captured', 'point': {'mode': 'sensored', 'rpm': 600, 'amps': .5},
                    'finished_unix_s': finished}), encoding='utf-8')
                for artifact in ('analysis.json', 'motion.json', 'pair_psd_low_frequency.svg',
                                 'pair_psd_full_range.svg'):
                    if name != 'point_incomplete':
                        (folder / artifact).write_text('{}', encoding='utf-8')
            (root / 'matrix.json').write_text(json.dumps({'points': {
                'matrix-point': {'folder': 'point_matrix', 'status': 'complete'}}}), encoding='utf-8')
            with patch.object(script_site, 'RESULTS', root):
                runs = script_site.recent_manual_spectra()
            self.assertEqual([run['folder'] for run in runs], ['point_manual_new', 'point_manual_old'])
            self.assertEqual(runs[0]['rpm'], 600)


if __name__ == '__main__':
    unittest.main()
