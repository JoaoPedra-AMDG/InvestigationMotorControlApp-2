"""Offline integration checks. No test can discover or command a physical ODrive."""
import copy
import csv
import json
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path
from http.server import ThreadingHTTPServer
from app import Rig, make_handler
from hardware import HardwareController, DEFAULT_PROFILE, empty_board


def wait_for(predicate, timeout=5):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.01)
    raise AssertionError('Offline fixture did not reach the expected state.')


class SnapshotFixture:
    """Manually published known snapshots, available only in this test module."""
    def __init__(self):
        self.lock=threading.RLock();self.commands=[]
        self.data={'state':'DISCONNECTED','error':'','profile':copy.deepcopy(DEFAULT_PROFILE),
                   'boards':{role:empty_board() for role in ('test','load')},'readiness':[],
                   'control_ready':False,'sample_id':0,'acquired_at_s':None}
    def snapshot(self):
        with self.lock:return copy.deepcopy(self.data)
    def publish(self, sample, state='RUNNING', connected=True, age=0):
        with self.lock:
            now=time.perf_counter()-age
            self.data.update(state=state,sample_id=sample,acquired_at_s=now,control_ready=connected)
            for role in ('test','load'):
                board=self.data['boards'][role]
                board.update(connected=connected,state='CLOSED_LOOP_CONTROL',errors=1 if state=='FAULT' else 0,
                             sample_id=2*sample+(role=='load'),read_start_s=now-.001,read_end_s=now,
                             acquired_at_s=now,feedback_method='sensored',serial='TEST-ONLY-'+role,configuration={})
                board['signals'].update(dc_voltage_v=48.1 if role=='test' else 48.3,
                    dc_current_a=2.5 if role=='test' else -2.,speed_rpm=1000.,position_turns=sample/10,
                    torque_nm=.4 if role=='test' else -.2,iq_a=4. if role=='test' else -2.)
    def start(self,rpm,load_a,method):
        self.commands.append(('start',rpm,load_a,method))
        with self.lock:self.data['state']='STARTING'
        return self.snapshot()
    def stop(self):
        self.commands.append(('stop',))
        with self.lock:self.data['state']='CONNECTED'
        return self.snapshot()
    def close(self):self.commands.append(('close',))


class HardwareOnlyHTTPTests(unittest.TestCase):
    def test_disconnected_startup_endpoints_and_script_allowlist(self):
        with tempfile.TemporaryDirectory() as directory:
            attempted=[]
            def forbidden_connector(serial):
                attempted.append(serial)
                raise AssertionError('Physical discovery is forbidden in this test.')
            controller=HardwareController(connector=forbidden_connector)
            rig=Rig(directory,rate=50,controller=controller)
            server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(rig))
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            url=f'http://127.0.0.1:{server.server_port}'
            def get(path):
                with urllib.request.urlopen(url+path) as r:return json.load(r)
            def post(data, origin=None):
                req=urllib.request.Request(url+'/api/command',json.dumps(data).encode(),
                    {'Content-Type':'application/json','Origin':url if origin is None else origin})
                with urllib.request.urlopen(req) as r:return json.load(r)
            try:
                for path in ('/','/workbench.js','/style.css'):
                    with urllib.request.urlopen(url+path) as r:self.assertEqual(r.status,200)
                status=get('/api/status')
                self.assertEqual(status['source'],'HARDWARE')
                self.assertEqual(status['state'],'DISCONNECTED')
                self.assertFalse(status['recording']);self.assertIsNone(status['actual_hz'])
                for board in status['hardware']['boards'].values():
                    self.assertFalse(board['connected'])
                    self.assertTrue(all(value is None for value in board['signals'].values()))
                ready=get('/api/readiness')
                self.assertFalse(ready['record_allowed']);self.assertFalse(ready['motion_allowed'])
                self.assertEqual(get('/api/runs'),[]);self.assertEqual(get('/api/aggregate'),[])
                names={script['name'] for script in get('/api/scripts')}
                self.assertIn('hardware.py',names);self.assertNotIn('simulation.py',names)
                self.assertIn('HardwareController',get('/api/script?name=hardware.py')['content'])
                for path in ('/api/script?name=../app.py','/api/script?name=connection-profile.json'):
                    with self.assertRaises(urllib.error.HTTPError) as cm:get(path)
                    self.assertEqual(cm.exception.code,400)
                with self.assertRaises(urllib.error.HTTPError) as cm:post({'action':'plan'},'http://untrusted.test')
                self.assertEqual(cm.exception.code,403)
                point=post({'action':'plan','capture_high_rate':False,'rpm':1000,'load_a':2})
                post({'action':'select','id':point['id']})
                for action in ({'action':'record','allow_unsettled':True},{'action':'run_test','id':point['id']},
                               {'action':'connect'},{'action':'fault'},{'action':'communication_loss'}):
                    with self.assertRaises(urllib.error.HTTPError) as cm:post(action)
                    self.assertEqual(cm.exception.code,400)
                self.assertEqual(attempted,[])
                self.assertEqual(get('/api/runs'),[])
                with self.assertRaises(ValueError):rig.store.run_dir('../outside')
            finally:
                server.shutdown();server.server_close();thread.join();rig.close()


class RecorderWorkflowTests(unittest.TestCase):
    def test_real_adapter_contract_reaches_csv_and_stops(self):
        from test_hardware import FixtureConnector,valid_profile
        with tempfile.TemporaryDirectory() as directory:
            boards=FixtureConnector()
            controller=HardwareController(valid_profile(),connector=boards)
            rig=Rig(directory,rate=50,controller=controller)
            try:
                rig.action({'action':'connect'})
                object.__setattr__(boards.test.axis0,'vel_estimate',1000/60)
                object.__setattr__(boards.load.axis0.motor.foc,'Iq_measured',-2.)
                p=rig.action({'action':'plan','capture_high_rate':False,'duration_s':.2,'settle_s':0,'rpm':1000,'load_a':2})
                rig.action({'action':'run_test','id':p['id']})
                wait_for(lambda:rig.active_run is not None)
                wait_for(lambda:rig.store.run(rig.active_run)['status']!='recording')
                with Path(rig.last_file).open(newline='',encoding='utf-8') as f:rows=list(csv.DictReader(f))
                self.assertGreater(len(rows),1)
                self.assertTrue(all(r['fault']=='' for r in rows))
                self.assertEqual({r['feedback_mode'] for r in rows},{'sensored'})
                self.assertAlmostEqual(float(rows[0]['command_rpm']),1000.)
                self.assertAlmostEqual(float(rows[0]['load_command_a']),2.)
                self.assertEqual(boards.test.axis0.current_state,1)
                self.assertEqual(boards.load.axis0.current_state,1)
            finally:rig.close()

    def make_recording(self, rig, fixture):
        p=rig.action({'action':'plan','capture_high_rate':False,'test_type':'startup','duration_s':5,'rpm':1000,'load_a':2})
        rig.action({'action':'select','id':p['id']})
        fixture.publish(1);wait_for(lambda:rig.last_sample==1)
        rig.action({'action':'record'})
        return rig.active_run

    def test_csv_contains_only_new_snapshots_and_preserves_missing_channels(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture=SnapshotFixture();rig=Rig(directory,rate=100,controller=fixture)
            try:
                run_id=self.make_recording(rig,fixture)
                fixture.publish(2);wait_for(lambda:rig.sequence==1)
                time.sleep(.08)
                self.assertEqual(rig.sequence,1,'Repeated polling must not invent new readings.')
                fixture.publish(3);wait_for(lambda:rig.sequence==2)
                rig.action({'action':'end_record'})
                with Path(rig.last_file).open(newline='',encoding='utf-8') as f:rows=list(csv.DictReader(f))
                self.assertEqual(len(rows),2)
                self.assertEqual([int(r['sample']) for r in rows],[0,1])
                self.assertLess(float(rows[0]['time_s']),float(rows[1]['time_s']))
                self.assertEqual({r['source'] for r in rows},{'HARDWARE'})
                for row in rows:
                    self.assertEqual(float(row['drive_dc_voltage_v']),48.1)
                    self.assertEqual(float(row['load_dc_voltage_v']),48.3)
                    self.assertEqual(float(row['load_dc_current_a']),-2)
                    self.assertEqual(float(row['drive_rotor_rpm']),1000)
                    self.assertEqual(float(row['load_torque_nm']),-.2)
                    self.assertEqual(row['ia_a'],'');self.assertEqual(row['encoder_mech_rad'],'')
                    self.assertNotEqual(row['test_sample_id'],row['load_sample_id'])
                folder=rig.store.run_dir(run_id)
                meta=json.loads((folder/'metadata.json').read_text())
                self.assertEqual(meta['source'],'HARDWARE')
                self.assertFalse(meta['acquisition']['synchronization']['simultaneous'])
                self.assertTrue((folder/'software'/'hardware.py').exists())
                self.assertEqual(rig.store.run(run_id)['acquisition_status'],'recorded')
            finally:rig.close()

    def test_control_fault_and_acquisition_failure_remain_distinct(self):
        for disconnected in (False,True):
            with self.subTest(disconnected=disconnected),tempfile.TemporaryDirectory() as directory:
                fixture=SnapshotFixture();rig=Rig(directory,rate=100,controller=fixture)
                try:
                    run_id=self.make_recording(rig,fixture)
                    fixture.publish(2);wait_for(lambda:rig.sequence==1)
                    fixture.publish(3,state='DISCONNECTED' if disconnected else 'FAULT',connected=not disconnected)
                    wait_for(lambda:rig.store.run(run_id)['status']!='recording' and ('stop',) in fixture.commands)
                    run=rig.store.run(run_id)
                    self.assertEqual(run['status'],'invalid-acquisition' if disconnected else 'awaiting review')
                    self.assertEqual(run['control_outcome'],'not assessed' if disconnected else 'failure')
                    self.assertEqual(run['acquisition_status'],'invalid' if disconnected else 'recorded')
                    self.assertIn(('stop',),fixture.commands)
                finally:rig.close()

    def test_selected_point_records_then_stops_after_planned_duration(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture=SnapshotFixture();rig=Rig(directory,rate=100,controller=fixture)
            try:
                p=rig.action({'action':'plan','capture_high_rate':False,'duration_s':.1,'settle_s':0,'rpm':1000,'load_a':2})
                fixture.publish(1);wait_for(lambda:rig.last_sample==1)
                rig.action({'action':'run_test','id':p['id']})
                self.assertIn(('start',1000.,2.,'sensored'),fixture.commands)
                fixture.publish(2);wait_for(lambda:rig.recording is not None)
                run_id=rig.active_run
                fixture.publish(3);wait_for(lambda:rig.sequence==1)
                wait_for(lambda:not rig.automated_point and rig.store.run(run_id)['status']!='recording' and ('stop',) in fixture.commands)
                self.assertFalse(rig.automated_point)
                self.assertIn(('stop',),fixture.commands)
                self.assertEqual(rig.store.run(run_id)['status'],'awaiting review')
            finally:rig.close()


if __name__=='__main__':unittest.main()
