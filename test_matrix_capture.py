"""Offline checks for matrix sequencing, finite captures and ripple processing."""
import copy
import csv
import io
import math
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from app import Rig
from batch_runner import BatchRunner
from capture_runtime import CaptureService,channel_spec,normalize_capture
from experiment import Store,DEFAULT_PLAN
from processing import process_results,results_csv
from test_app import SnapshotFixture,wait_for


def capture_fixture(role='test',mode='sensored',n=1000):
    profile={'encoder_reference_verified':True,'encoder_reference_path':'inc_encoder0.pos_estimate',
        'encoder_reference_scale_rad':2*math.pi,'capture_bandwidth_hz':2000,'capture_filtering':'OFFLINE FIXTURE ONLY','test_direction':1}
    board={'feedback_method':mode,'configuration':{'axis0.config.motor.pole_pairs':6},'serial':'OFFLINE-'+role,'firmware':'OFFLINE'}
    channels=channel_spec(role,board,profile)
    board['capture']={'available':True,'channels':channels,'sample_rate_hz':8000,'samples':n,'window_s':n/8000}
    raw={'timestamps':list(range(n))}
    for name,(path,scale) in channels.items():
        if name in ('ia_a','ib_a','ic_a'):
            phase={'ia_a':0,'ib_a':-2*math.pi/3,'ic_a':2*math.pi/3}[name]
            raw[path]=[3*math.sin(2*math.pi*100*i/8000+phase)+.2*math.sin(2*math.pi*800*i/8000) for i in range(n)]
        elif name.endswith('rotor_rpm'):raw[path]=[1000/60]*n
        elif name=='estimated_electrical_rad':raw[path]=[(2*math.pi*100*i/8000+.4)%(2*math.pi) for i in range(n)]
        elif name=='encoder_mech_rad':raw[path]=[(1000/60*i/8000+.9)%1 for i in range(n)]
        elif name.endswith('position_turns'):raw[path]=[43+1000/60*i/8000 for i in range(n)]
        else:raw[path]=[48. if 'voltage' in name else 2.]*n
    return board,profile,raw


class CaptureTests(unittest.TestCase):
    def test_normalize_cycles_phase_channels_and_position_origins(self):
        board,profile,raw=capture_fixture(mode='sensorless')
        rows,meta=normalize_capture(raw,'test',board,profile,DEFAULT_PLAN)
        self.assertEqual(len(board['capture']['channels']),9)
        self.assertEqual(rows[0]['drive_position_turns'],0)
        self.assertEqual(rows[0]['drive_position_turns_raw'],43)
        self.assertEqual(rows[0]['rotor_revolutions'],0)
        self.assertEqual(rows[0]['estimated_revolutions'],0)
        self.assertAlmostEqual(rows[-1]['rotor_revolutions'],1000/60*999/8000)
        self.assertAlmostEqual(rows[-1]['estimated_revolutions'],1000/60*999/8000)
        self.assertAlmostEqual(rows[10]['drive_rotor_rpm'],1000)
        self.assertEqual(rows[10]['control_cycle'],10)
        self.assertEqual(rows[10]['time_s'],10/8000)
        self.assertFalse(meta['acquisition']['synchronization']['simultaneous'])
        self.assertFalse(meta['acquisition']['partial'])
        short={k:v[:-1] for k,v in raw.items()}
        self.assertTrue(normalize_capture(short,'test',board,profile,DEFAULT_PLAN)[1]['acquisition']['partial'])

    def test_capture_runs_off_thread_and_cancellation_keeps_raw_data(self):
        fixtures={r:capture_fixture(r) for r in ('test','load')};gate=threading.Event();entered=[]
        def reader(role,cap):
            entered.append(role);gate.wait(2);return fixtures[role][2]
        service=CaptureService(reader)
        try:
            service.start({r:r for r in fixtures},{r:f[0] for r,f in fixtures.items()},fixtures['test'][1],DEFAULT_PLAN)
            wait_for(lambda:len(entered)==2)
            self.assertEqual(service.status()['state'],'capturing')
            with self.assertRaises(ValueError):service.start({}, {}, {}, {})
            service.cancel();gate.set();wait_for(lambda:service.status()['state']=='failed')
            result=service.take();self.assertTrue(result['cancelled']);self.assertEqual(set(result['datasets']),{'test','load'})
        finally:gate.set();service.thread.join(3)

    def test_preflight_prevents_motion_and_zero_only_changes_display(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture=SnapshotFixture();rig=Rig(directory,rate=100,controller=fixture)
            try:
                fixture.publish(1);wait_for(lambda:rig.last_sample==1)
                p=rig.action({'action':'plan','capture_high_rate':True})
                with self.assertRaisesRegex(ValueError,'High-rate'):rig.action({'action':'run_test','id':p['id']})
                self.assertFalse(any(c[0]=='start' for c in fixture.commands))
                self.assertEqual(rig.status()['hardware']['boards']['test']['signals']['position_revolutions'],0)
                fixture.publish(2)
                status=rig.status();self.assertAlmostEqual(status['hardware']['boards']['test']['signals']['position_revolutions'],.1)
                self.assertEqual(status['hardware']['boards']['test']['signals']['speed_rpm'],1000)
                self.assertEqual(rig.action({'action':'zero_position'})['hardware']['boards']['test']['signals']['position_revolutions'],0)
                self.assertEqual(fixture.snapshot()['boards']['test']['signals']['position_turns'],.2)
            finally:rig.close()


    def test_high_rate_workflow_saves_both_buffers_and_stops(self):
        class CapturingFixture(SnapshotFixture):
            def __init__(self):super().__init__();self.result=None
            def start_capture(self,plan):
                datasets={}
                for role in ('test','load'):
                    board,profile,raw=capture_fixture(role)
                    datasets[role]=normalize_capture(raw,role,board,profile,plan)
                self.result={'datasets':datasets,'errors':[],'cancelled':False}
            def capture_result(self):result=self.result;self.result=None;return result
            def cancel_capture(self):self.commands.append(('cancel_capture',))
        with tempfile.TemporaryDirectory() as directory:
            fixture=CapturingFixture();rig=Rig(directory,rate=100,controller=fixture)
            try:
                fixture.publish(1)
                for role in ('test','load'):fixture.data['boards'][role]['capture']=capture_fixture(role)[0]['capture']
                wait_for(lambda:rig.last_sample==1)
                p=rig.action({'action':'plan','duration_s':.3,'settle_s':0})
                rig.action({'action':'run_test','id':p['id']});fixture.publish(2)
                wait_for(lambda:rig.recording is not None)
                sample=3;deadline=time.monotonic()+3
                while rig.recording and time.monotonic()<deadline:
                    fixture.publish(sample);sample+=1;time.sleep(.02)
                with rig.lock:
                    self.assertIsNone(rig.recording)
                    run=rig.store.run(rig.active_run)
                self.assertEqual(run.get('capture',{}).get('state'),'complete',(run,rig.error))
                self.assertEqual(run['status'],'awaiting review',run.get('reason'))
                self.assertEqual(len(run['datasets']),2)
                for dataset in run['datasets']:
                    rows,meta=rig.store.load_dataset(run['id'],dataset)
                    self.assertEqual(len(rows),1000)
                    self.assertEqual(rows[0]['rotor_revolutions'],0)
                    self.assertEqual(meta['acquisition']['requested_hz'],8000)
                self.assertIn(('stop',),fixture.commands)
            finally:rig.close()


class QueueFixture:
    def __init__(self,path):
        self.output=Path(path);self.store=Store(path);self.lock=threading.RLock()
        self.starting=False;self.recording=None;self.automated_point=False;self.capture_pending=None
        self.active_run=None;self.run_phase='idle';self.error='';self.commands=[];self.speed=0;self.connected=True
        self.hardware=SimpleNamespace(snapshot=self.snapshot)
        self.batch=BatchRunner(self)
    def snapshot(self):
        return {'state':'CONNECTED','acquired_at_s':time.perf_counter(),
                'boards':{r:{'connected':self.connected,'state':'IDLE','signals':{'speed_rpm':self.speed}} for r in ('test','load')}}
    def action(self,data,**kwargs):
        p=self.store.plan(data['id'])
        if p['method']=='sensorless':raise ValueError('Sensorless startup unavailable')
        self.commands.append(p['id']);self.automated_point=True;self.run_phase='settling'
    def finish(self,p,invalid=False):
        run=self.store.create_run(p,{'source':'TEST_FIXTURE','plan':p});self.active_run=run['id']
        run.update(status='invalid-acquisition' if invalid else 'awaiting review',acquisition_status='invalid' if invalid else 'recorded');self.store.update(run)
        self.automated_point=False;self.run_phase='failed' if invalid else 'review'


class BatchTests(unittest.TestCase):
    def test_sequence_waits_for_stop_and_reports_progress(self):
        with tempfile.TemporaryDirectory() as directory:
            rig=QueueFixture(directory);points=[rig.store.edit_plan({'rpm':r}) for r in (1000,1500)]
            rig.batch.start([p['id'] for p in points]);rig.batch.tick()
            self.assertEqual(rig.commands,[points[0]['id']])
            rig.finish(points[0]);rig.speed=20;rig.batch.tick()
            self.assertEqual(rig.batch.data['items'][0]['state'],'recorded');self.assertEqual(len(rig.commands),1)
            rig.speed=0;rig.batch.tick();self.assertEqual(rig.commands,[p['id'] for p in points])
            rig.finish(points[1]);rig.batch.tick();self.assertEqual(rig.batch.data['state'],'complete')

    def test_unsupported_point_pauses_and_restart_never_autostarts(self):
        with tempfile.TemporaryDirectory() as directory:
            rig=QueueFixture(directory);p=rig.store.edit_plan({'method':'sensorless'})
            rig.batch.start([p['id']]);rig.batch.tick()
            self.assertEqual(rig.batch.data['state'],'paused');self.assertIn('Sensorless',rig.batch.data['reason']);self.assertEqual(rig.commands,[])
            restored=QueueFixture(directory);restored.batch.tick();self.assertEqual(restored.commands,[])
            restored.batch.cancel();restored.batch.tick();self.assertEqual(restored.batch.data['state'],'cancelled')

    def test_failure_and_disconnect_do_not_start_next_test(self):
        with tempfile.TemporaryDirectory() as directory:
            rig=QueueFixture(directory);points=[rig.store.edit_plan({'rpm':r}) for r in (1000,1500)]
            rig.batch.start([p['id'] for p in points]);rig.connected=False;rig.batch.tick()
            self.assertEqual(rig.batch.data['state'],'paused');self.assertEqual(rig.commands,[])
            rig.connected=True;rig.batch.resume();rig.batch.tick();rig.finish(points[0],True);rig.batch.tick()
            self.assertEqual(rig.batch.data['state'],'paused');self.assertEqual(len(rig.commands),1)


class ProcessingTests(unittest.TestCase):
    def add(self,store,p,n=1000,bandwidth=2000,invalid=False):
        board,profile,raw=capture_fixture(mode=p['method'],n=n);profile['capture_bandwidth_hz']=bandwidth
        rows,meta=normalize_capture(raw,'test',board,profile,p)
        # This emulates the hardware storage contract entirely inside a temporary
        # unit-test folder; fixture identity is retained in every metadata file.
        meta['test_fixture_only']=True
        run=store.create_run(p,meta)
        run.update(status='invalid-acquisition' if invalid else 'awaiting review',acquisition_status='invalid' if invalid else 'recorded')
        store.update(run);store.add_dataset(run['id'],'capture_test',rows,meta)
        return run,rows,meta

    def test_both_metrics_common_windows_retry_dedup_and_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            store=Store(directory);plans=[store.edit_plan({'method':m}) for m in ('sensored','sensorless')]
            for p in plans:self.add(store,p,n=1200 if p['method']=='sensored' else 1000)
            latest=self.add(store,plans[0],n=1200)[0]
            result=process_results(store,{})
            self.assertEqual(len(result['rows']),2,result['excluded'])
            self.assertIn(latest['id'],{r['run_id'] for r in result['rows']})
            self.assertEqual(len({r['comparison_key'] for r in result['rows']}),1)
            for r in result['rows']:
                self.assertAlmostEqual(r['peak_to_peak'],.4*math.sin(2*math.pi/5),delta=.003)
                self.assertAlmostEqual(r['abs_peak'],.2*math.sin(2*math.pi/5),delta=.003)
            exported=list(csv.DictReader(io.StringIO(results_csv(store).decode())))
            self.assertEqual(len(exported),2);self.assertIn('abs_peak',exported[0])

    def test_incompatible_acquisitions_and_invalid_runs_are_separated(self):
        with tempfile.TemporaryDirectory() as directory:
            store=Store(directory)
            for bandwidth in (2000,1500):self.add(store,store.edit_plan({}),bandwidth=bandwidth)
            self.add(store,store.edit_plan({}),invalid=True)
            self.add(store,store.edit_plan({}),bandwidth=None)
            result=process_results(store,{})
            self.assertEqual(len(result['rows']),2);self.assertEqual(len({r['comparison_key'] for r in result['rows']}),2)
            self.assertEqual(len(result['excluded']),2)

    def test_processing_restart_and_interrupted_capture_excluded(self):
        with tempfile.TemporaryDirectory() as directory:
            store=Store(directory);run,rows,meta=self.add(store,store.edit_plan({}))
            run=store.run(run['id']);run['status']='processing';store.update(run)
            restored=Store(directory)
            self.assertEqual(restored.run(run['id'])['status'],'invalid-acquisition')
            self.assertEqual(process_results(restored,{})['rows'],[])


if __name__=='__main__':unittest.main()
