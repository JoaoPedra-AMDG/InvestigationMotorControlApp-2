"""ODrive motor workbench: real boards only, explicit operator motion commands."""
import argparse
import copy
import json
import math
import shutil
import sys
import threading
import time
from collections import deque
from datetime import datetime,timezone
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit,parse_qs
from hardware import HardwareController,DEFAULT_PROFILE
from experiment import Store,Recorder,DEFAULT_PLAN,atomic_json,read_json
from import_data import import_dataset
from signals import UNITS
from batch_runner import BatchRunner
from tuning import TuningSession

ROOT=Path(__file__).resolve().parent
SCRIPTS={'app.py':'Web server, run workflow and CSV acquisition','hardware.py':'ODrive connection, state checks and explicit motor commands',
 'experiment.py':'Run files, recordings, review history and ZIP exports','analysis.py':'Angle, current and timing analysis',
 'capture_helpers.py':'Optional onboard capture integration helper (hardware capability verification required)',
 'import_data.py':'Oscilloscope / DAQ CSV import and channel provenance','signals.py':'Canonical measurement units',
 'capture_runtime.py':'Finite onboard capture and original per-board timestamps','batch_runner.py':'Explicit batch queue and restart recovery',
 'spectrum.py':'Phase-current PSD and desired-frequency comparison',
 'tuning.py':'Guided bounded velocity-PI trials and response assessment',
 'processing.py':'Ripple processing and load/speed comparisons','settings_support.py':'Validated board settings for troubleshooting'}


class Rig:
    def __init__(self,output=ROOT/'recordings',rate=20,controller=None):
        self.output=Path(output);self.rate=rate;self.store=Store(self.output)
        self.lock=threading.RLock();self.shutdown=threading.Event()
        self.profile_path=self.output/'connection-profile.json'
        profile=read_json(self.profile_path) if self.profile_path.exists() else dict(DEFAULT_PROFILE)
        self.hardware=controller or HardwareController(profile)
        self.selected=None;self.recording=None;self.active_run=None;self.latest={};self.last_file='';self.error=''
        self.run_phase='idle';self.automated_point=False;self.command=0.;self.load=0.;self.mode=None
        self.starting=False;self.command_epoch=0
        self.capture_pending=None;self.position_zero={}
        self.batch=BatchRunner(self)
        self.tuning=TuningSession(self)
        self.sequence=0;self.last_sample=None;self.settle_history=deque(maxlen=10000);self.sample_times=deque(maxlen=100)
        self.worker=threading.Thread(target=self.loop,daemon=True);self.worker.start()

    def plan(self):return self.store.plan(self.selected) if self.selected else dict(DEFAULT_PLAN,id='manual')
    def settled(self):
        with self.lock:
            p=self.plan();now=time.perf_counter();hold=p['settle_s'];window=[r for r in self.settle_history if now-r['host_perf_s']<=hold+.1]
            profile=self.hardware.snapshot()['profile']
            return bool(window and now-window[0]['host_perf_s']>=hold and now-window[-1]['host_perf_s']<.5 and
                all(r.get('drive_rotor_rpm') is not None and r.get('load_iq_a') is not None and
                    abs(r['drive_rotor_rpm']-p['rpm']*profile['test_direction'])<=p['settle_rpm'] and
                    abs(r['load_iq_a']-p['load_a']*profile['load_direction'])<=p['settle_load_a'] and r['state']=='RUNNING' for r in window))
    def status(self):
        with self.lock:
            h=self.hardware.snapshot();actual=(len(self.sample_times)-1)/(self.sample_times[-1]-self.sample_times[0]) if len(self.sample_times)>1 and self.sample_times[-1]>self.sample_times[0] else None
            for role,b in h['boards'].items():
                raw=b['signals'].get('position_turns')
                if b['connected'] and raw is not None:
                    self.position_zero.setdefault(role,raw)
                    b['signals']['position_revolutions']=raw-self.position_zero[role]
                else:b['signals']['position_revolutions']=None
            return dict(self.latest,source='HARDWARE',state=h['state'],hardware=h,error=self.error or h['error'],
                selected=self.selected,recording=bool(self.recording),active_run=self.active_run,run_phase=self.run_phase,
                requested_hz=self.rate,actual_hz=actual,settled=self.settled(),file=self.last_file,
                telemetry_age_s=time.perf_counter()-h['acquired_at_s'] if h['acquired_at_s'] is not None else None,
                queue_depth=self.recording.queue.qsize() if self.recording else 0,recovery=self.store.recovery,
                batch=self.batch.snapshot(),capture_pending=bool(self.capture_pending),
                tuning_state=self.tuning.snapshot()['state'])
    def readiness(self):
        h=self.hardware.snapshot();checks=copy.deepcopy(h['readiness'])
        storage=shutil.disk_usage(self.output).free>50_000_000
        checks.append({'name':'Storage','state':'pass' if storage else 'blocked','detail':'At least 50 MB free for recording.'})
        checks.append({'name':'Settling','state':'pass' if self.settled() else 'waiting','detail':'Speed and load current must remain inside the selected run tolerances.'})
        for role,board in h['boards'].items():
            cap=board.get('capture',{})
            checks.append({'name':role.title()+' high-rate capture','state':'pass' if cap.get('available') else 'blocked','detail':cap.get('detail','Connect to check onboard capture compatibility.')})
        connected=all(b['connected'] for b in h['boards'].values())
        fresh=h['acquired_at_s'] is not None and time.perf_counter()-h['acquired_at_s']<1
        return {'checks':checks,'hardware':h,'settled':self.settled(),'record_allowed':connected and fresh and storage,
                'motion_allowed':h['control_ready']}
    def event(self,name,data=None):
        if self.active_run:self.store.event(self.active_run,name,data or {})

    def _row(self,h):
        b=h['boards'];test=b['test']['signals'];load=b['load']['signals'];cfg=b['test']['configuration']
        row=dict.fromkeys(UNITS)
        for prefix,signals in [('drive',test),('load',load)]:
            for key,ending in [('dc_voltage_v','dc_voltage_v'),('dc_current_a','dc_current_a'),('speed_rpm','rotor_rpm'),
                               ('position_turns','position_turns'),('torque_nm','torque_nm'),('motor_temp_c','motor_temp_c'),('controller_temp_c','controller_temp_c')]:row[prefix+'_'+ending]=signals.get(key)
        feedback=b['test'].get('feedback_method',b['test'].get('feedback'))
        estimated_rpm=test.get('sensorless_speed_rpm') if feedback=='sensorless' else None
        faults=[]
        for role,board in b.items():
            errors=board.get('errors')
            if isinstance(errors,dict):
                if any(v not in (0,None) for v in errors.values()):faults.append(role+':'+str(errors))
            elif errors not in (0,None):faults.append(role+':'+str(errors))
        row.update(command_rpm=test.get('speed_command_rpm'),encoder_rpm=test.get('encoder_speed_rpm'),estimated_rpm=estimated_rpm,
                   encoder_mech_rad=test.get('encoder_mech_rad'),estimated_electrical_rad=test.get('estimated_electrical_rad') if feedback=='sensorless' else None,
                   ia_a=test.get('ia_a'),ib_a=test.get('ib_a'),ic_a=test.get('ic_a'),drive_id_a=test.get('id_a'),drive_iq_a=test.get('iq_a'),
                   id_command_a=test.get('id_command_a'),iq_command_a=test.get('iq_command_a'),load_iq_a=load.get('iq_a'),load_command_a=self.load,
                   fault=';'.join(faults) or None,
                   state=h['state'],control_stage=h.get('control_stage') or ('closed_loop' if h['state']=='RUNNING' else h['state']))
        load_kt=b['load']['configuration'].get('axis0.config.motor.torque_constant')
        load_torque=load.get('torque_command_nm')
        row['load_command_a']=load_torque/load_kt*h['profile']['load_direction'] if load_torque is not None and load_kt and load_kt>0 else None
        now=h['acquired_at_s'];row.update(source='HARDWARE',feedback_mode=feedback,host_perf_s=now,monotonic_ns=int(now*1e9),
           test_read_start_s=b['test']['read_start_s'],test_read_end_s=b['test']['read_end_s'],load_read_start_s=b['load']['read_start_s'],load_read_end_s=b['load']['read_end_s'],
           test_sample_id=b['test']['sample_id'],load_sample_id=b['load']['sample_id'],time_s=now)
        return row

    def begin_record(self,allow_unsettled=False):
        if self.starting:raise ValueError('Wait for the start command to finish.')
        if self.recording:raise ValueError('A recording is already active.')
        if not self.selected:raise ValueError('Select a test-matrix point first.')
        p=self.plan();ready=self.readiness();h=self.hardware.snapshot()
        if p['status']!='pending':raise ValueError('Create a new attempt instead of replacing an earlier run.')
        if not ready['record_allowed'] or not self.latest:raise ValueError('Connect both boards and resolve acquisition readiness first.')
        if h['boards']['test'].get('feedback_method')!=p['method'] or h['boards']['load'].get('feedback_method')!='sensored':
            raise ValueError('Board feedback routing must match the selected test; the load must remain sensored.')
        if p['test_type']=='steady state' and not self.settled() and not allow_unsettled:raise ValueError('Wait until the selected speed and load have settled.')
        if p.get('capture_high_rate',False):
            self.capture_preflight(p,h)
            if not self.settled():raise ValueError('High-rate steady-state capture requires settled speed and load.')
        self.run_zero=time.perf_counter();self.run_started_host=self.run_zero;self.sequence=0
        meta={'schema_version':3,'source':'HARDWARE','software_version':'3.0.0','python_version':sys.version,
          'plan':dict(p),'devices':h['boards'],'profile':h['profile'],'calibration':h['profile'].get('calibration',{}),
          'acquisition':{'source':'HOST_TELEMETRY','requested_hz':self.rate,'bandwidth_hz':None,'filtering':'Firmware report filtering; see saved board configuration. No interpolation.',
             'synchronization':{'simultaneous':False,'method':'Sequential USB reads; each board read interval retained','uncertainty_s':None}},
          'channel_units':UNITS,'initial_conditions':self.latest,'readiness_at_start':ready,'unsettled_override':bool(allow_unsettled),
          'load_definition':'Opposing q-axis current command in A, converted through configured load Kt to the ODrive torque input.',
          'torque_definition':'Configured Kt multiplied by firmware-reported Iq; not independently measured shaft torque.',
          'timestamp_meaning':'time_s relative to recorder start on host perf_counter clock; read intervals are absolute host clock seconds.'}
        run=self.store.create_run(p,meta);self.active_run=run['id'];self.last_file=str(self.store.run_dir(run['id'])/'telemetry.csv')
        software=self.store.run_dir(run['id'])/'software';software.mkdir()
        for name in SCRIPTS:shutil.copyfile(ROOT/name,software/name)
        self.recording=Recorder(self.store.run_dir(run['id']),list(self.record_row(self.latest)))
        p['status']='recording';self.store.save_plans();self.run_phase='recording'
        if p.get('capture_high_rate',False):
            try:self.hardware.start_capture(dict(p))
            except Exception as exc:
                self.hardware.stop();self.automated_point=False;self.end_record(failure='Capture could not start: '+str(exc));raise
            self.capture_pending={'run_id':run['id'],'plan_id':p['id']};self.run_phase='capturing'
    def record_row(self,row):return dict(row,time_s=row['host_perf_s']-self.run_zero,elapsed_s=row['host_perf_s']-self.run_zero,sample=self.sequence)
    def end_record(self,reason='operator finished',failure=None,control_failure=False):
        if not self.recording:return
        if self.capture_pending and (failure or control_failure):self.hardware.cancel_capture()
        recorder=self.recording;self.recording=None;integrity=recorder.close();run=self.store.run(self.active_run)
        bad=failure or integrity['error'] or (run.get('capture',{}).get('error') or None)
        pending=bool(self.capture_pending)
        run.update(status='invalid-acquisition' if bad else 'processing' if pending else 'awaiting review',acquisition_status='invalid' if bad else 'recorded',
          reason=bad or reason,recording_integrity=integrity,finished_utc=datetime.now(timezone.utc).isoformat())
        if control_failure:run['control_outcome']='failure'
        self.store.update(run);self.plan().update(status=run['status'],reason=run['reason']);self.store.save_plans()
        self.event('recording_finished',{'reason':reason,'failure':bad,'integrity':integrity});self.run_phase='failed' if bad else 'processing' if pending else 'review'

    def capture_preflight(self,p,h):
        for role,b in h['boards'].items():
            cap=b.get('capture',{})
            if not cap.get('available'):raise ValueError(role+': '+cap.get('detail','High-rate capture has not been verified on this board.'))
            if p['duration_s']<cap['window_s']+.1:raise ValueError('Recording duration must cover the full onboard capture window plus 0.1 seconds.')

    def collect_capture(self):
        if not self.capture_pending:return
        result=self.hardware.capture_result()
        if result is None:return
        pending=self.capture_pending;self.capture_pending=None;errors=list(result['errors']);datasets=[]
        errors.extend(role+': incomplete onboard buffer' for role,(_,meta) in result['datasets'].items() if meta['acquisition'].get('partial'))
        for role,(rows,meta) in result['datasets'].items():
            meta['capture_interrupted']=bool(errors)
            datasets.append(self.store.add_dataset(pending['run_id'],'capture_'+role,rows,meta))
            if not errors:
                try:
                    from spectrum import analyze_spectrum
                    atomic_json(self.store.run_dir(pending['run_id'])/f'spectrum-{role}.json',analyze_spectrum(rows,meta))
                except (ValueError,TypeError,KeyError) as exc:
                    self.store.event(pending['run_id'],'spectrum_unavailable',{'role':role,'reason':str(exc)})
        run=self.store.run(pending['run_id']);run['capture']={'state':'failed' if errors else 'complete','datasets':datasets,'error':'; '.join(errors)}
        if errors:run.update(status='invalid-acquisition',acquisition_status='invalid',reason='; '.join(errors))
        elif run['status']=='processing':run['status']='awaiting review'
        self.store.update(run);self.store.event(run['id'],'high_rate_capture_finished',run['capture'])
        if errors:
            self.hardware.stop();self.error='; '.join(errors);self.automated_point=False
            if self.recording:self.end_record(failure=self.error)
            self.run_phase='failed'
        elif not self.recording:self.run_phase='failed' if run['acquisition_status']=='invalid' or run['control_outcome']=='failure' else 'review'
        plan=self.store.plan(pending['plan_id']);plan.update(status=run['status'],reason=run.get('reason',''));self.store.save_plans()

    def action(self,data,*,from_batch=False):
        action=data.get('action')
        tuning_active=self.tuning.snapshot()['state'] in ('awaiting_operator','running','stopping','restoring')
        if action=='tuning_prepare':return self.tuning.prepare(data.get('role'),data.get('rpm'),data.get('load_a'))
        if action=='tuning_start':return self.tuning.start(data.get('physical_checked'))
        if action=='tuning_cancel':return self.tuning.cancel()
        if action in ('tuning_apply','tuning_restore','tuning_save'):
            with self.lock:
                if self.starting or self.recording or self.automated_point or self.capture_pending or self.batch.reserved():
                    raise ValueError('Finish the active test or batch before changing tuning gains.')
            return {'tuning_apply':self.tuning.apply_recommendation,
                    'tuning_restore':self.tuning.restore,'tuning_save':self.tuning.save}[action]()
        if tuning_active and action in ('batch_start','batch_resume','run_test','start','record','configure_connection',
            'refresh_settings','preview_settings','apply_settings','update_limits','clear_errors'):
            raise ValueError('Finish or cancel the tuning session before this action.')
        if action in ('batch_start','batch_resume','batch_cancel'):
            if action=='batch_cancel':
                with self.lock:self.batch.cancel()
                return self.action({'action':'stop'})
            with self.lock:
                if action=='batch_resume':return self.batch.resume()
                ids=data.get('ids',[p['id'] for p in self.store.plans if p['selected'] and p['status']=='pending'])
                return self.batch.start(ids)
        if action=='zero_position':
            with self.lock:self.position_zero={}
            return self.status()
        if self.batch.reserved() and not from_batch and action in ('run_test','start','select','plan','skip','repeat','record','subset','matrix'):
            raise ValueError('The batch owns the test matrix. Cancel the batch before changing tests or using manual controls.')
        if action in ('refresh_settings','preview_settings','apply_settings','update_limits'):
            with self.lock:
                if self.starting or self.recording or self.automated_point or self.capture_pending or self.batch.reserved():
                    raise ValueError('Finish the active test or cancel the batch before changing settings.')
                if action=='refresh_settings':return self.hardware.refresh_settings()
                if action=='preview_settings':return self.hardware.preview_settings(data['values'])
                if action=='update_limits':
                    result=self.hardware.update_limits(data['max_speed_rpm'],data['max_load_a'])
                    atomic_json(self.profile_path,result['profile']);return result
                try:return self.hardware.apply_settings(data.get('token'),data.get('persist',False))
                finally:
                    result=self.hardware.snapshot().get('settings_result')
                    if result:
                        with (self.output/'settings-history.jsonl').open('a',encoding='utf-8') as f:
                            f.write(json.dumps({'utc':datetime.now(timezone.utc).isoformat(),**result})+'\n')
        if action=='configure_connection':
            result=self.hardware.configure(data['profile']);atomic_json(self.profile_path,result['profile'])
            return result
        if action=='connect':
            result=self.hardware.connect()
            with self.lock:self.error='';self.latest={};self.position_zero={};self.settle_history.clear();self.sample_times.clear();self.last_sample=None
            return result
        if action=='disconnect':
            self.tuning.cancel_event.set()
            if self.tuning.snapshot()['state']=='awaiting_operator':
                self.tuning._set(state='cancelled',task='Connection released before tuning began.')
            with self.lock:self.batch.pause('Boards disconnected. Reconnect and explicitly resume.');self.hardware.cancel_capture()
            with self.lock:self.command_epoch+=1;self.automated_point=False
            result=self.hardware.disconnect()
            with self.lock:self.end_record(failure='Operator disconnected during recording' if self.recording else None)
            return result
        if action=='clear_errors':
            result=self.hardware.clear_errors()
            with self.lock:self.error=''
            return result
        if action=='stop':
            self.tuning.cancel_event.set()
            if self.tuning.snapshot()['state']=='awaiting_operator':
                self.tuning._set(state='cancelled',task='Tuning cancelled before motion.')
            with self.lock:self.batch.cancel();self.command_epoch+=1;self.automated_point=False
            if self.capture_pending:self.hardware.cancel_capture()
            result=self.hardware.stop()
            with self.lock:self.automated_point=False;self.event('operator_stop');self.end_record('Operator stopped motors');self.run_phase='stopped'
            return result
        if action=='end_record':
            with self.lock:
                stop_needed=self.automated_point;self.automated_point=False
                if self.capture_pending:self.hardware.cancel_capture()
            if stop_needed:self.hardware.stop()
            with self.lock:self.end_record()
            return self.status()
        if action in ('start','run_test'):
            with self.lock:
                if from_batch and self.batch.data['state']!='running':raise ValueError('Batch cancelled before start.')
                if self.starting:raise ValueError('A start command is already in progress.')
                if self.capture_pending:raise ValueError('Wait for the capture download to finish.')
                if action=='run_test':
                    if self.recording or self.automated_point:raise ValueError('Finish or stop the current run first.')
                    if data.get('id'):self.selected=data['id']
                    p=self.plan()
                    if not self.selected or p['status']!='pending':raise ValueError('Select a pending test-matrix point.')
                    if p['test_type']!='steady state':raise ValueError('Use manual Apply and Record for transients, startup and minimum-speed investigations.')
                    if p.get('capture_high_rate',False):self.capture_preflight(p,self.hardware.snapshot())
                    rpm,load_a,method=p['rpm'],p['load_a'],p['method']
                else:
                    rpm=float(data['rpm']);load_a=float(data['load_a']);method=data['mode']
                    if self.automated_point:raise ValueError('Stop the selected-point workflow before applying different conditions.')
                self.starting=True;epoch=self.command_epoch
            try:result=self.hardware.start(rpm,load_a,method)
            except Exception:
                with self.lock:self.starting=False
                raise
            with self.lock:
                self.starting=False
                if epoch!=self.command_epoch:raise ValueError('Start workflow cancelled by Stop or Disconnect.')
                self.error='';self.command,self.load,self.mode=rpm,load_a,method;self.settle_history.clear();self.event('operator_applied_conditions',{'rpm':rpm,'load_a':load_a,'method':method})
                if action=='run_test':self.automated_point=True;self.run_phase='settling';self.point_start=time.perf_counter();self.position_zero={}
            return result
        with self.lock:
            if action=='matrix':return {'created':len(self.store.generate(data))}
            if action=='plan':
                if (self.starting or self.recording or self.automated_point) and data.get('id')==self.selected:raise ValueError('Stop the active workflow before editing its test point.')
                return self.store.edit_plan(data)
            if action=='select':
                if self.starting or self.recording or self.automated_point:raise ValueError('Stop the active workflow before selecting another point.')
                p=self.store.plan(data['id']);self.selected=p['id'];self.settle_history.clear();return p
            if action=='subset':
                ids=set(data['ids'])
                for p in self.store.plans:p['selected']=p['id'] in ids
                self.store.save_plans();return {'selected':len(ids)}
            if action=='skip':
                if (self.starting or self.recording or self.automated_point) and self.selected in data['ids']:raise ValueError('Stop the active workflow before skipping its test point.')
                self.store.skip(data['ids'],data.get('reason',''));return {'ok':True}
            if action=='repeat':
                if self.starting or self.recording or self.automated_point:raise ValueError('Finish the active workflow first.')
                old=self.store.run(data['run_id']);p=self.store.edit_plan({k:v for k,v in old['plan'].items() if k!='id'})
                p['pair_id']=old['plan'].get('pair_id');p['retry_of']=old['id'];self.store.save_plans();self.selected=p['id'];return p
            if action=='record':self.begin_record(data.get('allow_unsettled') is True)
            elif action=='process_results':
                from processing import process_results
                if self.recording or self.capture_pending or self.automated_point or self.batch.data['state']=='running':raise ValueError('Finish the active tests before processing results.')
                return process_results(self.store,data.get('settings',{}))
            elif action=='review':return self.store.review(data['run_id'],data.get('dataset','telemetry'),data.get('settings',{}))
            elif action=='spectrum':
                from spectrum import analyze_spectrum
                run_id=data['run_id'];dataset=data['dataset']
                if dataset not in ('capture_test','capture_load'):
                    raise ValueError('Choose an original test or load capture.')
                rows,metadata=self.store.load_dataset(run_id,dataset)
                result=analyze_spectrum(rows,metadata)
                atomic_json(self.store.run_dir(run_id)/f'spectrum-{metadata["role"]}.json',result)
                return result
            elif action=='accept':
                result=self.store.finish_review(data['run_id'],data['outcome'],data.get('reason',''))
                if not self.recording and not self.automated_point:
                    self.selected=next((p['id'] for p in self.store.plans if p['selected'] and p['status']=='pending'),None)
                return result
            elif action=='note':self.store.event(data['run_id'],'operator_note',{'note':str(data['note'])[:10000]});return {'ok':True}
            elif action=='import':return import_dataset(self.store,data['run_id'],data)
            else:raise ValueError('Unknown action.')
            return self.status()

    def loop(self):
        while not self.shutdown.wait(1/self.rate):
            stop_needed=False
            try:
                with self.lock:
                    self.collect_capture()
                    h=self.hardware.snapshot();now=time.perf_counter()
                    connected=all(b['connected'] for b in h['boards'].values())
                    fresh=h['acquired_at_s'] is not None and now-h['acquired_at_s']<1
                    if (self.recording or self.automated_point) and (not connected or not fresh or h['state']=='FAULT'):
                        self.hardware.stop()
                        if self.recording:self.end_record(failure='Communication unavailable or stale' if not connected or not fresh else None,
                            reason='Board control failure',control_failure=h['state']=='FAULT')
                        self.automated_point=False;self.error=h.get('error') or 'Board data is unavailable';self.run_phase='failed';stop_needed=True
                    if connected and fresh and h['sample_id']!=self.last_sample:
                        self.last_sample=h['sample_id'];row=self._row(h);self.latest=row;self.sample_times.append(row['host_perf_s']);self.settle_history.append(row)
                        if self.recording:
                            if not self.recording.push(self.record_row(row)):
                                self.hardware.stop()
                                self.error=self.recording.error or 'Recorder rejected a sample';self.end_record(failure=self.error);self.automated_point=False;stop_needed=True
                            self.sequence+=1
                    if self.automated_point and self.run_phase=='settling':
                        if self.settled():self.begin_record()
                        elif now-self.point_start>60:
                            self.error='Selected point did not settle within 60 seconds.';self.automated_point=False;self.run_phase='failed';stop_needed=True
                            self.plan().update(status='unsuccessful',reason=self.error);self.store.save_plans()
                    if self.recording and now>=self.run_started_host and self.plan().get('capture_high_rate',False) and not self.settled():
                        self.hardware.stop();self.end_record(failure='Speed or load left the steady-state tolerance during capture.');self.automated_point=False
                    if self.recording and now-self.run_started_host>=self.plan()['duration_s']:
                        if self.capture_pending:self.hardware.cancel_capture()
                        if self.automated_point:self.hardware.stop()
                        self.end_record('Planned duration reached');self.automated_point=False
                    if not connected or not fresh:
                        self.latest={};self.settle_history.clear();self.sample_times.clear()
                if stop_needed:self.hardware.stop()
                self.batch.tick()
            except Exception as exc:
                try:self.hardware.stop()
                except Exception:pass
                with self.lock:
                    self.error='Acquisition/workflow failure: '+str(exc);self.automated_point=False;self.run_phase='failed'
                    try:self.end_record(failure=self.error)
                    except Exception:pass
    def close(self):
        self.tuning.cancel_event.set()
        self.shutdown.set();self.worker.join(timeout=12)
        self.hardware.close()
        with self.lock:self.end_record(failure='Application closed during recording' if self.recording else None)


def make_handler(rig):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,code,body,ctype='application/json',filename=None):
            payload=body if isinstance(body,bytes) else json.dumps(body,allow_nan=False).encode()
            self.send_response(code);self.send_header('Content-Type',ctype);self.send_header('Content-Length',str(len(payload)))
            self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff')
            if filename:self.send_header('Content-Disposition',f'attachment; filename="{filename}"')
            self.end_headers();self.wfile.write(payload)
        def do_GET(self):
            try:
                u=urlsplit(self.path);q={k:v[0] for k,v in parse_qs(u.query).items()}
                if u.path=='/':return self.send(200,(ROOT/'index.html').read_bytes(),'text/html; charset=utf-8')
                if u.path in ('/workbench.js','/style.css'):return self.send(200,(ROOT/u.path[1:]).read_bytes(),'text/javascript' if u.path.endswith('.js') else 'text/css')
                if u.path=='/api/status':return self.send(200,rig.status())
                if u.path=='/api/tuning':return self.send(200,rig.tuning.snapshot())
                if u.path=='/api/readiness':return self.send(200,rig.readiness())
                if u.path=='/api/scripts':return self.send(200,[{'name':n,'description':d} for n,d in SCRIPTS.items()])
                if u.path=='/api/script':
                    name=q['name']
                    if name not in SCRIPTS:raise ValueError('Script not available.')
                    return self.send(200,{'name':name,'content':(ROOT/name).read_text(encoding='utf-8-sig')})
                with rig.lock:
                    if u.path=='/api/plans':return self.send(200,rig.store.plans)
                    if u.path=='/api/runs':return self.send(200,rig.store.list_runs())
                    if u.path=='/api/results':
                        from processing import results
                        return self.send(200,results(rig.store))
                    if u.path=='/api/spectra':
                        run_dir=rig.store.run_dir(q['id'])
                        rig.store.run(q['id'])
                        return self.send(200,{role:read_json(run_dir/f'spectrum-{role}.json')
                            for role in ('test','load') if (run_dir/f'spectrum-{role}.json').exists()})
                    if u.path=='/results.csv':
                        from processing import results_csv
                        return self.send(200,results_csv(rig.store),'text/csv; charset=utf-8','ripple-results.csv')
                    if u.path=='/api/run':return self.send(200,dict(rig.store.run(q['id']),metadata=read_json(rig.store.run_dir(q['id'])/'metadata.json')))
                    if u.path=='/api/aggregate':return self.send(200,rig.store.aggregate(q.get('metric','angle_error_deg.rms')))
                    if u.path=='/api/data':
                        rows,meta=rig.store.load_dataset(q['id'],q.get('dataset','telemetry'));step=max(1,math.ceil(len(rows)/2500))
                        return self.send(200,{'rows':rows[::step],'metadata':meta,'original_samples':len(rows),'display_stride':step,'alignment':'Original dataset clock; no interpolation'})
                    if u.path=='/download':return self.send(200,rig.store.bundle(q['id']),'application/zip',q['id']+'.zip')
                self.send(404,{'error':'Not found'})
            except (ValueError,KeyError,StopIteration,OSError) as exc:self.send(400,{'error':str(exc) or 'Not found'})
        def do_POST(self):
            if self.path!='/api/command':return self.send(404,{'error':'Not found'})
            host=self.headers.get('Host','')
            if host not in {f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}'} or self.headers.get('Origin')!='http://'+host or self.headers.get('Content-Type')!='application/json':
                return self.send(403,{'error':'Use the local dashboard.'})
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=10_000_000:raise ValueError('Request exceeds the 10 MB limit.')
                data=json.loads(self.rfile.read(size))
                if not isinstance(data,dict):raise ValueError('Expected an object.')
                self.send(200,rig.action(data))
            except (ValueError,KeyError,TypeError,StopIteration,RuntimeError,TimeoutError) as exc:self.send(400,{'error':str(exc) or 'Not found'})
            except OSError as exc:self.send(500,{'error':'Storage/communication failure: '+str(exc)})
    return Handler


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--rate',type=int,default=20,choices=range(1,101),metavar='1..100');args=parser.parse_args()
    # Bind the server before opening storage to prevent a second process recovering an active run.
    server=ThreadingHTTPServer(('127.0.0.1',args.port),BaseHTTPRequestHandler)
    rig=Rig(rate=args.rate);server.RequestHandlerClass=make_handler(rig)
    print(f'ODRIVE HARDWARE WORKBENCH — http://127.0.0.1:{args.port}',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close();rig.close()
