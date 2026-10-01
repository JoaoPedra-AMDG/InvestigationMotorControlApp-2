"""Guided, bounded velocity-PI trials on an already commissioned two-motor rig.

Only the test motor's velocity PI gains are swept. The load motor can have a
diagnostic baseline, but current-loop gains are not inferred from host polling.
All proposed gains are restored before a recommendation is presented.
"""
import copy
import math
import threading
import time
import uuid
from datetime import datetime, timezone

from experiment import atomic_json


GAIN_PATHS = ('axis0.controller.config.vel_gain', 'axis0.controller.config.vel_integrator_gain')


def trial_metrics(samples, target_rpm, target_load_a):
    if len(samples) < 10:
        raise ValueError('Too few fresh samples to assess the tuning trial.')
    settled = samples[len(samples)//2:]
    speeds = [abs(float(s['drive_rpm'])) for s in settled]
    iq = [abs(float(s['load_iq_a'])) for s in settled]
    if not all(math.isfinite(v) for v in speeds + iq):
        raise ValueError('A trial sample is missing or invalid.')
    errors = [v-target_rpm for v in speeds]
    mean = sum(speeds)/len(speeds)
    rms = math.sqrt(sum(e*e for e in errors)/len(errors))
    jitter = math.sqrt(sum((v-mean)**2 for v in speeds)/len(speeds))
    overshoot = max(0.,max(abs(float(s['drive_rpm'])) for s in samples)-target_rpm)
    load_error = math.sqrt(sum((v-target_load_a)**2 for v in iq)/len(iq))
    return dict(speed_rms_error_rpm=rms, speed_sd_rpm=jitter, overshoot_rpm=overshoot,
        load_iq_rms_error_a=load_error, mean_speed_rpm=mean,
        score=rms+2*overshoot+jitter, samples=len(samples))


class TuningSession:
    def __init__(self, rig):
        self.rig=rig
        self.lock=threading.RLock()
        self.cancel_event=threading.Event()
        self.thread=None
        self.data={'state':'idle','task':'Check readiness to begin.','trials':[]}

    def snapshot(self):
        with self.lock:return copy.deepcopy(self.data)

    def _set(self, **changes):
        with self.lock:
            self.data.update(changes)
            if self.data.get('id'):
                atomic_json(self.rig.output/'tuning'/self.data['id']/'session.json',self.data)

    def prepare(self, role, rpm, load_a):
        with self.rig.lock:
            if self.rig.starting or self.rig.recording or self.rig.automated_point or self.rig.capture_pending or self.rig.batch.reserved():
                raise ValueError('Finish the active test or batch before tuning.')
            h=self.rig.hardware.snapshot()
            if not h['control_ready'] or h['state']!='CONNECTED':
                raise ValueError('Connect both boards and resolve every motion-readiness check before tuning.')
            if any(b.get('feedback_method')!='sensored' for b in h['boards'].values()):
                raise ValueError('This guided tuning session requires both motors commissioned in sensored mode.')
            if role not in ('test','load'):raise ValueError('Choose the test or load motor.')
            rpm=float(rpm);load_a=float(load_a)
            profile=h['profile']
            if not math.isfinite(rpm) or not 0<rpm<=profile['max_speed_rpm']:
                raise ValueError('Trial speed must fit the validated rig maximum.')
            if not math.isfinite(load_a) or not 0<=load_a<=profile['max_load_a']:
                raise ValueError('Trial load must fit the validated rig maximum.')
            gains={path:h['boards']['test']['configuration'].get(path) for path in GAIN_PATHS}
            if role=='test' and any(not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0 for v in gains.values()):
                raise ValueError('Positive test velocity PI gains must be readable before automated trials.')
            with self.lock:
                if self.thread and self.thread.is_alive():raise ValueError('A tuning session is already running.')
                ident=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_')+uuid.uuid4().hex[:8]
                self.data=dict(id=ident,state='awaiting_operator',role=role,rpm=rpm,load_a=load_a,
                    board_serials={r:b['serial'] for r,b in h['boards'].items()},baseline_gains=gains,
                    current_gains=gains,proposed_gains=None,trials=[],
                    task='Check that the coupled shafts are clear, guards are fitted, both USB isolators are installed, and the physical stop is reachable. Stay at the rig during all trials.',
                    note='The site will run bounded trials and restore the original gains before presenting a recommendation.')
                folder=self.rig.output/'tuning'/ident;folder.mkdir(parents=True,exist_ok=False)
                atomic_json(folder/'session.json',self.data)
        return self.snapshot()

    def start(self, physical_checked):
        if self.snapshot()['state']!='awaiting_operator':
            raise ValueError('Check readiness again before starting a tuning session.')
        with self.rig.lock:
            if self.rig.starting or self.rig.recording or self.rig.automated_point or self.rig.capture_pending or self.rig.batch.reserved():
                raise ValueError('Another test owns the rig.')
            h=self.rig.hardware.snapshot()
            if not h['control_ready'] or h['state']!='CONNECTED':
                raise ValueError('Board readiness changed. Recheck the tuning session.')
            if any(h['boards'][role]['serial']!=serial for role,serial in self.data['board_serials'].items()):
                raise ValueError('Connected board identity changed. Recheck the tuning session.')
            if any(h['boards']['test']['configuration'].get(path)!=self.data['baseline_gains'][path] for path in GAIN_PATHS):
                raise ValueError('Velocity gains changed after baseline. Recheck the tuning session.')
        with self.lock:
            if self.data['state']!='awaiting_operator':raise ValueError('Check readiness again before starting a tuning session.')
            if physical_checked is not True:raise ValueError('Complete and confirm the physical rig task shown above.')
            self.cancel_event.clear()
            self._set(state='running',task='Running a bounded trial. Stay at the rig; Stop both is always available.')
            self.thread=threading.Thread(target=self._run,daemon=True,name='guided-tuning')
            self.thread.start()
        return self.snapshot()

    def cancel(self):
        state=self.snapshot()['state']
        if state=='awaiting_operator':
            self._set(state='cancelled',task='Tuning cancelled before motion. No gains were changed.')
            return self.snapshot()
        if state not in ('running','stopping','restoring'):
            raise ValueError('No tuning trial is running.')
        self.cancel_event.set()
        self.rig.hardware.stop()
        self._set(state='stopping',task='Stop requested. Waiting for both motors to report IDLE and coast below 1 rpm.')
        return self.snapshot()

    def _wait_stopped(self, timeout=90):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            h=self.rig.hardware.snapshot()
            if h['state']=='CONNECTED' and all(b.get('state_code')==1 and
                    isinstance(b['signals'].get('speed_rpm'),(int,float)) and
                    abs(b['signals']['speed_rpm'])<1 for b in h['boards'].values()):return
            time.sleep(.1)
        raise TimeoutError('Both shafts were not verified stopped within 90 seconds.')

    def _one_trial(self, factor):
        role=self.data['role'];baseline=self.data['baseline_gains'];rpm=self.data['rpm'];load=self.data['load_a']
        proposed={p:baseline[p]*factor for p in GAIN_PATHS}
        if role=='test':
            self.rig.hardware.set_tuning_gains(self.data['current_gains'],proposed,baseline)
            self._set(current_gains=proposed)
        self._set(task=f'Trial {len(self.data["trials"])+1}: {rpm:g} rpm, {load:g} A; gain factor {factor:g}.')
        self.rig.hardware.start(rpm,load,'sensored')
        deadline=time.monotonic()+self.rig.hardware.snapshot()['profile']['startup_timeout_s']+20
        while time.monotonic()<deadline:
            if self.cancel_event.is_set():raise InterruptedError('Operator stopped tuning.')
            h=self.rig.hardware.snapshot()
            if h['state']=='FAULT' or h['sample_age_s'] is None or h['sample_age_s']>1:
                raise RuntimeError('Board fault or stale measurements during startup.')
            if h['state']=='RUNNING':break
            time.sleep(.1)
        else:raise TimeoutError('The trial did not reach the requested speed and load.')
        samples=[];end=time.monotonic()+8;last_id=None
        while time.monotonic()<end:
            if self.cancel_event.is_set():raise InterruptedError('Operator stopped tuning.')
            h=self.rig.hardware.snapshot()
            if h['state']!='RUNNING' or h['sample_age_s'] is None or h['sample_age_s']>1:
                raise RuntimeError('Board state or telemetry became unavailable during tuning.')
            if h['sample_id']!=last_id:
                last_id=h['sample_id']
                test=h['boards']['test']['signals'];load_board=h['boards']['load']['signals']
                row=dict(time_s=time.monotonic(),drive_rpm=test.get('speed_rpm'),
                    drive_iq_a=test.get('iq_a'),load_iq_a=load_board.get('iq_a'))
                if any(v is None or not isinstance(v,(int,float)) or not math.isfinite(v) for v in row.values()):
                    raise RuntimeError('A required tuning measurement is unavailable.')
                samples.append(row)
            time.sleep(.05)
        self.rig.hardware.stop();self._wait_stopped()
        metrics=trial_metrics(samples,rpm,load)
        trial=dict(factor=factor,gains=proposed if role=='test' else None,metrics=metrics,samples=samples)
        self._set(trials=[*self.data['trials'],trial])
        if metrics['overshoot_rpm']>max(30,rpm*.2) or metrics['speed_sd_rpm']>max(15,rpm*.1):
            raise RuntimeError('Excessive overshoot or speed variation; no further gain trials attempted.')

    def _run(self):
        completed=False
        try:
            for factor in ((1.,.9,1.1) if self.data['role']=='test' else (1.,)):
                if self.cancel_event.is_set():raise InterruptedError('Operator stopped tuning.')
                self._one_trial(factor)
            if self.data['role']=='test':
                original=self.data['trials'][0]
                best=min(self.data['trials'],key=lambda t:t['metrics']['score'])
                if best['metrics']['score']>original['metrics']['score']*.9:best=original
                recommendation=best['gains']
                note='The lowest bounded-trial speed-response score is recommended only if it improves on baseline by at least 10%. Inspect the plots before applying.'
            else:
                recommendation=None
                note='Load current tracking was measured. Current-controller gains were not changed because host polling cannot identify a safe current-loop setting.'
            self._set(proposed_gains=recommendation,note=note,state='restoring',task='Restoring starting gains before review.')
            completed=True
        except Exception as exc:
            self._set(state='stopping',task='Stopping after trial issue: '+str(exc),note=str(exc))
        finally:
            try:
                self.rig.hardware.stop();self._wait_stopped()
                if self.data['role']=='test' and self.data['current_gains']!=self.data['baseline_gains']:
                    self.rig.hardware.set_tuning_gains(self.data['current_gains'],self.data['baseline_gains'],self.data['baseline_gains'])
                    self._set(current_gains=self.data['baseline_gains'])
                self._set(state='cancelled' if self.cancel_event.is_set() else 'review' if completed else 'failed',
                    task='Review the recorded trials and recommendation. Original gains are active.')
            except Exception as exc:
                self._set(state='needs_attention',task='The motors or gain restoration could not be verified. Inspect both boards before further motion.',note=str(exc))

    def apply_recommendation(self):
        with self.lock:
            if self.data['state']!='review' or self.data['role']!='test' or not self.data['proposed_gains']:
                raise ValueError('No reviewed velocity-gain recommendation is available.')
            proposed=self.data['proposed_gains'];baseline=self.data['baseline_gains'];expected=self.data['current_gains']
        self.rig.hardware.set_tuning_gains(expected,proposed,baseline)
        self._set(current_gains=proposed,state='applied',task='Suggested gains are active temporarily. Run a validation test before saving to the board.')
        return self.snapshot()

    def restore(self):
        with self.lock:
            if self.thread and self.thread.is_alive():raise ValueError('Wait for trials to stop before restoring.')
            if self.data.get('role')!='test':raise ValueError('No velocity gains were changed.')
            expected=self.data['current_gains'];baseline=self.data['baseline_gains']
        self.rig.hardware.set_tuning_gains(expected,baseline,baseline)
        self._set(current_gains=baseline,state='review',task='Original gains restored.')
        return self.snapshot()

    def save(self):
        with self.lock:
            if self.data['state']!='applied':
                raise ValueError('Apply and validate recommended gains before saving to the test board.')
            expected=dict(self.data['current_gains'])
        self.rig.hardware.save_tuning_gains(expected)
        self._set(state='saved',task='Test-board gains saved. Reconnect and verify both boards before motion.')
        return self.snapshot()
