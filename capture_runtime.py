"""Finite onboard captures via the official synchronous ODrive helper.

Capture downloads run off the control thread so watchdog feeding and Stop remain
available. Each board has its own clock; paired files are never time-aligned.
"""
import copy
import math
import threading
import time
import uuid
import numpy as np


def channel_spec(role, board, profile):
    prefix='drive' if role=='test' else 'load'
    channels={
        'ia_a':('axis0.motor.alpha_beta_controller.current_meas_phA',1),
        'ib_a':('axis0.motor.alpha_beta_controller.current_meas_phB',1),
        'ic_a':('axis0.motor.alpha_beta_controller.current_meas_phC',1),
        prefix+'_dc_voltage_v':('vbus_voltage',1),
        prefix+'_dc_current_a':('ibus',1),
        prefix+'_rotor_rpm':('axis0.vel_estimate',60),
        prefix+'_position_turns':('axis0.pos_estimate',1)}
    if role=='test' and board.get('feedback_method')=='sensorless':
        if not profile.get('encoder_reference_verified') or not profile.get('encoder_reference_path'):
            raise ValueError('Sensorless capture requires a verified independent encoder angle path and scale.')
        channels['encoder_mech_rad']=(profile['encoder_reference_path'],profile['encoder_reference_scale_rad'])
        channels['estimated_electrical_rad']=('axis0.motor.sensorless_estimator.phase',1)
    return channels


def inspect_capture(device, role, board, profile, read):
    """Read capability evidence, without configuring/starting a capture."""
    try:
        channels=channel_spec(role,board,profile)
        rate=read(device,'control_loop_hz');size=read(device,'oscilloscope.size')
        ready=all(callable(read(device,'oscilloscope.'+name)) for name in ('config','trigger','get_raw'))
        ready=ready and read(device,'oscilloscope.trigger_pos') is not None
        ready=ready and isinstance(rate,(int,float)) and rate>0 and isinstance(size,(int,float)) and size>0
        missing=[path for path,_ in channels.values() if read(device,path) is None]
        count=int(size)//(4*len(channels)) if ready else 0
        return dict(available=bool(ready and not missing and count>=3),sample_rate_hz=rate,
            samples=count,window_s=count/rate if ready else None,channels=channels,
            detail=('Missing channels: '+', '.join(missing)) if missing else
                'Finite onboard buffer; per-board clock. Bandwidth must be declared separately.' if ready else
                'Compatible oscilloscope API and a readable control_loop_hz are required.')
    except (ValueError,TypeError) as exc:
        return dict(available=False,detail=str(exc))


def normalize_capture(raw, role, board, profile, plan):
    cap=board['capture'];rate=cap['sample_rate_hz'];channels=cap['channels']
    cycles=raw.get('timestamps',[])
    if len(cycles)<3 or any(len(raw.get(path,[]))!=len(cycles) for path,_ in channels.values()):
        raise ValueError('Capture returned missing channels or unequal lengths.')
    if any(not isinstance(t,(int,float)) or not math.isfinite(t) for t in cycles):raise ValueError('Invalid capture clock.')
    rows=[]
    for i,cycle in enumerate(cycles):
        row={'time_s':cycle/rate,'sample':i,'control_cycle':cycle,'control_stage':'closed_loop'}
        for key,(path,scale) in channels.items():
            value=raw[path][i]
            row[key]=float(value)*scale if isinstance(value,(int,float)) and math.isfinite(value) else None
        rows.append(row)
    prefix='drive' if role=='test' else 'load';position=prefix+'_position_turns'
    for row in rows:
        row[position+'_raw']=row[position]
        row[position]=row[position]-rows[0][position+'_raw'] if row[position] is not None and rows[0][position+'_raw'] is not None else None
    if role=='test' and 'encoder_mech_rad' in channels:
        vals=[r['encoder_mech_rad'] for r in rows]
        if all(v is not None for v in vals):
            unwrapped=np.unwrap(vals);rev=(unwrapped-unwrapped[0])/(2*math.pi)
            for row,v in zip(rows,rev):row['rotor_revolutions']=float(v)
    else:
        for row in rows:row['rotor_revolutions']=row[position]
    pp=board['configuration'].get('axis0.config.motor.pole_pairs')
    angles=[r.get('estimated_electrical_rad') for r in rows]
    if pp and all(v is not None for v in angles):
        unwrapped=np.unwrap(angles);rev=(unwrapped-unwrapped[0])/(2*math.pi*pp)
        for row,v in zip(rows,rev):row['estimated_revolutions']=float(v)
    meta={'source':'HARDWARE','role':role,'plan':copy.deepcopy(plan),'profile':copy.deepcopy(profile),
        'device':{k:copy.deepcopy(board.get(k)) for k in ('serial','firmware','configuration','provenance','feedback_method')},
        'calibration':copy.deepcopy(profile.get('calibration',{})), 'pole_pairs':pp,
        'channels':channels,'position_origin':'First sample of this capture; original axis values retained in *_raw.',
        'acquisition':{'source':'ODRIVE_ONBOARD','requested_hz':rate,'actual_hz':(len(rows)-1)/(rows[-1]['time_s']-rows[0]['time_s']) if rows[-1]['time_s']>rows[0]['time_s'] else None,
            'bandwidth_hz':profile.get('capture_bandwidth_hz'),'filtering':profile.get('capture_filtering',''),
            'timestamp_meaning':'Control-cycle index relative to this board trigger / reported control_loop_hz.',
            'synchronization':{'simultaneous':False,'method':'Same onboard buffer; inter-channel skew unverified; boards not synchronized','uncertainty_s':None},
            'expected_samples':cap['samples'],'partial':len(rows)!=cap['samples']}}
    return rows,meta


class CaptureService:
    def __init__(self, reader=None):
        self.reader=reader;self.lock=threading.Lock();self.thread=None;self.cancelled=False;self.result=None
        self.info={'state':'idle','id':None,'error':''}

    def status(self):
        with self.lock:return copy.deepcopy(self.info)

    def start(self,devices,boards,profile,plan):
        with self.lock:
            if self.thread and self.thread.is_alive():raise ValueError('A capture is still in progress.')
            for role in ('test','load'):
                if not boards[role].get('capture',{}).get('available'):raise ValueError(role+': high-rate capture unavailable.')
            self.info={'state':'capturing','id':uuid.uuid4().hex,'error':''};self.result=None;self.cancelled=False
            self.thread=threading.Thread(target=self._run,args=(dict(devices),copy.deepcopy(boards),copy.deepcopy(profile),copy.deepcopy(plan)),daemon=True)
            self.thread.start()

    def _run(self,devices,boards,profile,plan):
        outputs={};errors=[];output_lock=threading.Lock()
        def read_role(role):
            try:
                if self.reader:raw=self.reader(devices[role],boards[role]['capture'])
                else:
                    from odrive.utils import high_rate_capture,TimestampFmt
                    raw=high_rate_capture(devices[role],[p for p,_ in boards[role]['capture']['channels'].values()],
                        unsafe=False,trigger_point=0.,trigger_timeout=5.,return_as=dict,t_fmt=TimestampFmt.CONTROL_CYCLE)
                dataset=normalize_capture(raw,role,boards[role],profile,plan)
                with output_lock:outputs[role]=dataset
            except Exception as exc:
                with output_lock:errors.append(role+': '+str(exc))
        workers=[threading.Thread(target=read_role,args=(r,),daemon=True) for r in ('test','load')]
        started=time.monotonic()
        for thread in workers:thread.start()
        for thread in workers:thread.join(timeout=max(0,25-(time.monotonic()-started)))
        timed_out=any(t.is_alive() for t in workers)
        with output_lock,self.lock:
            if timed_out:errors.append('Capture download exceeded 25 seconds; reconnect before another capture.')
            if self.cancelled:errors.append('Capture interrupted by stop, disconnect, or lost steady state.')
            self.result={'datasets':copy.deepcopy(outputs),'errors':list(errors),'cancelled':self.cancelled}
            self.info.update(state='failed' if errors else 'complete',error='; '.join(errors))
        # Retain ownership while late downloads unwind; never overlap buffers.
        for thread in workers:thread.join()

    def cancel(self):
        with self.lock:
            if self.info['state']=='capturing':self.cancelled=True

    def take(self):
        with self.lock:
            result=self.result;self.result=None;return result
