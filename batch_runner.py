"""One explicitly requested queue; never resumes motion on application restart."""
import copy
import time
from experiment import atomic_json,read_json


class BatchRunner:
    def __init__(self,rig):
        self.rig=rig;self.path=rig.output/'batch.json'
        self.data=read_json(self.path) if self.path.exists() else {'state':'idle','items':[],'current':None,'reason':''}
        self.wait_since=None
        if self.data['state'] in ('running','paused'):
            self.data.update(state='paused',reason='Application restarted. Review interrupted runs; motion will not resume automatically.')
            self.save()

    def save(self):atomic_json(self.path,self.data)
    def snapshot(self):return copy.deepcopy(self.data)
    def reserved(self):return self.data['state'] in ('running','paused')

    def start(self,ids):
        if self.reserved():raise ValueError('Cancel or resume the existing batch first.')
        if self.rig.starting or self.rig.recording or self.rig.automated_point or self.rig.capture_pending:raise ValueError('Finish the active test first.')
        plans=[self.rig.store.plan(i) for i in dict.fromkeys(ids)]
        if not plans:raise ValueError('No pending selected tests to run.')
        if any(p['status']!='pending' for p in plans):raise ValueError('Batch contains a previously run or skipped point.')
        self.data={'state':'running','current':None,'reason':'','items':[{'id':p['id'],'state':'queued','phase':'queued','run_id':None} for p in plans]}
        self.wait_since=None;self.save()
        return self.snapshot()

    def pause(self,reason):
        if self.reserved():self.data.update(state='paused',reason=reason);self.save()

    def cancel(self):
        if self.reserved():
            for item in self.data['items']:
                if item['state'] in ('queued','blocked','running'):item['state']='cancelled'
            self.data.update(state='cancelled',reason='Stopped by operator',current=None);self.save()

    def resume(self):
        if self.data['state']!='paused':raise ValueError('No paused batch.')
        if self.data['current'] is not None:raise ValueError('The interrupted point needs review and a new attempt. Cancel this queue first.')
        self.data.update(state='running',reason='');self.wait_since=None;self.save();return self.snapshot()

    def tick(self):
        rig=self.rig
        with rig.lock:
            if self.data['state']!='running':return
            current=self.data['current']
            if current:
                item=next(i for i in self.data['items'] if i['id']==current)
                if rig.active_run and rig.store.run(rig.active_run)['plan_id']==current:item['run_id']=rig.active_run
                if rig.automated_point or rig.starting or rig.capture_pending:
                    if item['phase']!=rig.run_phase:item['phase']=rig.run_phase;self.save()
                    return
                run=rig.store.run(item['run_id']) if item['run_id'] else None
                if rig.run_phase=='failed' or not run or run['acquisition_status']=='invalid' or run['control_outcome']=='failure':
                    item.update(state='failed',phase='failed');self.pause(rig.error or 'Test failed; review it and create a new attempt.');return
                item.update(state='recorded',phase='awaiting review');self.data['current']=None;self.wait_since=time.monotonic();self.save()
            item=next((i for i in self.data['items'] if i['state'] in ('queued','blocked')),None)
            if not item:self.data.update(state='complete',reason='All queued tests recorded; processing and review remain available.');self.save();return
            h=rig.hardware.snapshot()
            if not all(b.get('connected') for b in h['boards'].values()):
                item.update(state='blocked',phase='blocked');self.pause('Connect both boards before running the queue.');return
            if h['acquired_at_s'] is None or time.perf_counter()-h['acquired_at_s']>=1 or h['state']=='FAULT':
                item.update(state='blocked',phase='blocked');self.pause('Resolve stale feedback or board faults before resuming.');return
            stationary=all(b.get('state')=='IDLE' and b['signals'].get('speed_rpm') is not None and abs(b['signals']['speed_rpm'])<=5 for b in h['boards'].values())
            if not stationary:
                self.wait_since=self.wait_since or time.monotonic()
                if time.monotonic()-self.wait_since>30:self.pause('Both motors must report IDLE and less than 5 rpm before the next point.')
                return
            next_id=item['id']
        try:rig.action({'action':'run_test','id':next_id},from_batch=True)
        except Exception as exc:
            with rig.lock:
                if self.data['state']=='running':item.update(state='blocked',phase='blocked');self.pause(str(exc))
            return
        with rig.lock:
            if self.data['state']=='running':item.update(state='running',phase='settling');self.data['current']=next_id;self.wait_since=None;self.save()
