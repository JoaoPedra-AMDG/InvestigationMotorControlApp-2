"""Durable plans, immutable raw datasets, versioned reviews and bounded recording."""
import csv
import hashlib
import io
import json
import math
import os
import queue
import shutil
import statistics
import threading
import time
import uuid
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from analysis import analyze

def uid(prefix):
    return prefix+'_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'_'+uuid.uuid4().hex[:12]

def atomic_json(path, data):
    path=Path(path)
    tmp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        with tmp.open('x',encoding='utf-8') as f:
            json.dump(data,f,indent=2,allow_nan=False)
            f.flush();os.fsync(f.fileno())
        # Windows readers and sync clients can briefly hold a destination handle.
        # Keep the old complete JSON until the atomic replacement succeeds.
        for attempt in range(7):
            try:
                os.replace(tmp,path);break
            except PermissionError:
                if attempt==6:raise
                time.sleep(.01*2**attempt)
    finally:
        if tmp.exists(): tmp.unlink()

def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))

def finite(value, low, high, name):
    value=float(value)
    if not math.isfinite(value) or not low<=value<=high:
        raise ValueError(f'{name} must be between {low} and {high}.')
    return value

DEFAULT_PLAN={'method':'sensored','rpm':1000.,'load_a':2.,'test_type':'steady state','repeat':1,
              'duration_s':5.,'settle_rpm':10.,'settle_load_a':.2,'settle_s':1.,'notes':'','selected':True,'capture_high_rate':True}
TYPES=['steady state','speed change','load disturbance','startup','minimum-speed investigation']

def validate_plan(data):
    p=dict(DEFAULT_PLAN,**{k:v for k,v in data.items() if k in DEFAULT_PLAN})
    if p['method'] not in ('sensored','sensorless') or p['test_type'] not in TYPES:
        raise ValueError('Unknown method or test type.')
    for name,lo,hi in [('rpm',0,100000),('load_a',0,1000),('duration_s',.1,3600),('settle_rpm',.01,10000),('settle_load_a',.001,100),('settle_s',0,60)]:
        p[name]=finite(p[name],lo,hi,name)
    p['repeat']=int(finite(p['repeat'],1,100,'repeat'))
    p['notes']=str(p['notes'])[:4000]
    p['selected']=bool(p['selected'])
    if not isinstance(p['capture_high_rate'],bool):raise ValueError('High-rate capture choice must be true or false.')
    return p


class Recorder:
    def __init__(self, folder, fields, capacity=2048):
        self.folder=Path(folder); self.fields=fields
        self.queue=queue.Queue(maxsize=capacity)
        self.finish=threading.Event();self.done=threading.Event()
        self.error=None;self.dropped=0;self.written=0;self.accepted=0
        self.file=(self.folder/'telemetry.csv').open('x',newline='',encoding='utf-8')
        self.thread=threading.Thread(target=self._write,daemon=True)
        self.thread.start()

    def push(self,row):
        if self.finish.is_set() or self.error: return False
        try:
            self.queue.put_nowait(dict(row));self.accepted+=1;return True
        except queue.Full:
            self.dropped+=1;self.error='Acquisition queue overflow';self.finish.set();return False

    def _write(self):
        try:
            writer=csv.DictWriter(self.file,fieldnames=self.fields,extrasaction='raise')
            writer.writeheader();last=time.perf_counter()
            while not self.finish.is_set() or not self.queue.empty():
                try: row=self.queue.get(timeout=.1)
                except queue.Empty: continue
                try:
                    writer.writerow(row);self.written+=1
                    if time.perf_counter()-last>.5:
                        self.file.flush();os.fsync(self.file.fileno());last=time.perf_counter()
                finally: self.queue.task_done()
            self.file.flush();os.fsync(self.file.fileno())
        except Exception as exc:
            self.error='Storage failure: '+str(exc);self.finish.set()
        finally:
            try:self.file.close()
            except OSError as exc:self.error='Storage close failure: '+str(exc)
            self.done.set()

    def close(self):
        self.finish.set();self.thread.join(timeout=10)
        if not self.done.is_set():self.error='Writer did not finish within 10 seconds'
        return {'written_samples':self.written,'accepted_samples':self.accepted,'dropped_samples':self.dropped,'error':self.error}


class Store:
    def __init__(self, root):
        self.root=Path(root);self.root.mkdir(parents=True,exist_ok=True)
        self.runs=self.root/'runs';self.runs.mkdir(exist_ok=True)
        self.lock=threading.RLock()
        self.plan_file=self.root/'plan.json'
        self.plans=read_json(self.plan_file) if self.plan_file.exists() else []
        self.recovery=[]
        for path in self.runs.glob('*/run.json'):
            try:
                run=read_json(path)
                if run['status'] in ('recording','processing'):
                    run.update(status='invalid-acquisition',acquisition_status='interrupted',reason='Application interrupted before recording/capture finalization')
                    atomic_json(path,run);self.event(run['id'],'recovered_interrupted_session',{})
                    for plan in self.plans:
                        if plan['id']==run.get('plan_id'):plan.update(status=run['status'],reason=run['reason'])
                    self.save_plans()
            except (ValueError,KeyError,OSError) as exc:
                self.recovery.append({'folder':str(path.parent),'error':str(exc)})

    def save_plans(self): atomic_json(self.plan_file,self.plans)
    def plan(self,id):
        return next(p for p in self.plans if p['id']==id)
    def run_dir(self,id):
        if not isinstance(id,str) or not id.startswith('run_') or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_' for c in id):
            raise ValueError('Invalid run identifier.')
        folder=self.runs/id
        if not folder.is_dir():raise ValueError('Run not found.')
        return folder
    def run(self,id):return read_json(self.run_dir(id)/'run.json')
    def update(self,run):atomic_json(self.run_dir(run['id'])/'run.json',run)
    def event(self,id,event,data):
        with self.lock:
            with (self.run_dir(id)/'events.jsonl').open('a',encoding='utf-8') as f:
                f.write(json.dumps({'utc':datetime.now(timezone.utc).isoformat(),'event':event,'data':data},allow_nan=False)+'\n');f.flush()
    def list_runs(self):
        runs=[]
        for path in self.runs.glob('*/run.json'):
            try:runs.append(read_json(path))
            except (ValueError,OSError):continue
        return sorted(runs,key=lambda r:r['id'],reverse=True)
    def generate(self,data):
        speed_base=finite(data.get('speed_base',2000),1,100000,'Speed base')
        load_base=finite(data.get('load_base',4),.001,1000,'Load base')
        levels=data.get('speeds',[.1,.3,.5,.7,.9]); loads=data.get('loads',[0,.25,.5,.9])
        if len(levels)!=5 or len(loads)!=4:raise ValueError('Provide five speed fractions and four load fractions.')
        levels=[finite(x,0,1,'Speed fraction') for x in levels];loads=[finite(x,0,1,'Load fraction') for x in loads]
        batch=uid('matrix');new=[]
        for i,speed in enumerate(levels):
            for j,load in enumerate(loads):
                for repeat in range(1,4):
                    methods=['sensored','sensorless'] if (i*4+j+repeat)%2 else ['sensorless','sensored']
                    pair=uid('pair')
                    for method in methods:
                        p=validate_plan(dict(data,rpm=speed*speed_base,load_a=load*load_base,method=method,repeat=repeat))
                        p.update(id=uid('plan'),pair_id=pair,matrix_id=batch,status='pending',reason='')
                        new.append(p)
        with self.lock:self.plans.extend(new);self.save_plans()
        return new
    def edit_plan(self,data):
        with self.lock:
            if data.get('id'):
                p=self.plan(data['id'])
                if p['status'] not in ('pending','skipped'):raise ValueError('Recorded conditions are immutable. Create a repeat instead.')
                p.update(validate_plan(dict(p,**data)))
            else:
                p=validate_plan(data);p.update(id=uid('plan'),pair_id=uid('pair'),status='pending',reason='');self.plans.append(p)
            self.save_plans();return p
    def skip(self,ids,reason):
        if not str(reason).strip():raise ValueError('A reason is required.')
        with self.lock:
            chosen=[self.plan(id) for id in ids]
            if any(p['status'] not in ('pending','skipped') for p in chosen):raise ValueError('Only unrecorded plans can be skipped.')
            for p in chosen:p.update(status='skipped',reason=str(reason)[:4000],selected=False)
            self.save_plans()
    def create_run(self,plan,metadata):
        id=uid('run');folder=self.runs/id;folder.mkdir(exist_ok=False)
        run={'id':id,'plan_id':plan['id'],'plan':dict(plan),'source':metadata['source'],'status':'recording',
             'control_outcome':'not assessed','acquisition_status':'recording','reason':'','datasets':[],
             'reviews':[],'latest_review':None,'created_utc':datetime.now(timezone.utc).isoformat()}
        atomic_json(folder/'metadata.json',metadata);atomic_json(folder/'run.json',run)
        self.event(id,'recording_started',{'plan':plan})
        return run
    def add_dataset(self,id,name,rows,meta):
        folder=self.run_dir(id)/'datasets';folder.mkdir(exist_ok=True)
        dataset=uid(name);csvpath=folder/(dataset+'.csv')
        fields=list(rows[0]) if rows else ['time_s']
        with csvpath.open('x',newline='',encoding='utf-8') as f:
            writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows);f.flush();os.fsync(f.fileno())
        atomic_json(folder/(dataset+'.json'),meta)
        run=self.run(id);run['datasets'].append(dataset);self.update(run)
        self.event(id,'dataset_saved',{'dataset':dataset,'samples':len(rows)})
        return dataset
    def load_dataset(self,id,dataset='telemetry'):
        folder=self.run_dir(id);run=self.run(id)
        if dataset=='telemetry':path=folder/'telemetry.csv';meta=read_json(folder/'metadata.json')
        else:
            if dataset not in run['datasets']:raise ValueError('Unknown dataset.')
            path=folder/'datasets'/(dataset+'.csv');meta=read_json(path.with_suffix('.json'))
        rows=[]
        with path.open(newline='',encoding='utf-8') as f:
            for row in csv.DictReader(f):
                item={}
                for key,value in row.items():
                    if value in ('',None):item[key]=None
                    elif value in ('True','False'):item[key]=value=='True'
                    else:
                        try:
                            number=float(value);item[key]=number if math.isfinite(number) else None
                        except ValueError:item[key]=value
                rows.append(item)
        return rows,meta
    def review(self,id,dataset,settings):
        run=self.run(id)
        if run['status']=='recording':raise ValueError('Finish recording before analysis.')
        rows,meta=self.load_dataset(id,dataset)
        result=analyze(rows,meta,settings)
        revision=uid('analysis');folder=self.run_dir(id)/'analysis'/revision;folder.mkdir(parents=True)
        derived=result.pop('derived')
        atomic_json(folder/'settings.json',dict(settings,dataset=dataset))
        atomic_json(folder/'summary.json',result)
        if derived:
            with (folder/'derived.csv').open('x',newline='',encoding='utf-8') as f:
                w=csv.writer(f);w.writerow(derived);w.writerows(zip(*derived.values()))
        with (folder/'metrics.csv').open('x',newline='',encoding='utf-8') as f:
            w=csv.writer(f);w.writerow(['metric','value_json'])
            for key,value in result['metrics'].items():w.writerow([key,json.dumps(value)])
        run['reviews'].append(revision);run['latest_review']={'id':revision,'dataset':dataset,'summary':result}
        self.update(run);self.event(id,'analysis_completed',{'revision':revision,'dataset':dataset,'settings':settings})
        return dict(result,derived=derived)
    def finish_review(self,id,outcome,reason):
        if outcome not in ('completed','unsuccessful','invalid-acquisition'):raise ValueError('Invalid review outcome.')
        run=self.run(id)
        if run['status']=='recording':raise ValueError('Recording is still active.')
        review=run.get('latest_review')
        if outcome=='completed' and (not review or not review['summary']['valid_acquisition'] or run['acquisition_status'] not in ('recorded','valid')):
            raise ValueError('Review a valid dataset first. Interrupted/failed acquisitions cannot be accepted.')
        if outcome=='completed' and review and any(f['code']=='control_fault' for f in review['summary']['quality_flags']):
            raise ValueError('The reviewed dataset includes a control fault; record an unsuccessful outcome.')
        if outcome!='completed' and not str(reason).strip():raise ValueError('A reason is required.')
        if outcome=='completed' and run['control_outcome']=='failure':raise ValueError('A recorded control failure cannot be accepted as successful.')
        run.update(status=outcome,reason=str(reason)[:4000])
        if outcome=='unsuccessful':run['control_outcome']='failure'
        elif outcome=='completed':run.update(control_outcome='success',acquisition_status='valid')
        else:run['acquisition_status']='invalid'
        self.update(run);self.event(id,'operator_review',{'outcome':outcome,'reason':reason})
        with self.lock:
            p=self.plan(run['plan_id']);p.update(status=outcome,reason=reason);self.save_plans()
        return run
    def bundle(self,id):
        run=self.run(id)
        if run['status']=='recording':raise ValueError('Finish recording before exporting.')
        folder=self.run_dir(id);out=io.BytesIO();hashes={}
        with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED) as z:
            for path in sorted(folder.rglob('*')):
                if path.is_file() and not path.name.endswith('.tmp'):
                    content=path.read_bytes();name=str(path.relative_to(folder)).replace('\\','/')
                    hashes[name]=hashlib.sha256(content).hexdigest();z.writestr(id+'/'+name,content)
            z.writestr(id+'/checksums.json',json.dumps(hashes,indent=2))
        return out.getvalue()
    def aggregate(self,metric='angle_error_deg.rms'):
        groups={}
        for r in reversed(self.list_runs()):
            if r['status']!='completed' or not r.get('latest_review'):continue
            review=r['latest_review'];value=review['summary']['metrics']
            try:
                for key in metric.split('.'):value=value[key]
                if not isinstance(value,(int,float)) or not math.isfinite(value):continue
                _,meta=self.load_dataset(r['id'],review['dataset'])
            except (KeyError,TypeError,OSError):continue
            p=r['plan'];cal=meta.get('calibration',{}).get('id')
            # Different bandwidth, calibration, windows and sources are never pooled.
            acq=meta.get('acquisition',{})
            comparable={k:acq.get(k) for k in ('source','requested_hz','bandwidth_hz','filtering','synchronization','ranges')}
            signature=json.dumps([comparable,review['summary']['settings'],cal],sort_keys=True)
            key=(review['summary'].get('source',r['source']),p['method'],p['rpm'],p['load_a'],p['test_type'],signature)
            # Retry attempts share a pair id; distinct matrices/manual repeats do not.
            independent_repeat=(p.get('pair_id',p['id']),p['repeat'])
            groups.setdefault(key,{})[independent_repeat]={'value':value,'run_id':r['id']}
        result=[]
        for key,repeats in groups.items():
            values=[v['value'] for v in repeats.values()]
            result.append(dict(source=key[0],method=key[1],rpm=key[2],load_a=key[3],test_type=key[4],
                               n=len(values),mean=statistics.mean(values),std=statistics.stdev(values) if len(values)>1 else None,
                               run_ids=[v['run_id'] for v in repeats.values()],metric=metric,group_signature=key[5]))
        return result
