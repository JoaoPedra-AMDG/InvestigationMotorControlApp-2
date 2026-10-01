"""Explicit CSV column/unit mappings; retains original bytes and missing values."""
import csv
import hashlib
import io
import json
import math
from signals import UNITS
from experiment import finite, atomic_json


def parse_csv(text, mapping):
    if not isinstance(text,str) or len(text.encode('utf-8'))>8_000_000:raise ValueError('CSV import limit is 8 MB.')
    reader=csv.DictReader(io.StringIO(text.lstrip('\ufeff')))
    if not reader.fieldnames or len(set(reader.fieldnames))!=len(reader.fieldnames):raise ValueError('CSV needs unique column headers.')
    if 'time_s' not in mapping:raise ValueError('Map a timestamp column to time_s.')
    for target,spec in mapping.items():
        if target not in UNITS and target not in ('time_s','sample'):raise ValueError('Unknown canonical channel: '+target)
        if spec['column'] not in reader.fieldnames:raise ValueError('Missing CSV column: '+spec['column'])
        required_unit='s' if target=='time_s' else 'count' if target=='sample' else UNITS[target]
        if spec.get('unit')!=required_unit:raise ValueError(f'{target}: normalized unit must be {required_unit}.')
        finite(spec.get('scale',1),-1e12,1e12,'Scale');finite(spec.get('offset',0),-1e12,1e12,'Offset')
        if spec.get('kind') not in ('measured','estimated','commanded','simulated'):raise ValueError('Declare each channel kind.')
    rows=[]
    for row in reader:
        if len(rows)>=200000:raise ValueError('Import limit is 200,000 rows per dataset.')
        result={}
        for target,spec in mapping.items():
            value=row.get(spec['column'])
            if value in ('',None):result[target]=None;continue
            try:value=float(value)*float(spec.get('scale',1))+float(spec.get('offset',0))
            except ValueError:raise ValueError(f'Non-numeric value in {spec["column"]}, row {len(rows)+2}.')
            result[target]=value if math.isfinite(value) else None
        if result['time_s'] is None:raise ValueError('Every imported row requires a finite timestamp.')
        rows.append(result)
    if len(rows)<3:raise ValueError('Import at least three samples.')
    return rows


def import_dataset(store,run_id,data):
    if store.run(run_id)['status']=='recording':raise ValueError('Finish recording before importing.')
    mapping=data['mapping'];rows=parse_csv(data['csv'],mapping)
    declaration=data.get('metadata',{})
    acquisition=dict(declaration.get('acquisition',{}));sync=dict(acquisition.get('synchronization',{}))
    for key in ('requested_hz','bandwidth_hz','switching_hz'):
        if acquisition.get(key) is not None:acquisition[key]=finite(acquisition[key],1e-9,1e12,key)
    if sync.get('uncertainty_s') is not None:sync['uncertainty_s']=finite(sync['uncertainty_s'],0,100,'Timing uncertainty')
    sync['simultaneous']=sync.get('simultaneous') is True
    acquisition['synchronization']=sync
    acquisition['source']=data.get('source','EXTERNAL_DAQ')
    if acquisition['source'] not in ('EXTERNAL_DAQ','ODRIVE_ONBOARD'):raise ValueError('Select external DAQ or ODrive onboard import.')
    calibration=dict(declaration.get('calibration',{}))
    if calibration.get('verified'):
        pole=finite(calibration.get('pole_pairs',0),1,1000,'Pole pairs')
        if pole!=int(pole):raise ValueError('Pole pairs must be integer.')
        calibration['pole_pairs']=int(pole)
        calibration['offset_rad']=finite(calibration.get('offset_rad'),-1000,1000,'Fixed electrical offset')
        if not calibration.get('id') or calibration.get('encoder_direction') not in (-1,1):raise ValueError('Verified calibration needs an ID and encoder direction.')
        if not declaration.get('independent_encoder_provenance'):raise ValueError('Document the physical encoder source and exclusion from control.')
    # A provenance assertion is explicit operator input, never inferred from a column name.
    synthetic=declaration.get('source')=='SIMULATION' or any(spec.get('kind')=='simulated' for spec in mapping.values())
    meta={'source':'SIMULATION' if synthetic else 'IMPORTED','plan':store.run(run_id)['plan'],'acquisition':acquisition,'calibration':calibration,
          'signals':mapping,'operator_declaration':declaration,'original_sha256':hashlib.sha256(data['csv'].encode('utf-8')).hexdigest(),
          'clock_alignment_to_run':'unverified; plotted on this instrument clock only'}
    dataset=store.add_dataset(run_id,'import',rows,meta)
    folder=store.run_dir(run_id)/'datasets'
    (folder/(dataset+'.original.csv')).write_text(data['csv'],encoding='utf-8',newline='')
    atomic_json(folder/(dataset+'.mapping.json'),mapping)
    return {'dataset':dataset,'samples':len(rows)}
