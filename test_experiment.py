import io
import json
import math
import tempfile
import unittest
import zipfile
from unittest.mock import patch
import numpy as np
from analysis import analyze,wrap_degrees,timing_report
from experiment import Store,Recorder,DEFAULT_PLAN
from import_data import parse_csv,import_dataset


def metadata(plan):
    """Analytic fixtures are explicitly tagged; never experimental data."""
    return {'source':'SIMULATION','plan':dict(plan),
            'calibration':{'id':'TEST-FIXTURE-ONLY','verified':True,'pole_pairs':7,
                           'offset_rad':.2,'encoder_direction':1},
            'acquisition':{'source':'TEST_FIXTURE','requested_hz':8000,'bandwidth_hz':2000,
                           'filtering':'Analytic numerical fixture, no physical readings',
                           'synchronization':{'simultaneous':True,'method':'Analytic fixture clock','uncertainty_s':0}}}


def fixture(n=8000):
    """Known balanced phase currents and a fixed four-degree observer error."""
    t=np.arange(n)/8000
    mechanical=t*1000*2*np.pi/60
    theta=7*mechanical+.2
    iq=3+.09*np.sin(6*theta)
    id_current=.02*np.sin(2*theta)
    alpha=id_current*np.cos(theta)-iq*np.sin(theta)
    beta=id_current*np.sin(theta)+iq*np.cos(theta)
    estimate=theta+np.deg2rad(4+.7*np.sin(3*theta))
    rows=[{'time_s':float(t[i]),'sample':i,'encoder_mech_rad':float(mechanical[i]%(2*np.pi)),
           'estimated_electrical_rad':float(estimate[i]%(2*np.pi)),
           'ia_a':float(alpha[i]),'ib_a':float(-.5*alpha[i]+np.sqrt(3)/2*beta[i]),
           'ic_a':float(-.5*alpha[i]-np.sqrt(3)/2*beta[i]),
           'drive_iq_a':float(iq[i]),'drive_id_a':float(id_current[i]),
           'iq_command_a':3.,'id_command_a':0.,'encoder_rpm':1000.,'command_rpm':1000.,
           'load_iq_a':-2.,'load_command_a':2.,'control_stage':'closed_loop',
           'source':'SIMULATION'} for i in range(n)]
    meta=metadata(dict(DEFAULT_PLAN))
    return rows,meta


class AnalysisTests(unittest.TestCase):
    def test_corrupt_time_sequence_and_clipping_withhold_sensitive_metrics(self):
        for mode in ('invalid_time','duplicate_sample','clipped'):
            rows,meta=fixture(1000)
            if mode=='invalid_time':rows[10]['time_s']=None
            elif mode=='duplicate_sample':rows[10]['sample']=rows[9]['sample']
            else:rows[10]['clipped']=True
            result=analyze(rows,meta,{'start_s':0,'end_s':.12})
            self.assertFalse(result['valid_acquisition'])
            self.assertNotIn('ia_residual_a',result['metrics'])
            self.assertNotIn('angle_error_deg',result['metrics'])
            json.dumps(result,allow_nan=False)

    def test_invalid_calibration_and_timing_declarations_do_not_create_angle_metrics(self):
        for key,value in [('offset_rad',float('nan')),('uncertainty_s',-1)]:
            rows,meta=fixture(1000)
            target=meta['calibration'] if key=='offset_rad' else meta['acquisition']['synchronization']
            target[key]=value
            result=analyze(rows,meta)
            self.assertNotIn('angle_error_deg',result['metrics'])
            json.dumps(result,allow_nan=False)

    def test_wrapping_bias_and_common_reference(self):
        self.assertEqual(wrap_degrees([181,-181,359,180]).tolist(),[-179,179,-1,-180])
        rows,meta=fixture()
        result=analyze(rows,meta)
        self.assertTrue(result['valid_acquisition'])
        self.assertAlmostEqual(result['metrics']['angle_error_deg']['bias'],4,places=2)
        self.assertAlmostEqual(result['metrics']['angle_error_deg']['rms'],math.sqrt(16+.7**2/2),places=2)
        self.assertGreater(result['metrics']['ia_residual_a']['rms'],.03)
        self.assertLess(result['metrics']['ia_residual_a']['rms'],.2)
        actual=np.array([r['drive_iq_a'] for r in rows])
        np.testing.assert_allclose(result['derived']['iq_reference_a'],actual,atol=1e-10)
        self.assertFalse(result['pwm_ripple_supported'])

    def test_missing_and_unsynchronized_channels(self):
        rows,meta=fixture(1000);meta['calibration']['verified']=False
        for r in rows:r['ia_a']=None
        result=analyze(rows,meta)
        self.assertNotIn('angle_error_deg',result['metrics'])
        self.assertNotIn('ia_residual_a',result['metrics'])
        self.assertNotIn('iq_reference_a',result['metrics'])
        rows,meta=fixture(1000);meta['acquisition']['synchronization']['uncertainty_s']=.01
        result=analyze(rows,meta)
        self.assertNotIn('angle_error_deg',result['metrics'])

    def test_gaps_duplicates_and_invalid_window(self):
        report=timing_report([0,.001,.002,.005,.005,.004],1000)
        self.assertEqual(report['gap_count'],1);self.assertEqual(report['duplicate_timestamps'],1)
        self.assertEqual(report['estimated_missing_samples'],2);self.assertEqual(report['out_of_order'],1)
        rows,meta=fixture(1000);del rows[300:305]
        result=analyze(rows,meta);self.assertFalse(result['valid_acquisition'])
        self.assertNotIn('ia_residual_a',result['metrics'])
        with self.assertRaises(ValueError):analyze(rows,meta,{'start_s':2,'end_s':1})

    def test_fundamental_removed_not_full_sine_ripple(self):
        t=np.arange(8000)/8000
        rows=[{'time_s':float(x),'ia_a':5*math.sin(2*math.pi*100*x)+.2*math.sin(2*math.pi*700*x),
               'ib_a':3*math.sin(2*math.pi*100*x),'ic_a':2*math.sin(2*math.pi*100*x)} for x in t]
        _,meta=fixture(1)
        result=analyze(rows,meta,{'fundamental_hz':100,'window_type':'steady state'})
        self.assertAlmostEqual(result['metrics']['ia_residual_a']['rms'],.2/math.sqrt(2),places=8)
        self.assertAlmostEqual(result['metrics']['ib_residual_a']['rms'],0,places=8)

    def test_clipping_and_transient_tracking(self):
        rows,meta=fixture(1000);rows[0]['clipped']=True
        result=analyze(rows,meta,{'window_type':'transient'})
        self.assertFalse(result['valid_acquisition']);self.assertIn('iq_tracking_error_a',result['metrics'])
        self.assertNotIn('ia_residual_a',result['metrics'])


class StorageTests(unittest.TestCase):
    def test_matrix_order_retry_and_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            store=Store(directory);plans=store.generate({})
            self.assertEqual(len(plans),120)
            for i in range(0,120,2):
                self.assertEqual(plans[i]['pair_id'],plans[i+1]['pair_id'])
                self.assertNotEqual(plans[i]['method'],plans[i+1]['method'])
            self.assertNotEqual(plans[0]['method'],plans[2]['method'])
            with self.assertRaises(ValueError):store.skip([plans[0]['id']],'')
            run=store.create_run(plans[0],metadata(plans[0]))
            recovered=Store(directory).run(run['id'])
            self.assertEqual(recovered['status'],'invalid-acquisition')
            self.assertEqual(recovered['acquisition_status'],'interrupted')

    def test_raw_and_versioned_review_export_and_repeats(self):
        with tempfile.TemporaryDirectory() as directory:
            store=Store(directory);p=store.edit_plan({});rows,meta=fixture(3000)
            independent=store.edit_plan(dict(p,id=None,repeat=2))
            for plan in [p,p,independent]:
                run=store.create_run(plan,meta);id=run['id']
                run.update(status='awaiting review',acquisition_status='recorded');store.update(run)
                dataset=store.add_dataset(id,'capture',rows,meta)
                first=store.review(id,dataset,{'window_type':'steady state'})
                store.review(id,dataset,{'window_type':'steady state'})
                self.assertEqual(len(store.run(id)['reviews']),2)
                store.finish_review(id,'completed','')
            aggregate=store.aggregate();self.assertEqual(aggregate[0]['n'],2)
            self.assertEqual(aggregate[0]['std'],0)
            bundle=store.bundle(id)
            with zipfile.ZipFile(io.BytesIO(bundle)) as z:
                self.assertTrue(any(x.endswith('checksums.json') for x in z.namelist()))
                self.assertTrue(any(x.endswith('derived.csv') for x in z.namelist()))
                self.assertTrue(any('/datasets/' in x and x.endswith('.csv') for x in z.namelist()))

    def test_disk_failure_and_queue_overflow(self):
        with tempfile.TemporaryDirectory() as directory:
            class BrokenWriter:
                def __init__(self,*a,**k):pass
                def writeheader(self):raise OSError('disk full fixture')
            with patch('experiment.csv.DictWriter',BrokenWriter):
                recorder=Recorder(directory,['time_s']);recorder.done.wait(2)
                result=recorder.close();self.assertIn('disk full',result['error'])
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(Recorder,'_write',lambda self:self.finish.wait(1)):
                recorder=Recorder(directory,['time_s'],capacity=1)
                self.assertTrue(recorder.push({'time_s':0}))
                self.assertFalse(recorder.push({'time_s':1}))
                self.assertEqual(recorder.dropped,1)
                recorder.thread.join();recorder.file.close()

    def test_import_preserves_gaps_missing_and_raw(self):
        mapping={'time_s':{'column':'Time ms','unit':'s','scale':.001,'kind':'measured'},
                 'ia_a':{'column':'Ia','unit':'A','kind':'measured'}}
        text='Time ms,Ia\n0,1\n1,\n4,2\n'
        rows=parse_csv(text,mapping);self.assertIsNone(rows[1]['ia_a']);self.assertEqual(rows[2]['time_s'],.004)
        with tempfile.TemporaryDirectory() as directory:
            store=Store(directory);p=store.edit_plan({});run=store.create_run(p,metadata(p))
            run.update(status='awaiting review',acquisition_status='recorded');store.update(run)
            result=import_dataset(store,run['id'],{'csv':text,'mapping':mapping,'metadata':{}})
            original=store.run_dir(run['id'])/'datasets'/(result['dataset']+'.original.csv')
            self.assertEqual(original.read_bytes(),text.encode())


if __name__=='__main__':unittest.main()
