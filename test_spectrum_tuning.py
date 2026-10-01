"""Offline signal-analysis and gain-write checks; never connects to USB."""
import math
import tempfile
import unittest

from spectrum import analyze_spectrum
from tuning import trial_metrics, GAIN_PATHS
from hardware import HardwareController
from test_hardware import FixtureConnector, valid_profile
from app import Rig


class SpectrumTests(unittest.TestCase):
    def test_desired_band_fraction_has_current_variance_units(self):
        fs=8000;n=4000
        rows=[]
        for i in range(n):
            t=i/fs
            rows.append(dict(time_s=t,ia_a=math.sin(2*math.pi*30*t)+.5*math.sin(2*math.pi*180*t),
                ib_a=math.sin(2*math.pi*30*t-2*math.pi/3)+.5*math.sin(2*math.pi*180*t),
                ic_a=math.sin(2*math.pi*30*t+2*math.pi/3)+.5*math.sin(2*math.pi*180*t)))
        meta=dict(role='test',pole_pairs=3,plan={'rpm':600},acquisition={'source':'ODRIVE_ONBOARD','requested_hz':fs,'partial':False})
        report=analyze_spectrum(rows,meta)
        self.assertEqual(report['desired_electrical_hz'],30)
        self.assertAlmostEqual(report['target_fraction'],.8,delta=.02)
        self.assertAlmostEqual(report['resolution_hz'],2,places=5)
        self.assertIn('not electrical power',report['meaning'])

    def test_rejects_incomplete_capture(self):
        rows=[dict(time_s=i/8000,ia_a=0.,ib_a=0.,ic_a=0.) for i in range(800)]
        meta=dict(role='test',pole_pairs=3,plan={'rpm':600},acquisition={'source':'ODRIVE_ONBOARD','requested_hz':8000,'partial':True})
        with self.assertRaisesRegex(ValueError,'partial'):analyze_spectrum(rows,meta)


class TuningTests(unittest.TestCase):
    def test_trial_metrics_distinguish_steady_error_and_overshoot(self):
        samples=[dict(drive_rpm=100 if i<10 else 98,load_iq_a=1) for i in range(20)]
        report=trial_metrics(samples,100,1)
        self.assertAlmostEqual(report['speed_rms_error_rpm'],2)
        self.assertEqual(report['overshoot_rpm'],0)
        self.assertEqual(report['load_iq_rms_error_a'],0)

    def test_gain_write_checks_stopped_state_expected_values_and_bound(self):
        fixture=FixtureConnector();controller=HardwareController(valid_profile(),connector=fixture)
        self.addCleanup(controller.close)
        controller.connect()
        baseline=dict(zip(GAIN_PATHS,(.1,.2)))
        proposed=dict(zip(GAIN_PATHS,(.11,.22)))
        controller.set_tuning_gains(baseline,proposed,baseline)
        self.assertAlmostEqual(fixture.test.axis0.controller.config.vel_gain,.11)
        with self.assertRaisesRegex(ValueError,'changed outside'):
            controller.set_tuning_gains(baseline,baseline,baseline)
        with self.assertRaisesRegex(ValueError,'between'):
            controller.set_tuning_gains(proposed,dict(zip(GAIN_PATHS,(.2,.22))),baseline)
        self.assertAlmostEqual(fixture.test.axis0.controller.config.vel_gain,.11)
        controller.set_tuning_gains(proposed,baseline,baseline)
        self.assertAlmostEqual(fixture.test.axis0.controller.config.vel_gain,.1)

    def test_tuning_requires_explicit_physical_task_before_motion(self):
        fixture=FixtureConnector();controller=HardwareController(valid_profile(),connector=fixture)
        with tempfile.TemporaryDirectory() as directory:
            rig=Rig(directory,rate=20,controller=controller)
            try:
                controller.connect()
                state=rig.tuning.prepare('test',300,0)
                self.assertEqual(state['state'],'awaiting_operator')
                with self.assertRaisesRegex(ValueError,'physical rig task'):
                    rig.tuning.start(False)
                self.assertFalse(any(row[0]=='write' and row[1].endswith('.requested_state') and row[2]==8
                    for row in fixture.log))
            finally:rig.close()


if __name__=='__main__':unittest.main()
