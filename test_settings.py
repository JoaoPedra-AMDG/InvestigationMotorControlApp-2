"""Board setup tests using only explicit offline register fixtures."""
import unittest
from hardware import HardwareController,CLOSED_LOOP
from test_hardware import FixtureConnector,valid_profile

VALUES={'test_watchdog_s':1.,'load_watchdog_s':1.,'test_ramp_rpm_s':600.,'load_ramp_nm_s':.2}

class SettingsTests(unittest.TestCase):
    def setUp(self):
        self.fixture=FixtureConnector()
        for board in (self.fixture.test,self.fixture.load):
            object.__setattr__(board.axis0.config,'enable_watchdog',False)
            object.__setattr__(board.axis0.controller.config,'control_mode',3)
            object.__setattr__(board.axis0.controller.config,'input_mode',1)
            object.__setattr__(board.axis0.config,'startup_closed_loop_control',False)
        self.controller=HardwareController(valid_profile(max_speed_rpm=0,max_load_a=0),connector=self.fixture)
        self.addCleanup(self.controller.close);self.controller.connect();self.fixture.log.clear()
    def mutations(self):return [x for x in self.fixture.log if x[0] in ('write','feed','clear','save')]
    def preview(self):return self.controller.preview_settings(VALUES)['settings_preview']['token']

    def test_preview_is_read_only_and_apply_resolves_named_checks_without_arming(self):
        token=self.preview();self.assertEqual(self.mutations(),[])
        result=self.controller.apply_settings(token)
        self.assertEqual(len(result['settings_result']['verified']),18)
        result=self.controller.update_limits(1200,5)
        for check in result['readiness']:
            if check['name'] in ('Local rig limits','Test watchdog','Load watchdog','Test controller mode','Load controller mode'):self.assertEqual(check['state'],'pass',check)
        self.assertAlmostEqual(self.fixture.test.axis0.controller.config.vel_ramp_rate,10)
        self.assertFalse(any(x[0]=='write' and 'requested_state' in x[1] for x in self.mutations()))
        self.assertEqual(self.fixture.test.axis0.current_state,1)
        with self.assertRaises(ValueError):self.controller.apply_settings(token)

    def test_invalid_rates_and_timeouts_rejected_before_writes(self):
        for field,value in [('test_ramp_rpm_s',0),('load_watchdog_s',10),('test_watchdog_s',float('nan'))]:
            with self.assertRaises(ValueError):self.controller.preview_settings(dict(VALUES,**{field:value}))
        self.assertEqual(self.mutations(),[])

    def test_motion_after_preview_blocks_apply(self):
        token=self.preview();object.__setattr__(self.fixture.load.axis0,'current_state',CLOSED_LOOP)
        with self.assertRaisesRegex(ValueError,'IDLE'):self.controller.apply_settings(token)
        self.assertEqual(self.mutations(),[])

    def test_stale_preview_is_rejected(self):
        token=self.preview();object.__setattr__(self.fixture.test.axis0.config,'watchdog_timeout',.7)
        with self.assertRaisesRegex(ValueError,'changed after preview'):self.controller.apply_settings(token)
        self.assertEqual(self.mutations(),[])

    def test_partial_write_reports_verified_subset(self):
        token=self.preview();self.fixture.load.axis0.config._fail_writes.add('watchdog_timeout')
        with self.assertRaisesRegex(ValueError,'incomplete'):self.controller.apply_settings(token)
        result=self.controller.snapshot()['settings_result']
        self.assertEqual(result['state'],'partial or failed');self.assertGreater(len(result['verified']),0)
        self.assertLess(len(result['verified']),18)
        self.assertFalse(any(x[0]=='save' for x in self.fixture.log))

    def test_limits_cannot_raise_board_limits(self):
        with self.assertRaises(ValueError):self.controller.update_limits(7000,5)
        with self.assertRaises(ValueError):self.controller.update_limits(1000,11)
        self.assertEqual(self.controller.snapshot()['profile']['max_speed_rpm'],0)
        self.assertEqual(self.mutations(),[])

    def test_persistent_save_is_explicit_and_releases_connections(self):
        for role in ('test','load'):
            board=getattr(self.fixture,role)
            def save(role=role):self.fixture.log.append(('save',role,None,None));return True
            object.__setattr__(board,'save_configuration',save)
        result=self.controller.apply_settings(self.preview(),True)
        self.assertEqual(result['settings_result']['saved'],['test','load'])
        self.assertEqual(result['state'],'DISCONNECTED')
        self.assertFalse(result['boards']['test']['connected'])

    def test_save_blocks_automatic_startup_before_any_write(self):
        object.__setattr__(self.fixture.test.axis0.config,'startup_closed_loop_control',True)
        with self.assertRaisesRegex(ValueError,'automatic arming'):self.controller.apply_settings(self.preview(),True)
        self.assertEqual(self.mutations(),[])

if __name__=='__main__':unittest.main()
