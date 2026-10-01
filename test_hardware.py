"""Offline adapter contract tests. No ODrive package import/discovery or USB use.

Fixtures represent explicit register values; they are never a runtime backend.
"""
import copy
import math
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from hardware import HardwareController, DEFAULT_PROFILE, IDLE, CLOSED_LOOP


class RegisterNode:
    def __init__(self, log, path, **values):
        object.__setattr__(self, '_log', log)
        object.__setattr__(self, '_path', path)
        object.__setattr__(self, '_fail_reads', set())
        object.__setattr__(self, '_fail_writes', set())
        object.__setattr__(self, '_read_hooks', {})
        for key, value in values.items():
            object.__setattr__(self, key, value)

    def __getattribute__(self, name):
        if not name.startswith('_'):
            if name in object.__getattribute__(self, '_fail_reads'):
                raise ConnectionError('Fixture link lost while reading '+name)
            hook=object.__getattribute__(self,'_read_hooks').get(name)
            if hook:hook()
        return object.__getattribute__(self, name)

    def __setattr__(self, name, value):
        if name.startswith('_'):
            object.__setattr__(self, name, value)
            return
        self._log.append(('write', self._path+'.'+name, value, threading.get_ident()))
        if name in self._fail_writes:
            raise ConnectionError('Fixture write failed: '+name)
        object.__setattr__(self, name, value)


class Axis(RegisterNode):
    def __setattr__(self, name, value):
        super().__setattr__(name, value)
        if name == 'requested_state':
            if value == IDLE or self._accept_arm:
                object.__setattr__(self, 'current_state', value)
                object.__setattr__(self, 'is_armed', value == CLOSED_LOOP)
            if value == CLOSED_LOOP:
                # Real >=0.6.10 firmware initializes inputs at state entry.
                object.__setattr__(self.controller, 'input_vel', self.config.init_vel)
                object.__setattr__(self.controller, 'input_torque', self.config.init_torque)

    def watchdog_feed(self):
        self._log.append(('feed', self._path, None, threading.get_ident()))


class Board(RegisterNode):
    def clear_errors(self):
        self._log.append(('clear', self._path, None, threading.get_ident()))
        object.__setattr__(self.axis0, 'active_errors', 0)
        object.__setattr__(self.axis0, 'disarm_reason', 0)


class FixtureConnector:
    def __init__(self):
        self.log = []
        self.devices = {}
        for role, serial in [('test', 0xABC), ('load', 0xDEF)]:
            node = lambda path, **kw: RegisterNode(self.log, role+'.'+path, **kw)
            cfg_motor = node('axis0.config.motor', phase_resistance_valid=True,
                phase_inductance_valid=True, phase_resistance=.1, phase_inductance=.0001,
                pole_pairs=7, torque_constant=.1, current_soft_max=10., current_hard_max=15.,
                current_control_bandwidth=1000., direction=1)
            cfg = node('axis0.config', motor=cfg_motor, load_encoder=1, commutation_encoder=1,
                enable_watchdog=True, watchdog_timeout=1., init_vel=0., init_torque=0.,
                sensorless_ramp=node('sensorless_ramp', vel=50., accel=10., current=1., ramp_time=.5))
            cfg_controller = node('axis0.controller.config', control_mode=2 if role=='test' else 1,
                input_mode=2 if role=='test' else 6, vel_limit=100., vel_ramp_rate=10.,
                torque_ramp_rate=.2, vel_gain=.1, vel_integrator_gain=.2, pos_gain=10.,
                use_commutation_vel=False, use_load_encoder_for_commutation_vel=False)
            controller = node('axis0.controller', config=cfg_controller, input_vel=0., input_torque=0.)
            motor = node('axis0.motor',
                foc=node('axis0.motor.foc', Iq_measured=2., Id_measured=.1, Iq_setpoint=2.1,
                    Id_setpoint=0., I_measured_report_filter_k=.1),
                alpha_beta_controller=node('alpha_beta_controller', current_meas_phA=1.,
                    current_meas_phB=-.3, current_meas_phC=-.7),
                sensorless_estimator=node('sensorless_estimator', phase=.4, phase_vel=14*math.pi),
                motor_thermistor=node('motor_thermistor', temperature=28., config=node('thermistor.config', enabled=True)),
                fet_thermistor=node('fet_thermistor', temperature=30.))
            axis = Axis(self.log, role+'.axis0', config=cfg, controller=controller, motor=motor,
                current_state=IDLE, requested_state=IDLE, active_errors=0, disarm_reason=0,
                procedure_result=0, is_armed=False, vel_estimate=0., pos_estimate=1.25,
                commutation_mapper=node('commutation_mapper', config=node('commutation_mapper.config', offset_valid=True, offset=.2, scale=7.)),
                pos_vel_mapper=node('pos_vel_mapper', config=node('pos_vel_mapper.config', scale=1.)))
            object.__setattr__(axis, '_accept_arm', True)
            board = Board(self.log, role, axis0=axis, serial_number=serial,
                fw_version_major=0, fw_version_minor=6, fw_version_revision=12,
                fw_version_unreleased=0, vbus_voltage=24., ibus=1.2, ibus_report_filter_k=.2,
                config=node('config', dc_bus_overvoltage_trip_level=30., dc_bus_undervoltage_trip_level=20.,
                    dc_max_positive_current=10., dc_max_negative_current=-10.),
                inc_encoder0=node('inc_encoder0', raw=1234))
            self.devices[f'{serial:012X}'] = board
        self.test = self.devices['000000000ABC']
        self.load = self.devices['000000000DEF']

    def connect(self, serial, timeout):
        self.log.append(('connect', serial, timeout, threading.get_ident()))
        return self.devices[serial]

    def release(self, device):
        self.log.append(('release', device._path, None, threading.get_ident()))


def valid_profile(**overrides):
    return dict(copy.deepcopy(DEFAULT_PROFILE), test_serial='ABC', load_serial='DEF',
        **dict(dict(roles_verified=True, axis_units_verified=True, max_speed_rpm=1200.,
            max_load_a=5., stop_policy_verified=True, polling_hz=50., startup_timeout_s=1.), **overrides))


class HardwareContractTests(unittest.TestCase):
    def setUp(self):
        self.fixture = FixtureConnector()
        self.controller = HardwareController(valid_profile(), connector=self.fixture)
        self.addCleanup(self.controller.close)

    def wait_status(self, predicate, timeout=2):
        deadline = time.perf_counter()+timeout
        while time.perf_counter()<deadline:
            result=self.controller.snapshot()
            if predicate(result):return result
            time.sleep(.01)
        self.fail('Adapter did not reach expected state: '+str(self.controller.snapshot()))

    def mutations(self):
        return [row for row in self.fixture.log if row[0] in ('write','feed','clear')]

    def connect(self):
        result = self.controller.connect()
        self.assertTrue(result['control_ready'], result['readiness'])
        return result

    def test_construction_and_profile_do_not_discover_boards(self):
        self.assertEqual(self.fixture.log, [])
        result=self.controller.snapshot()
        self.assertEqual(result['state'],'DISCONNECTED')
        self.assertTrue(all(v is None for v in result['boards']['test']['signals'].values()))
        self.assertFalse(result['control_ready'])

    def test_connect_and_disconnect_read_only_and_owner_thread(self):
        result=self.connect()
        self.assertEqual(result['boards']['test']['serial'],'000000000ABC')
        self.assertEqual(result['boards']['load']['firmware'],'0.6.12')
        self.assertEqual(result['boards']['test']['signals']['dc_voltage_v'],24.)
        self.assertEqual(result['boards']['test']['signals']['torque_nm'],.2)
        self.assertIn('estimate', result['boards']['test']['provenance']['torque_nm']['kind'])
        self.assertIsNone(result['boards']['test']['signals']['encoder_mech_rad'])
        self.controller.disconnect()
        self.assertEqual(self.mutations(),[])
        self.assertEqual(len({row[3] for row in self.fixture.log}),1)
        self.assertNotEqual(self.fixture.log[0][3],threading.get_ident())

    def test_wrong_serial_releases_connection_without_writes(self):
        object.__setattr__(self.fixture.test,'serial_number',0x123)
        with self.assertRaisesRegex(ValueError,'identity'):
            self.controller.connect()
        self.assertEqual(self.controller.snapshot()['state'],'DISCONNECTED')
        self.assertEqual(self.mutations(),[])
        self.assertEqual(len([e for e in self.fixture.log if e[0]=='release']),1)

    def test_local_limits_and_invalid_input_rejected_without_writes(self):
        self.connect()
        for rpm,load in [(1201,1),(600,6),(-1,0),(float('nan'),0),(600,float('inf'))]:
            with self.subTest(rpm=rpm,load=load):
                with self.assertRaises(ValueError):self.controller.start(rpm,load,'sensored')
        self.assertEqual(self.mutations(),[])

    def test_missing_commissioning_prerequisites_fail_before_first_write(self):
        paths=[(self.fixture.test.axis0.config,'init_vel',2.),
            (self.fixture.test.axis0.config,'enable_watchdog',False),
            (self.fixture.load.axis0.config.motor,'current_soft_max',2.),
            (self.fixture.test.axis0.config.motor,'phase_resistance_valid',False),
            (self.fixture.test.axis0.commutation_mapper.config,'offset_valid',False),
            (self.fixture.load.axis0.controller.config,'input_mode',1),
            (self.fixture.test,'fw_version_unreleased',1)]
        self.connect()
        for obj,key,bad in paths:
            with self.subTest(key=key):
                previous=getattr(obj,key)
                object.__setattr__(obj,key,bad)
                try:
                    with self.assertRaisesRegex(ValueError,'Start blocked'):
                        self.controller.start(600,2,'sensored')
                finally:object.__setattr__(obj,key,previous)
        self.assertEqual(self.mutations(),[])

    def test_start_holds_load_until_speed_then_stop_requests_both_idle(self):
        self.connect()
        result=self.controller.start(600,2,'sensored')
        self.assertEqual(result['state'],'STARTING')
        self.assertEqual(self.fixture.test.axis0.controller.input_vel,10.)
        self.assertEqual(self.fixture.load.axis0.controller.input_torque,0.)
        object.__setattr__(self.fixture.test.axis0,'vel_estimate',1.)
        self.wait_status(lambda s:s['state']=='RUNNING')
        self.assertAlmostEqual(self.fixture.load.axis0.controller.input_torque,-.2)
        self.assertTrue(any(e[0]=='feed' for e in self.fixture.log))
        result=self.controller.stop()
        self.assertEqual(result['state'],'CONNECTED')
        self.assertEqual(self.fixture.test.axis0.current_state,IDLE)
        self.assertEqual(self.fixture.load.axis0.current_state,IDLE)
        self.assertIn('coasting',result['control_stage'])
        # Runtime writes never reconfigure firmware, gains or feedback routing.
        self.assertTrue(all('.config.' not in e[1] for e in self.fixture.log if e[0]=='write'))

    def test_pending_load_timeout_faults_and_stops_both(self):
        self.connect()
        self.controller.start(600,2,'sensored')
        result=self.wait_status(lambda s:s['state']=='FAULT',timeout=2)
        self.assertIn('load-application speed',result['error'])
        self.assertEqual(self.fixture.test.axis0.current_state,IDLE)
        self.assertEqual(self.fixture.load.axis0.current_state,IDLE)
        self.assertEqual(self.fixture.load.axis0.controller.input_torque,0.)

    def test_control_fault_preserves_readings_and_stops_peer(self):
        object.__setattr__(self.fixture.test.axis0,'vel_estimate',2.)
        self.connect()
        self.controller.start(600,1,'sensored')
        object.__setattr__(self.fixture.load.axis0,'active_errors',256)
        result=self.wait_status(lambda s:s['state']=='FAULT')
        self.assertTrue(result['boards']['load']['connected'])
        self.assertEqual(result['boards']['load']['signals']['dc_voltage_v'],24.)
        self.assertEqual(self.fixture.test.axis0.current_state,IDLE)
        self.assertEqual(self.fixture.load.axis0.current_state,IDLE)
        with self.assertRaisesRegex(ValueError,'clear the fault'):
            self.controller.start(600,1,'sensored')
        cleared=self.controller.clear_errors()
        self.assertEqual(cleared['state'],'CONNECTED')
        self.assertEqual(self.fixture.test.axis0.current_state,IDLE)

    def test_link_loss_invalidates_paired_sample_and_stops_peer(self):
        object.__setattr__(self.fixture.test.axis0,'vel_estimate',2.)
        self.connect()
        self.controller.start(600,1,'sensored')
        self.fixture.load._fail_reads.add('vbus_voltage')
        result=self.wait_status(lambda s:s['state']=='FAULT')
        self.assertFalse(result['boards']['test']['connected'])
        self.assertTrue(all(v is None for v in result['boards']['test']['signals'].values()))
        self.assertEqual(self.fixture.test.axis0.current_state,IDLE)
        self.assertEqual(self.fixture.load.axis0.current_state,IDLE)

    def test_stop_attempts_test_board_even_when_load_stop_write_fails(self):
        object.__setattr__(self.fixture.test.axis0,'vel_estimate',2.)
        self.connect()
        self.controller.start(600,1,'sensored')
        self.fixture.load.axis0._fail_writes.add('requested_state')
        result=self.controller.stop()
        self.assertEqual(result['state'],'FAULT')
        self.assertIn('load',result['error'])
        self.assertEqual(self.fixture.test.axis0.current_state,IDLE)

    def test_sensorless_start_rejected_without_mutation(self):
        self.connect()
        with self.assertRaisesRegex(ValueError,'observer handover'):
            self.controller.start(600,1,'sensorless')
        self.assertEqual(self.mutations(),[])

    def test_sensorless_externally_running_session_is_read_only(self):
        object.__setattr__(self.fixture.test.axis0.config,'load_encoder',4)
        object.__setattr__(self.fixture.test.axis0.config,'commutation_encoder',4)
        object.__setattr__(self.fixture.test.axis0,'current_state',CLOSED_LOOP)
        result=self.controller.connect()
        self.assertFalse(result['control_ready'])
        self.assertEqual(result['boards']['test']['feedback_method'],'sensorless')
        self.assertEqual(result['boards']['test']['signals']['estimated_electrical_rad'],.4)
        self.controller.disconnect()
        self.assertEqual(self.mutations(),[])

    def test_unavailable_signals_and_disabled_thermistor_remain_null(self):
        object.__delattr__(self.fixture.test,'ibus')
        object.__setattr__(self.fixture.test.axis0.motor.motor_thermistor.config,'enabled',False)
        result=self.connect()
        signals=result['boards']['test']['signals']
        self.assertIsNone(signals['dc_current_a'])
        self.assertIsNone(signals['motor_temp_c'])
        self.assertEqual(signals['dc_voltage_v'],24.)

    def test_verified_physical_encoder_scale_is_recorded(self):
        self.controller.configure(dict(encoder_reference_path='inc_encoder0.raw',
            encoder_reference_scale_rad=2*math.pi/4096,encoder_reference_verified=True))
        result=self.connect()
        signal=result['boards']['test']['signals']['encoder_mech_rad']
        self.assertAlmostEqual(signal,1234*2*math.pi/4096)
        self.assertIsNone(result['boards']['load']['signals']['encoder_mech_rad'])
        self.assertEqual(result['boards']['test']['provenance']['encoder_mech_rad']['path'],'inc_encoder0.raw')

    def test_selected_control_estimates_cannot_be_independent_references(self):
        for path in ('axis0.pos_estimate','axis0.vel_estimate',
                     'axis0.motor.sensorless_estimator.phase',
                     'axis0.motor.sensorless_estimator.phase_vel'):
            for key,scale in [('encoder_reference_path','encoder_reference_scale_rad'),
                              ('encoder_speed_path','encoder_speed_scale_rpm')]:
                with self.subTest(path=path,field=key):
                    with self.assertRaisesRegex(ValueError,'cannot be independent'):
                        self.controller.configure({key:path,scale:1.,'encoder_reference_verified':True})
        result=self.controller.snapshot()
        self.assertEqual(result['profile']['encoder_reference_path'],'')
        self.assertEqual(result['profile']['encoder_speed_path'],'')
        self.assertEqual(self.fixture.log,[])

    def test_stop_arriving_during_readiness_cancels_start_before_arming(self):
        self.connect()
        entered,release,stop_requested=threading.Event(),threading.Event(),threading.Event()
        outcomes={}
        original_config,original_call=self.controller._configuration,self.controller._call
        def blocked_config(device):
            if device is self.fixture.test:
                entered.set()
                if not release.wait(2):raise TimeoutError('Fixture validation gate timed out')
            return original_config(device)
        def observed_call(action,*args,**kwargs):
            # stop() publishes its cancellation before submitting the owner operation.
            if action=='stop':stop_requested.set()
            return original_call(action,*args,**kwargs)
        def run(name,action):
            try:outcomes[name]=action()
            except Exception as exc:outcomes[name]=exc
        with patch.object(self.controller,'_configuration',side_effect=blocked_config), patch.object(self.controller,'_call',side_effect=observed_call):
            starter=threading.Thread(target=run,args=('start',lambda:self.controller.start(600,2,'sensored')))
            stopper=threading.Thread(target=run,args=('stop',self.controller.stop))
            starter.start()
            try:
                self.assertTrue(entered.wait(1),'Start did not reach fixture readiness gate')
                stopper.start()
                self.assertTrue(stop_requested.wait(1),'Stop was not submitted')
            finally:
                release.set()
                starter.join(3)
                if stopper.ident is not None:stopper.join(3)
        self.assertIsInstance(outcomes.get('start'),Exception)
        self.assertIn('cancelled',str(outcomes['start']).lower())
        self.assertIsInstance(outcomes.get('stop'),dict)
        self.assertFalse(any(e[0]=='write' and e[1].endswith('requested_state') and e[2]==CLOSED_LOOP for e in self.fixture.log))
        self.assertFalse(any(e[0]=='write' and e[1].endswith(('input_vel','input_torque')) and e[2]!=0 for e in self.fixture.log))
        self.assertEqual(self.controller.snapshot()['state'],'CONNECTED')

    def test_stop_during_pending_load_read_never_applies_nonzero_load(self):
        self.connect()
        self.controller.start(600,2,'sensored')
        entered,release,stop_requested=threading.Event(),threading.Event(),threading.Event()
        outcome={}
        def gate():
            entered.set()
            if not release.wait(2):raise TimeoutError('Fixture sample gate timed out')
        # Hold a real owner-thread telemetry read immediately before the decision
        # to apply load. The speed threshold is already satisfied in this sample.
        self.fixture.load._read_hooks['vbus_voltage']=gate
        object.__setattr__(self.fixture.test.axis0,'vel_estimate',2.)
        original_call=self.controller._call
        def observed_call(action,*args,**kwargs):
            if action=='stop':stop_requested.set()
            return original_call(action,*args,**kwargs)
        def stop():
            try:outcome['stop']=self.controller.stop()
            except Exception as exc:outcome['stop']=exc
        stopper=threading.Thread(target=stop)
        with patch.object(self.controller,'_call',side_effect=observed_call):
            try:
                self.assertTrue(entered.wait(1),'Polling did not reach fixture sample gate')
                stopper.start()
                self.assertTrue(stop_requested.wait(1),'Stop was not submitted')
            finally:
                release.set()
                if stopper.ident is not None:stopper.join(3)
                self.fixture.load._read_hooks.clear()
        self.assertIsInstance(outcome.get('stop'),dict)
        self.assertFalse(any(e[0]=='write' and e[1]=='load.axis0.controller.input_torque' and e[2]!=0 for e in self.fixture.log))
        self.assertEqual(self.fixture.test.axis0.current_state,IDLE)
        self.assertEqual(self.fixture.load.axis0.current_state,IDLE)

    def test_missing_package_never_creates_runtime_connector(self):
        self.controller.close()
        with patch('hardware.installed',return_value={'package_installed':False,'package_version':None}), patch('hardware.USBConnector',side_effect=AssertionError('Unexpected USB initialization')):
            ctrl=HardwareController(valid_profile())
            try:
                with self.assertRaisesRegex(ValueError,'not installed'):ctrl.connect()
                self.assertEqual(ctrl.snapshot()['state'],'DISCONNECTED')
                self.assertFalse(ctrl.snapshot()['package_installed'])
            finally:ctrl.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
