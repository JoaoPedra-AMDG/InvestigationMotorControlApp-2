"""Real ODrive runtime adapter; discovery and motion require explicit requests.

API sources (reviewed 2026-09-17):
https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html
https://docs.odriverobotics.com/v/latest/guides/python-package.html
https://docs.odriverobotics.com/v/latest/manual/hardware-config.html#sensorless

Motion and polling use the owner thread. Finite capture downloads use the official
synchronous, thread-safe helper off that thread. No synthetic runtime fallback. Injected
connectors are reserved for unit tests and never exposed through the web API.
"""
import copy
import importlib.metadata
import importlib.util
import itertools
import math
import queue
import re
import threading
import time
import uuid
from settings_support import proposed_settings, same
from concurrent.futures import Future, TimeoutError as FutureTimeout
from capture_runtime import CaptureService,inspect_capture

DEFAULT_PROFILE = dict(test_serial='', load_serial='', roles_verified=False,
    max_speed_rpm=0., max_load_a=0., test_direction=1, load_direction=-1,
    stop_policy='coast', stop_policy_verified=False, polling_hz=20.,
    connection_timeout_s=3., sensorless_startup_verified=False, sensorless_min_rpm=0.,
    axis_units_verified=False, startup_timeout_s=15., encoder_reference_path='',
    encoder_reference_scale_rad=None, encoder_reference_verified=False,
    encoder_speed_path='', encoder_speed_scale_rpm=None,
    calibration=dict(verified=False,id='',pole_pairs=None,offset_rad=None,encoder_direction=1),
    capture_bandwidth_hz=None,capture_filtering='')
SIGNAL_PATHS = {
    'dc_voltage_v': ('vbus_voltage', 'V', 'measured'),
    'dc_current_a': ('ibus', 'A', 'estimated'),
    'speed_rpm': ('axis0.vel_estimate', 'rpm', 'selected feedback estimate'),
    'position_turns': ('axis0.pos_estimate', 'turn', 'selected feedback estimate'),
    'iq_a': ('axis0.motor.foc.Iq_measured', 'A', 'measured; firmware filtered'),
    'id_a': ('axis0.motor.foc.Id_measured', 'A', 'measured; firmware filtered'),
    'iq_command_a': ('axis0.motor.foc.Iq_setpoint', 'A', 'commanded'),
    'id_command_a': ('axis0.motor.foc.Id_setpoint', 'A', 'commanded'),
    'ia_a': ('axis0.motor.alpha_beta_controller.current_meas_phA', 'A', 'measured'),
    'ib_a': ('axis0.motor.alpha_beta_controller.current_meas_phB', 'A', 'measured'),
    'ic_a': ('axis0.motor.alpha_beta_controller.current_meas_phC', 'A', 'measured'),
    'estimated_electrical_rad': ('axis0.motor.sensorless_estimator.phase', 'rad electrical', 'estimated'),
    'motor_temp_c': ('axis0.motor.motor_thermistor.temperature', 'degC', 'measured'),
    'controller_temp_c': ('axis0.motor.fet_thermistor.temperature', 'degC', 'measured'),
    'speed_command_rpm': ('axis0.controller.input_vel', 'rpm', 'commanded'),
    'torque_command_nm': ('axis0.controller.input_torque', 'Nm', 'commanded using configured Kt'),
}
EXTRA_SIGNALS = ('torque_nm', 'encoder_mech_rad', 'encoder_speed_rpm', 'sensorless_speed_rpm')
CONFIG_PATHS = [
    'axis0.config.load_encoder', 'axis0.config.commutation_encoder',
    'axis0.config.motor.phase_resistance_valid', 'axis0.config.motor.phase_inductance_valid',
    'axis0.config.motor.phase_resistance', 'axis0.config.motor.phase_inductance',
    'axis0.config.motor.pole_pairs', 'axis0.config.motor.torque_constant',
    'axis0.config.motor.current_soft_max', 'axis0.config.motor.current_hard_max',
    'axis0.config.motor.current_control_bandwidth', 'axis0.config.motor.direction',
    'axis0.config.enable_watchdog', 'axis0.config.watchdog_timeout',
    'axis0.config.init_vel', 'axis0.config.init_torque',
    'axis0.controller.config.control_mode', 'axis0.controller.config.input_mode',
    'axis0.controller.config.vel_limit', 'axis0.controller.config.vel_ramp_rate',
    'axis0.controller.config.torque_ramp_rate', 'axis0.controller.config.vel_gain',
    'axis0.controller.config.vel_integrator_gain', 'axis0.controller.config.pos_gain',
    'axis0.controller.config.use_commutation_vel',
    'axis0.controller.config.use_load_encoder_for_commutation_vel',
    'axis0.commutation_mapper.config.offset_valid', 'axis0.commutation_mapper.config.offset',
    'axis0.commutation_mapper.config.scale', 'axis0.pos_vel_mapper.config.scale',
    'config.dc_bus_overvoltage_trip_level', 'config.dc_bus_undervoltage_trip_level',
    'config.dc_max_positive_current', 'config.dc_max_negative_current',
    'ibus_report_filter_k', 'axis0.motor.foc.I_measured_report_filter_k',
    'axis0.config.sensorless_ramp.vel', 'axis0.config.sensorless_ramp.accel',
    'axis0.config.sensorless_ramp.current', 'axis0.config.sensorless_ramp.ramp_time',
    'axis0.motor.motor_thermistor.config.enabled',
]
IDLE, CLOSED_LOOP, SENSORLESS = 1, 8, 4  # Documented AxisState and EncoderId.
SENSORED_ENCODERS = {1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14}
SENSORLESS_BLOCKER = ('Sensorless automatic startup is unavailable: this adapter has no verified '
    'API marker distinguishing the open-loop ramp from observer handover. '
    'CLOSED_LOOP_CONTROL and procedure_result alone do not prove handover. '
    'A separately started sensorless session can be connected for read-only monitoring and recording.')


def installed():
    try:
        available = importlib.util.find_spec('odrive') is not None
        version = importlib.metadata.version('odrive') if available else None
    except (ValueError, importlib.metadata.PackageNotFoundError):
        available, version = False, None
    return dict(package_installed=available, package_version=version)


def _read(obj, path):
    try:
        for part in path.split('.'):
            obj = getattr(obj, part)
        return None if isinstance(obj, (float, int)) and not math.isfinite(obj) else obj
    except AttributeError:
        return None


def _serial(value):
    if isinstance(value, int):
        return f'{value:012X}'
    text = str(value or '').strip().upper().removeprefix('0X')
    if not re.fullmatch(r'[0-9A-F]{1,12}', text):
        raise ValueError('ODrive serial numbers must be hexadecimal, up to 12 characters.')
    return text.zfill(12)


def _number(value, name, low, high):
    try:
        number = float(value)
    except (TypeError, ValueError):
        number = float('nan')
    if isinstance(value, bool) or not math.isfinite(number) or not low <= number <= high:
        raise ValueError(f'{name} must be between {low:g} and {high:g}.')
    return number


def _empty_board(serial=''):
    return dict(connected=False, serial=serial, firmware=None, state='DISCONNECTED',
        errors={}, signals={key: None for key in (*SIGNAL_PATHS, *EXTRA_SIGNALS)},
        provenance={}, configuration={}, sample_id=None, acquired_at_s=None,
        read_start_s=None, read_end_s=None, feedback_method='unavailable', runtime_api=False)


empty_board = _empty_board


class USBConnector:
    def __init__(self):
        import odrive
        self.odrive = odrive

    def connect(self, serial, timeout):
        if not hasattr(self.odrive, 'find_sync'):
            raise RuntimeError('Installed ODrive package lacks find_sync; install the current official package.')
        return self.odrive.find_sync(serial_number=serial, timeout=timeout, interfaces=['usb'])

    def release(self, device):
        release = getattr(self.odrive, 'release_connection', None)
        if release is None:
            raise RuntimeError('ODrive package cannot explicitly release USB; close this app before opening the GUI.')
        release(device)


class HardwareController:
    def __init__(self, profile=None, *, connector=None):
        self._profile = dict(DEFAULT_PROFILE)
        self._connector, self._injected = connector, connector is not None
        self._devices = {}
        self._boards = {r: _empty_board() for r in ('test', 'load')}
        self._state, self._error, self._stage = 'DISCONNECTED', '', 'disconnected'
        self._sample_id, self._acquired_at = 0, None
        self._package = installed()
        self._owned_motion = False
        self.capture_service=CaptureService()
        self._settings_preview=None;self._settings_result=None
        self._pending_load = None
        self._motion_epoch = 0
        self._queue = queue.PriorityQueue(maxsize=32)
        self._counter = itertools.count()
        self._lock = threading.Lock()
        self._shutdown, self._abort_start = threading.Event(), threading.Event()
        self._snapshot = {}
        self._publish()
        self._thread = threading.Thread(target=self._loop, name='odrive-owner', daemon=True)
        self._thread.start()
        if profile:
            self.configure(profile)

    def snapshot(self):
        with self._lock:
            result = copy.deepcopy(self._snapshot)
        age = time.perf_counter()-result['acquired_at_s'] if result['acquired_at_s'] else None
        result['sample_age_s'] = age
        if age is None or age > max(.5, 3/result['profile']['polling_hz']):
            result['control_ready'] = False
        return result

    def _call(self, action, *args, urgent=False):
        if self._shutdown.is_set():
            raise RuntimeError('Hardware controller is closed.')
        future = Future()
        try:
            self._queue.put_nowait((0 if urgent else 1, next(self._counter), action, args, future))
        except queue.Full:
            raise ValueError('Hardware command queue is full.') from None
        try:
            return future.result(timeout=15)
        except FutureTimeout:
            future.cancel()
            self._abort_start.set()
            raise RuntimeError('Hardware operation timed out; pending start cancelled. Check board states.') from None

    def configure(self, profile):
        return self._call('configure', dict(profile))

    def connect(self):
        return self._call('connect')

    def disconnect(self):
        self._motion_epoch += 1
        self._abort_start.set()
        return self._call('disconnect', urgent=True)

    def start(self, rpm, load_a, method='sensored'):
        return self._call('start', rpm, load_a, method, self._motion_epoch)

    def stop(self):
        self._motion_epoch += 1
        self._abort_start.set()
        return self._call('stop', urgent=True)

    def clear_errors(self):
        return self._call('clear_errors')

    def start_capture(self,plan):return self._call('capture',plan)
    def capture_result(self):return self.capture_service.take()
    def cancel_capture(self):self.capture_service.cancel()
    def _do_capture(self,plan):
        if self._state!='RUNNING' or any(b.get('state_code')!=CLOSED_LOOP or any(v!=0 for v in b['errors'].values()) for b in self._boards.values()):
            raise ValueError('High-rate capture requires both motors in fault-free closed-loop operation.')
        self.capture_service.start(self._devices,self._boards,self._profile,plan)

    def refresh_settings(self):return self._call('refresh_settings')
    def preview_settings(self,values):return self._call('preview_settings',dict(values))
    def apply_settings(self,token,persist=False):return self._call('apply_settings',token,persist)
    def update_limits(self,rpm,load):return self._call('update_limits',rpm,load)
    def set_tuning_gains(self,expected,proposed,baseline):
        return self._call('set_tuning_gains',dict(expected),dict(proposed),dict(baseline))
    def save_tuning_gains(self,expected):
        return self._call('save_tuning_gains',dict(expected))

    def _do_save_tuning_gains(self,expected):
        self._settings_idle()
        board=self._devices['test']
        for path in ('axis0.controller.config.vel_gain','axis0.controller.config.vel_integrator_gain'):
            if not same(_read(board,path),expected.get(path)):
                raise ValueError('Gain changed before save. Review tuning settings again.')
        if _read(board,'axis0.config.startup_closed_loop_control') is not False:
            raise ValueError('Disable automatic closed-loop startup before saving.')
        for flag in ('startup_motor_calibration','startup_encoder_index_search','startup_encoder_offset_calibration','startup_homing'):
            if _read(board,'axis0.config.'+flag) is True:
                raise ValueError('Disable automatic startup procedures before saving.')
        if not callable(_read(board,'save_configuration')):
            raise ValueError('Test board cannot save configuration through this API.')
        try:
            if board.save_configuration() is not True:
                raise ValueError('The test board did not confirm configuration save.')
            self._stage='tuning gains saved; reconnect to verify persistence'
        finally:
            self._release_all();self._state='DISCONNECTED'

    def _do_set_tuning_gains(self,expected,proposed,baseline):
        """Change only the test motor's velocity PI gains while both axes are stopped."""
        self._settings_idle()
        board=self._devices['test']
        if self._boards['test'].get('feedback_method')!='sensored':
            raise ValueError('Automated velocity tuning requires sensored test feedback.')
        paths=('axis0.controller.config.vel_gain','axis0.controller.config.vel_integrator_gain')
        for path in paths:
            original=_number(baseline.get(path),path,1e-9,1e6)
            value=_number(proposed.get(path),path,original*.5,original*1.5)
            current=_read(board,path)
            if not same(current,expected.get(path)):
                raise ValueError('A velocity gain changed outside this tuning session. Refresh the session.')
        changed=[]
        try:
            for path in paths:
                obj=board
                parts=path.split('.')
                for part in parts[:-1]:obj=getattr(obj,part)
                setattr(obj,parts[-1],proposed[path])
                if not same(_read(board,path),proposed[path]):
                    raise ValueError('Velocity gain read-back failed: '+path)
                changed.append(path)
        except Exception:
            for path in changed:
                try:
                    obj=board
                    parts=path.split('.')
                    for part in parts[:-1]:obj=getattr(obj,part)
                    setattr(obj,parts[-1],expected[path])
                except Exception:pass
            raise
        self._boards['test']['configuration']=self._configuration(board)
        self._do_refresh_settings()

    def _settings_idle(self):
        if self._owned_motion or len(self._devices)!=2:raise ValueError('Connect both boards and stop motion before changing board settings.')
        if self.capture_service.thread and self.capture_service.thread.is_alive():raise ValueError('Wait for the capture download to finish.')
        for role,device in self._devices.items():
            if _serial(_read(device,'serial_number'))!=self._profile[role+'_serial']:raise ValueError('Board identity changed; reconnect.')
            speed=_read(device,'axis0.vel_estimate')
            if _read(device,'axis0.current_state')!=IDLE or _read(device,'axis0.is_armed') is not False or not isinstance(speed,(int,float)) or abs(speed*60)>1:
                raise ValueError('Both boards must report IDLE, disarmed, and speed below 1 rpm before settings can be changed.')

    def _do_refresh_settings(self):
        for role,device in self._devices.items():self._boards[role]['configuration']=self._configuration(device)
        if self._devices:self._poll()

    def _do_update_limits(self,rpm,load):
        rpm=_number(rpm,'Validated maximum speed (rpm)',.001,100000)
        load=_number(load,'Validated maximum load (A)',.001,1000)
        if self._devices:
            self._settings_idle()
            vel=_read(self._devices['test'],'axis0.controller.config.vel_limit')
            current=_read(self._devices['load'],'axis0.config.motor.current_soft_max')
            if not isinstance(vel,(int,float)) or rpm>vel*60 or not isinstance(current,(int,float)) or load>current:
                raise ValueError('Local maxima must fit the existing test velocity limit and load current limit. This tool does not raise board current or voltage limits.')
        self._profile.update(max_speed_rpm=rpm,max_load_a=load)
        self._settings_preview=None

    def _do_preview_settings(self,values):
        self._settings_idle()
        if not self._profile['roles_verified'] or not self._profile['axis_units_verified']:
            raise ValueError('Verify the physical board roles and motor-shaft units in Connections first.')
        self._do_refresh_settings()
        firmware=next((c for c in self._readiness() if c['name']=='Firmware compatibility'),None)
        if not firmware or firmware['state']!='pass':raise ValueError('Matching supported released firmware is required before changing board settings.')
        proposed=proposed_settings(values,self._profile['polling_hz']);changes=[]
        for role,paths in proposed.items():
            device=self._devices[role]
            if not callable(_read(device,'axis0.watchdog_feed')):raise ValueError(role+': watchdog feed API unavailable.')
            for path,value in paths.items():
                old=_read(device,path)
                if not isinstance(old,(int,float,bool)):raise ValueError(role+': unsupported setting '+path)
                changes.append({'role':role,'serial':self._profile[role+'_serial'],'path':path,'before':old,'after':value})
        self._settings_preview={'token':uuid.uuid4().hex,'expires_at':time.monotonic()+120,'epoch':self._motion_epoch,'changes':changes}
        self._settings_result=None
        self._do_refresh_settings()

    def _do_apply_settings(self,token,persist):
        preview=self._settings_preview
        self._settings_preview=None
        if not preview or token!=preview['token'] or time.monotonic()>preview['expires_at']:
            raise ValueError('Preview the settings again; the previous preview has expired or was already used.')
        if not isinstance(persist,bool):raise ValueError('Save to board must be true or false.')
        self._settings_idle()
        if self._motion_epoch!=preview['epoch']:raise ValueError('Stop or disconnect cancelled the settings preview.')
        if persist:
            for role,device in self._devices.items():
                if _read(device,'axis0.config.startup_closed_loop_control') is not False:
                    raise ValueError(role+': saving requires startup_closed_loop_control=False to prevent automatic arming after reboot.')
                for flag in ('startup_motor_calibration','startup_encoder_index_search','startup_encoder_offset_calibration','startup_homing'):
                    if _read(device,'axis0.config.'+flag) is True:raise ValueError(role+': disable automatic startup procedures before saving.')
        for change in preview['changes']:
            if not same(_read(self._devices[change['role']],change['path']),change['before']):
                raise ValueError('A board setting changed after preview. Refresh and preview again.')
        if persist and not all(callable(_read(d,'save_configuration')) for d in self._devices.values()):
            raise ValueError('Save configuration is unavailable on one board. Apply temporary settings instead.')
        result={'state':'applying','changes':preview['changes'],'verified':[],'saved':[],'error':''};self._settings_result=result
        try:
            for change in preview['changes']:
                self._settings_idle()
                if self._motion_epoch!=preview['epoch']:raise ValueError('Stop cancelled the remaining settings updates.')
                device=self._devices[change['role']]
                if change['path']=='axis0.config.enable_watchdog':device.axis0.watchdog_feed()
                obj=device;parts=change['path'].split('.')
                for part in parts[:-1]:obj=getattr(obj,part)
                setattr(obj,parts[-1],change['after'])
                if not same(_read(device,change['path']),change['after']):raise ValueError('Read-back verification failed for '+change['path'])
                result['verified'].append(change)
            self._do_refresh_settings()
            if persist:
                # Saving may reboot/disconnect a board. Never infer successful
                # persistence from a lost USB connection or automatically retry.
                for role in ('test','load'):
                    if self._motion_epoch!=preview['epoch']:raise ValueError('Stop cancelled the remaining saves.')
                    device=self._devices[role]
                    if _read(device,'axis0.current_state')!=IDLE or _read(device,'axis0.is_armed') is not False:raise ValueError('Board left IDLE before saving.')
                    if device.save_configuration() is not True:raise ValueError(role+': board did not confirm configuration saved.')
                    result['saved'].append(role)
                result['state']='saved; reconnect to verify persistence'
            else:result['state']='applied and read back; not saved to nonvolatile memory'
        except Exception as exc:
            result['state']='partial or failed';result['error']=str(exc)
            self._error='Settings update incomplete: '+str(exc)
            try:self._do_refresh_settings()
            except Exception:pass
            raise ValueError(self._error+' Inspect the verified changes before retrying.') from exc
        finally:
            if persist:
                self._release_all();self._state='DISCONNECTED';self._stage='disconnected after configuration save'

    def close(self):
        if not self._shutdown.is_set():
            try:
                self.disconnect()
            finally:
                self._shutdown.set()
                self._thread.join(timeout=4)

    def _loop(self):
        deadline = time.perf_counter()
        while not self._shutdown.is_set():
            try:
                _, _, action, args, future = self._queue.get(timeout=max(0, min(.05, deadline-time.perf_counter())))
                if future.set_running_or_notify_cancel():
                    try:
                        getattr(self, '_do_'+action)(*args)
                        self._publish()
                        future.set_result(self.snapshot())
                    except Exception as exc:
                        self._publish()
                        future.set_exception(exc)
            except queue.Empty:
                pass
            if time.perf_counter() >= deadline:
                if self._devices:
                    try:
                        self._poll()
                        if self._owned_motion:
                            self._feed_watchdogs()
                    except Exception as exc:
                        self._fault('Communication or control fault: '+str(exc))
                self._publish()
                deadline = time.perf_counter()+1/self._profile['polling_hz']

    def _do_configure(self, values):
        if self._devices:
            raise ValueError('Disconnect both boards before changing the local rig profile.')
        if set(values)-set(DEFAULT_PROFILE):
            raise ValueError('Unknown profile fields: '+', '.join(sorted(set(values)-set(DEFAULT_PROFILE))))
        p = dict(self._profile, **values)
        for key in ('test_serial', 'load_serial'):
            p[key] = _serial(p[key]) if p[key] else ''
        if p['test_serial'] and p['test_serial'] == p['load_serial']:
            raise ValueError('Test and load serial numbers must be distinct.')
        for key in ('roles_verified', 'stop_policy_verified', 'sensorless_startup_verified','axis_units_verified','encoder_reference_verified'):
            if not isinstance(p[key], bool):
                raise ValueError(f'{key} must be true or false.')
        for key, upper in [('max_speed_rpm', 100000), ('max_load_a', 1000)]:
            p[key] = _number(p[key], key, 0, upper)
        for key in ('test_direction', 'load_direction'):
            if isinstance(p[key], bool) or p[key] not in (-1, 1):
                raise ValueError(f'{key} must be +1 or -1.')
        if p['stop_policy'] != 'coast':
            raise ValueError('Only a validated coast stop (request IDLE on both boards) is implemented.')
        p['polling_hz'] = _number(p['polling_hz'], 'polling_hz', 1, 100)
        p['connection_timeout_s'] = _number(p['connection_timeout_s'], 'connection_timeout_s', .1, 5)
        if p['sensorless_min_rpm'] is not None:
            p['sensorless_min_rpm'] = _number(p['sensorless_min_rpm'], 'sensorless_min_rpm', 0, 100000)
        p['startup_timeout_s'] = _number(p['startup_timeout_s'],'startup_timeout_s',1,30)
        if p['capture_bandwidth_hz'] is not None:p['capture_bandwidth_hz']=_number(p['capture_bandwidth_hz'],'capture_bandwidth_hz',.001,1e7)
        p['capture_filtering']=str(p['capture_filtering'])[:1000]
        for path_key,scale_key in [('encoder_reference_path','encoder_reference_scale_rad'),('encoder_speed_path','encoder_speed_scale_rpm')]:
            path=p[path_key]
            if not isinstance(path,str) or (path and any(not part.isidentifier() or part.startswith('_') for part in path.split('.'))):
                raise ValueError('Encoder paths must be public firmware property names.')
            if path in ('axis0.pos_estimate','axis0.vel_estimate') or 'sensorless_estimator' in path:
                raise ValueError('Selected-control and sensorless estimates cannot be independent encoder references.')
            if p[scale_key] is not None:p[scale_key]=_number(p[scale_key],scale_key,-1e12,1e12)
            if p['encoder_reference_verified'] and path and (p[scale_key] is None or p[scale_key]==0):
                raise ValueError('A verified encoder path requires a finite, nonzero scale.')
        calibration=p['calibration']
        if not isinstance(calibration,dict):raise ValueError('Calibration must be an object.')
        if calibration.get('verified'):
            pole=_number(calibration.get('pole_pairs'),'pole_pairs',1,1000)
            if pole!=int(pole) or not calibration.get('id') or calibration.get('encoder_direction') not in (-1,1):raise ValueError('Verified calibration needs integer pole pairs, ID and direction.')
            _number(calibration.get('offset_rad'),'offset_rad',-1000,1000)
        self._profile = p
        self._boards = {r: _empty_board(p[r+'_serial']) for r in ('test', 'load')}

    def _configuration(self, device):
        return {path: _read(device, path) for path in CONFIG_PATHS}

    def _do_connect(self):
        if self._devices:
            raise ValueError('Already connected. Disconnect before reconnecting.')
        if not all(self._profile[r+'_serial'] for r in ('test', 'load')):
            raise ValueError('Enter distinct test and load board serial numbers first.')
        self._package = installed()
        if not self._injected and not self._package['package_installed']:
            self._error = 'ODrive Python package is not installed in this application environment.'
            raise ValueError(self._error)
        if self._connector is None:
            self._connector = USBConnector()
        try:
            for role in ('test', 'load'):
                serial = self._profile[role+'_serial']
                device = self._connector.connect(serial, self._profile['connection_timeout_s'])
                self._devices[role] = device
                if _serial(_read(device, 'serial_number')) != serial:
                    raise ValueError(f'{role.title()} board identity does not match requested serial.')
                self._boards[role] = _empty_board(serial)
                self._boards[role]['configuration'] = self._configuration(device)
                self._boards[role]['runtime_api'] = self._runtime_api(device)
                cfg=self._boards[role]['configuration']
                enc=cfg.get('axis0.config.load_encoder'),cfg.get('axis0.config.commutation_encoder')
                self._boards[role]['feedback_method']='sensorless' if enc==(SENSORLESS,SENSORLESS) else 'sensored'
                self._boards[role]['capture']=inspect_capture(device,role,self._boards[role],self._profile,_read)
            self._state, self._error, self._stage = 'CONNECTED', '', 'monitoring'
            self._poll()
        except Exception as exc:
            self._release_all()  # Connection and its cleanup are strictly read-only.
            self._state, self._error = 'DISCONNECTED', str(exc)
            raise

    def _runtime_api(self, device):
        return callable(_read(device, 'axis0.watchdog_feed')) and all(_read(device, p) is not None for p in
            ('axis0.requested_state', 'axis0.controller.input_vel', 'axis0.controller.input_torque'))

    def _poll(self):
        next_boards, next_id = {}, self._sample_id+1
        try:
            for role, device in self._devices.items():
                old = self._boards[role]
                board = _empty_board(old['serial'])
                board['configuration'], board['runtime_api'] = old['configuration'], old['runtime_api']
                board['capture']=old.get('capture',{'available':False,'detail':'Not inspected'})
                board['read_start_s'] = time.perf_counter()
                state = _read(device, 'axis0.current_state')
                if state is None:
                    raise RuntimeError(f'{role} board has no readable axis0.current_state.')
                board['connected'], board['state_code'] = True, int(state)
                board['state'] = {IDLE:'IDLE', CLOSED_LOOP:'CLOSED_LOOP_CONTROL'}.get(state, f'AXIS_STATE_{state}')
                version = [_read(device, 'fw_version_'+p) for p in ('major', 'minor', 'revision')]
                board['firmware'] = '.'.join(map(str, version)) if all(v is not None for v in version) else None
                board['firmware_unreleased'] = _read(device, 'fw_version_unreleased')
                board['errors'] = {k: _read(device, 'axis0.'+k) for k in ('active_errors', 'disarm_reason')}
                board['procedure_result'], board['is_armed'] = _read(device, 'axis0.procedure_result'), _read(device, 'axis0.is_armed')
                c = board['configuration']
                enc = c.get('axis0.config.load_encoder'), c.get('axis0.config.commutation_encoder')
                board['feedback_method'] = 'sensorless' if enc == (SENSORLESS, SENSORLESS) else ('sensored' if all(e in SENSORED_ENCODERS for e in enc) else 'mixed / unverified')
                for name, (path, unit, kind) in SIGNAL_PATHS.items():
                    value = _read(device, path)
                    if value is not None and name in ('speed_rpm', 'speed_command_rpm'):
                        value *= 60
                    board['signals'][name] = value
                    board['provenance'][name] = dict(path=path, unit=unit, kind=kind, available=value is not None,
                        timing='Sequential host read; not synchronized with other channels or drives')
                if c.get('axis0.motor.motor_thermistor.config.enabled') is not True:
                    board['signals']['motor_temp_c']=None
                    board['provenance']['motor_temp_c']['available']=False
                iq, kt = board['signals']['iq_a'], c.get('axis0.config.motor.torque_constant')
                if iq is not None and kt is not None and kt > 0:
                    board['signals']['torque_nm'] = iq*kt
                board['provenance']['torque_nm'] = dict(path='configured torque_constant * Iq_measured', unit='Nm',
                    kind='estimate from configured Kt; not independently measured shaft torque', available=board['signals']['torque_nm'] is not None)
                pp, phase_vel = c.get('axis0.config.motor.pole_pairs'), _read(device, 'axis0.motor.sensorless_estimator.phase_vel')
                if phase_vel is not None and pp is not None and pp > 0:
                    board['signals']['sensorless_speed_rpm'] = phase_vel*60/(2*math.pi*pp)
                board['provenance']['sensorless_speed_rpm'] = dict(path='axis0.motor.sensorless_estimator.phase_vel * 60 / (2*pi*pole_pairs)', unit='rpm', kind='estimated using configured pole pairs', available=board['signals']['sensorless_speed_rpm'] is not None)
                for name in ('encoder_mech_rad', 'encoder_speed_rpm'):
                    board['provenance'][name] = dict(available=False, kind='unavailable', reason='Independent encoder mapping/scaling unverified; axis estimates are not relabelled as independent reference.')
                if role=='test' and self._profile['encoder_reference_verified']:
                    for name,path_key,scale_key in [('encoder_mech_rad','encoder_reference_path','encoder_reference_scale_rad'),('encoder_speed_rpm','encoder_speed_path','encoder_speed_scale_rpm')]:
                        path,scale=self._profile[path_key],self._profile[scale_key]
                        if path and scale is not None:
                            value=_read(device,path)
                            if isinstance(value,(int,float)) and math.isfinite(value*scale):
                                board['signals'][name]=value*scale
                                board['provenance'][name]=dict(path=path,scale=scale,available=True,kind='operator-verified independent encoder reference; sequential host read')
                board['read_end_s'] = time.perf_counter()
                board['acquired_at_s'], board['sample_id'] = board['read_end_s'], next_id
                next_boards[role] = board
        except Exception:
            # Never publish partial paired samples or previous values as fresh.
            for role in self._devices:
                old = self._boards[role]
                self._boards[role] = _empty_board(self._profile[role+'_serial'])
                self._boards[role]['configuration'] = old['configuration']
                self._boards[role]['runtime_api'] = old['runtime_api']
            raise
        self._boards.update(next_boards)
        self._sample_id, self._acquired_at = next_id, time.perf_counter()
        if self._owned_motion:
            for role, board in next_boards.items():
                if board['state_code'] != CLOSED_LOOP or any(v != 0 for v in board['errors'].values()):
                    raise RuntimeError(f"{role} left closed-loop control or reported errors: {board['errors']}")
            if self._pending_load:
                pending=self._pending_load
                speed=next_boards['test']['signals']['speed_rpm']
                if self._abort_start.is_set():
                    self._pending_load=None
                elif speed is not None and speed*self._profile['test_direction']>=min(50.,pending['rpm']*.9):
                    self._devices['load'].axis0.controller.input_torque=pending['torque']
                    self._pending_load=None;self._state,self._stage='RUNNING','closed_loop'
                elif time.perf_counter()>pending['deadline']:
                    raise RuntimeError('Test motor did not reach the load-application speed before timeout.')
        if self._state == 'STOPPING' and all(b.get('state_code') == IDLE for b in next_boards.values()):
            self._state = 'CONNECTED'

    def _readiness(self):
        p, checks = self._profile, []
        def check(name, okay, detail):
            checks.append(dict(name=name, state='pass' if okay else 'blocked', detail=detail))
        check('Board connections', len(self._devices)==2 and all(b['connected'] for b in self._boards.values()), 'Both configured serial numbers must be connected.')
        check('Physical roles and direction', p['roles_verified'] and p['axis_units_verified'], 'Verify board labels, axis turns equal rotor turns, and opposing-load sign for this coupling.')
        check('Local rig limits', p['max_speed_rpm']>0 and p['max_load_a']>0, 'Set validated maximum speed and load current; zero disables control.')
        check('Stop procedure', p['stop_policy']=='coast' and p['stop_policy_verified'], 'Validate IDLE/coast on both drives; removing torque does not prove rotors have stopped.')
        if len(self._devices)==2:
            firmware = [b['firmware'] for b in self._boards.values()]
            compatible = firmware[0]==firmware[1] and all(f and re.fullmatch(r'0\.6\.(1[0-9]|[2-9][0-9])', f) for f in firmware)
            check('Firmware compatibility', compatible and all(b.get('firmware_unreleased')==0 for b in self._boards.values()), 'Both boards require the same released 0.6.x firmware, 0.6.10 or newer, and inspected APIs.')
            for role, board in self._boards.items():
                c, prefix = board['configuration'], role.title()
                check(prefix+' axis state', board.get('state_code')==(CLOSED_LOOP if self._owned_motion else IDLE), 'First Start requires IDLE; externally started sessions are monitored read-only.')
                check(prefix+' fault status', all(board['errors'].get(k)==0 for k in ('active_errors','disarm_reason')), 'Active errors and disarm reason must be readable and zero.')
                check(prefix+' motor calibration', c.get('axis0.config.motor.phase_resistance_valid') is True and c.get('axis0.config.motor.phase_inductance_valid') is True, 'Firmware resistance and inductance calibration flags must be valid.')
                check(prefix+' encoder calibration', c.get('axis0.commutation_mapper.config.offset_valid') is True, 'Commutation-mapper offset must be valid; commission in the ODrive GUI.')
                check(prefix+' feedback', board['feedback_method']=='sensored', 'Both feedback paths must use physical encoders. '+(SENSORLESS_BLOCKER if board['feedback_method']=='sensorless' else ''))
                wd = c.get('axis0.config.watchdog_timeout')
                check(prefix+' watchdog', c.get('axis0.config.enable_watchdog') is True and wd is not None and max(.5,6/p['polling_hz'])<=wd<=2, 'Preconfigure watchdog enabled, timeout >= max(0.5 s, six polling periods) and <= 2 s.')
                control, input_mode, ramp = (2,2,'vel_ramp_rate') if role=='test' else (1,6,'torque_ramp_rate')
                rate = c.get('axis0.controller.config.'+ramp)
                check(prefix+' controller mode', c.get('axis0.controller.config.control_mode')==control and c.get('axis0.controller.config.input_mode')==input_mode and rate is not None and rate>0, 'Preconfigure test velocity control + VEL_RAMP; load torque control + TORQUE_RAMP; positive validated ramp rates.')
                check(prefix+' initial setpoints', c.get('axis0.config.init_vel')==0 and c.get('axis0.config.init_torque')==0, 'Preconfigure init_vel=0 and init_torque=0; firmware overwrites inputs with these values when arming.')
                soft, hard, kt = (c.get('axis0.config.motor.'+k) for k in ('current_soft_max','current_hard_max','torque_constant'))
                vel = c.get('axis0.controller.config.vel_limit')
                okay = all(v is not None and v>0 for v in (soft,hard,kt,vel)) and hard>=soft
                okay = okay and (soft>=p['max_load_a'] if role=='load' else vel*60>=p['max_speed_rpm'])
                check(prefix+' configured limits', okay, 'Valid Kt, velocity and current limits required; local maxima must fit drive limits.')
                check(prefix+' runtime APIs', board['runtime_api'], 'Control setpoints, requested_state and watchdog_feed must be exposed.')
                check(prefix+' required telemetry', all(board['signals'].get(k) is not None for k in ('speed_rpm','iq_a','dc_voltage_v')), 'Speed, q-axis current and bus voltage must be readable for control and settling checks.')
        return checks

    def _publish(self):
        readiness = self._readiness()
        result = dict(self._package, state=self._state, error=self._error, profile=copy.deepcopy(self._profile),
            boards=copy.deepcopy(self._boards), readiness=readiness,
            control_ready=all(c['state']=='pass' for c in readiness) and self._state in ('CONNECTED','RUNNING'),
            sample_id=self._sample_id, acquired_at_s=self._acquired_at, control_stage=self._stage,
            settings_preview=copy.deepcopy(self._settings_preview),settings_result=copy.deepcopy(self._settings_result),
            source='HARDWARE', sensorless_start_available=False, sensorless_start_blocker=SENSORLESS_BLOCKER,
            acquisition=dict(kind='sequential host USB polling', requested_hz=self._profile['polling_hz'],
                synchronized=False, timestamp='Host perf_counter seconds at end of each board read',
                bandwidth_hz=None, onboard_capture='Finite buffer when compatible API and channels are present'),capture=self.capture_service.status())
        with self._lock:
            self._snapshot = result

    def _do_start(self, rpm, load_a, method, epoch):
        if self.capture_service.thread and self.capture_service.thread.is_alive():raise ValueError('Wait for the previous capture download to finish.')
        if epoch!=self._motion_epoch:raise ValueError('Start cancelled by a newer Stop or Disconnect request.')
        if method not in ('sensored','sensorless'):
            raise ValueError('Feedback method must be sensored or sensorless.')
        if method=='sensorless':
            raise ValueError(SENSORLESS_BLOCKER)
        if self._state=='FAULT':
            raise ValueError('Resolve and explicitly clear the fault before starting again.')
        rpm = _number(rpm, 'Target rpm', .001, self._profile['max_speed_rpm'])
        load_a = _number(load_a, 'Load current', 0, self._profile['max_load_a'])
        for role, device in self._devices.items():
            self._boards[role]['configuration'] = self._configuration(device)
            self._boards[role]['runtime_api'] = self._runtime_api(device)
        if self._devices:
            self._poll()
        blockers = [r['name']+': '+r['detail'] for r in self._readiness() if r['state']!='pass']
        if blockers:
            raise ValueError('Start blocked. '+' '.join(blockers))
        if epoch!=self._motion_epoch:raise ValueError('Start cancelled during readiness checks.')
        self._abort_start.clear()
        try:
            test, load = self._devices['test'].axis0, self._devices['load'].axis0
            if epoch!=self._motion_epoch or self._abort_start.is_set():raise RuntimeError('Start cancelled before arming.')
            if not self._owned_motion:
                self._stage = 'arming'
                for axis in (test,load):
                    if epoch!=self._motion_epoch or self._abort_start.is_set():raise RuntimeError('Start cancelled before arming.')
                    axis.controller.input_vel = 0.
                    axis.controller.input_torque = 0.
                self._feed_watchdogs()
                if epoch!=self._motion_epoch or self._abort_start.is_set():raise RuntimeError('Start cancelled before arming.')
                test.requested_state = CLOSED_LOOP
                if epoch!=self._motion_epoch or self._abort_start.is_set():raise RuntimeError('Start cancelled during arming.')
                load.requested_state = CLOSED_LOOP
                deadline = time.perf_counter()+2
                while True:
                    if self._abort_start.is_set():
                        raise RuntimeError('Start cancelled by Stop or Disconnect.')
                    states = [_read(d,'axis0.current_state') for d in self._devices.values()]
                    errors = [_read(d,'axis0.active_errors') for d in self._devices.values()]
                    if any(error!=0 for error in errors):
                        raise RuntimeError('An axis reported an error while arming.')
                    if states==[CLOSED_LOOP,CLOSED_LOOP]:
                        break
                    if time.perf_counter()>=deadline:
                        raise RuntimeError('Both axes did not enter CLOSED_LOOP_CONTROL within 2 seconds.')
                    self._feed_watchdogs()
                    time.sleep(.02)
            if self._abort_start.is_set():
                raise RuntimeError('Start cancelled.')
            starting=not self._owned_motion
            test.controller.input_vel = rpm*self._profile['test_direction']/60
            kt = self._boards['load']['configuration']['axis0.config.motor.torque_constant']
            torque=load_a*kt*self._profile['load_direction']
            if starting or self._pending_load:
                load.controller.input_torque=0.
                self._pending_load=dict(rpm=rpm,torque=torque,deadline=time.perf_counter()+self._profile['startup_timeout_s'])
            else:load.controller.input_torque=torque
            self._owned_motion = True
            self._state, self._stage, self._error = ('STARTING','accelerating; load held at zero','') if self._pending_load else ('RUNNING','closed_loop','')
            self._poll()
            self._feed_watchdogs()
        except Exception as exc:
            self._fault('Start or command failed: '+str(exc))
            raise

    def _feed_watchdogs(self):
        for device in self._devices.values():
            device.axis0.watchdog_feed()

    def _request_idle(self):
        failures = []
        for role in ('load','test'):
            if role in self._devices:
                try:
                    self._devices[role].axis0.requested_state = IDLE
                except Exception as exc:
                    failures.append(f'{role}: {exc}')
        self._owned_motion, self._stage = False, 'coasting / torque disabled requested'
        self._pending_load=None
        return failures

    def _do_stop(self):
        failures = self._request_idle()
        if failures:
            self._state, self._error = 'FAULT','Could not confirm both stop requests: '+'; '.join(failures)
        elif self._devices:
            try:
                self._poll()
                self._state = 'CONNECTED' if all(b.get('state_code')==IDLE for b in self._boards.values()) else 'STOPPING'
            except Exception as exc:
                self._state, self._error = 'FAULT',str(exc)
        else:
            self._state = 'DISCONNECTED'

    def _fault(self, reason):
        if self._owned_motion or self._stage=='arming':
            failures = self._request_idle()
            if failures:
                reason += '; Stop request failures: '+'; '.join(failures)
        self._state, self._error = 'FAULT',reason

    def _do_clear_errors(self):
        if not self._devices:
            self._error, self._state = '', 'DISCONNECTED'
            return
        if self._owned_motion or any(_read(d,'axis0.current_state')!=IDLE for d in self._devices.values()):
            raise ValueError('Both boards must be IDLE before clearing errors.')
        if not all(callable(_read(d,'clear_errors')) for d in self._devices.values()):
            raise ValueError('Firmware does not expose clear_errors on both boards.')
        for device in self._devices.values():
            device.clear_errors()
        self._poll()
        self._state, self._error, self._stage = 'CONNECTED','','monitoring'

    def _release_all(self):
        failures = []
        for role, device in self._devices.items():
            try:
                self._connector.release(device)
            except Exception as exc:
                failures.append(f'{role}: {exc}')
        self._devices = {}
        self._boards = {r:_empty_board(self._profile[r+'_serial']) for r in ('test','load')}
        self._acquired_at = None
        return failures

    def _do_disconnect(self):
        self.capture_service.cancel()
        if self._owned_motion:
            self._do_stop()
        failures = self._release_all()
        self._state, self._stage = 'DISCONNECTED','disconnected'
        if failures:
            self._error = 'USB release incomplete: '+'; '.join(failures)
