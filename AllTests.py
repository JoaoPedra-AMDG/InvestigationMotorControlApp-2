#!/usr/bin/env python3
"""Standalone ODrive pair test sequence. Run: python AllTests.py --run.

Only this file and installed Python/ODrive dependencies are needed. The
editable sweep and protection values are in SETTINGS below. Each high-rate
capture writes one CSV per motor; live readings go to monitor.jsonl.
"""
import argparse
import asyncio
import builtins
import csv
import hashlib
import importlib.metadata
import inspect
import itertools
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, asdict
from pathlib import Path
from types import SimpleNamespace
import numpy as np

# Keep status output usable in Windows terminals and redirected logs whose
# default encoding cannot represent symbols used in the test plan.
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(errors='backslashreplace')

ROOT = Path(__file__).resolve().parent
TOTAL_PHASES = 6
CURRENT_STAGE = 0
CURRENT_TEST = 0
CURRENT_TEST_TOTAL = 0
DEFAULT_MINIMUM_CAPTURE_SECONDS = 0.1
HIGH_RATE_SAMPLE_HZ = 8000
SITE_STOP_REQUEST = ROOT / 'site-estop.request'


def stage_label():
    return f'STAGE {CURRENT_STAGE}/{TOTAL_PHASES} | TEST {CURRENT_TEST}/{CURRENT_TEST_TOTAL}'


def print(*values, sep=' ', end='\n', file=None, flush=False):
    """Prefix every program status line while preserving normal print arguments."""
    value = sep.join(map(str, values)) + end
    lines = value.splitlines(keepends=True)
    for line in lines:
        builtins.print(f'{stage_label()} | {line}', end='', file=file, flush=flush)


def input(prompt=''):
    return builtins.input(f'{stage_label()} | {prompt}')


# ===== SETTINGS: EDIT THESE VALUES TO TUNE THE TEST =====
"""Sweep, configuration, calibration and safety settings for this rig.

The sequence applies persistent settings and calibrates on a fresh full run.
Runtime control settings are restored after a successful, verified stop.
Change these values only after reviewing the rig and GUI configuration.
"""

# Planned measurements: load board regulates positive velocity; test board
# applies negative mechanical torque. The test sensorless estimate is negative
# in its own coordinate system, even while the load encoder reads positive.
FIRST_SWEEP_RPM = 500
LAST_SWEEP_RPM = 1200
SWEEP_STEP_RPM = 20
TEST_OPPOSING_CURRENT_A = 0.5
TEST_SIGN_PROBE_CURRENT_A = 0.75
MINIMUM_SIGN_PROBE_DELTA_A = 0.3

# ODrive 0.6.12 controller enum values (runtime only).
TORQUE_CONTROL_MODE = 1
VELOCITY_CONTROL_MODE = 2
VELOCITY_RAMP_INPUT_MODE = 2
TORQUE_RAMP_INPUT_MODE = 6
IDLE_STATE = 1
CLOSED_LOOP_STATE = 8

# Firmware velocity ramp is in mechanical turns/s². It applies both to
# acceleration between points and to the commanded deceleration to zero.
LOAD_VELOCITY_RAMP_RPM_PER_S = 20.0
LOAD_VELOCITY_LIMIT_RPM = 1800.0
LOAD_VELOCITY_LIMIT_TOLERANCE = 1.2
TEST_VELOCITY_LIMIT_RPM = 1800.0
TEST_VELOCITY_LIMIT_TOLERANCE = 1.2
TEST_TORQUE_RAMP_A_PER_S = 1.0

# The load velocity controller starts the shaft before the test torque axis
# is armed. Sensorless entry still performs the firmware's open-loop ramp.
LOAD_FIRST_SENSORED_ARM_RPM = 100.0
LOAD_FIRST_SPEED_TOLERANCE_RPM = 20.0
LOAD_FIRST_MINIMUM_TRAVEL_TURN = 0.25
# Keep TEST disarmed until LOAD holds the highest configured speed.
SENSORLESS_HANDOVER_RPM = LAST_SWEEP_RPM
SENSORLESS_LOAD_FIRST_ARM_RPM = SENSORLESS_HANDOVER_RPM
SENSORLESS_MINIMUM_CAPTURE_RPM = 800.0
SENSORLESS_POLE_PAIRS = 3
SENSORLESS_ARM_STABLE_WINDOW_S = 1.0
SENSORLESS_ARM_MIN_SAMPLES = 5
SENSORLESS_ARM_POLL_S = 0.05
SENSORLESS_ARM_MAX_SAMPLE_AGE_S = 0.3
SENSORLESS_ARM_WINDOW_MIN_FRACTION = 0.9
SENSORLESS_ARM_INSTANT_TOLERANCE_RPM = 100.0
SENSORLESS_ARM_MEAN_TOLERANCE_RPM = 50.0
SENSORLESS_ARM_ESTIMATE_GRACE_SAMPLES = 5
SENSORLESS_ARM_ESTIMATE_GRACE_MAX_S = 0.5
SENSORLESS_SPEED_FAULT_CONSECUTIVE_SAMPLES = 3
SENSORLESS_HANDOVER_WINDOW_SAMPLES = 3
SENSORLESS_SAMPLE_MAX_AGE_S = 0.3
SENSORLESS_HANDOVER_MAX_WINDOW_S = 0.3
SENSORLESS_CAPTURE_CURRENT_TOLERANCE_A = 0.25
SENSORLESS_RAMP_ACCEL_EREV_PER_S2 = 1.0
SENSORLESS_FINISH_ON_VELOCITY = True
SENSORLESS_FINISH_ON_DISTANCE = False
SENSORLESS_USE_INITIAL_VELOCITY = False
STARTUP_DIRECTION_MIN_RPM = 180.0
STARTUP_DIRECTION_MAX_RPM = 400.0
STARTUP_ESTIMATE_MIN_RPM = 150.0
STARTUP_ESTIMATE_MAX_RPM = 450.0
STARTUP_COUPLING_RATIO_MIN = 0.7
STARTUP_COUPLING_RATIO_MAX = 1.3
STARTUP_MIN_TRAVEL_TURN = 1.0
STARTUP_STALL_TRAVEL_TURN = 0.25
STARTUP_STALL_CHECK_S = 8.0
STARTUP_TIMEOUT_MARGIN_S = 15.0
STARTUP_TIMEOUT_S = (SENSORLESS_HANDOVER_RPM * SENSORLESS_POLE_PAIRS / 60 /
                     SENSORLESS_RAMP_ACCEL_EREV_PER_S2 + STARTUP_TIMEOUT_MARGIN_S)
STARTUP_DIRECTION_HOLD_S = 0.5
HANDOVER_MEAN_MIN_RPM = SENSORLESS_HANDOVER_RPM - SENSORLESS_ARM_MEAN_TOLERANCE_RPM
HANDOVER_MEAN_MAX_RPM = SENSORLESS_HANDOVER_RPM + SENSORLESS_ARM_MEAN_TOLERANCE_RPM
HANDOVER_INSTANT_MIN_RPM = SENSORLESS_HANDOVER_RPM - SENSORLESS_ARM_INSTANT_TOLERANCE_RPM
HANDOVER_COUPLING_RATIO_MIN = 0.9
HANDOVER_COUPLING_RATIO_MAX = 1.1
HANDOVER_INPUT_VELOCITY_TOLERANCE_TURN_S = 0.1

# Host guards and settled-point requirements. The raw speed guard has a
# transient allowance; the ±50 RPM requirement applies to averaged captures.
LOAD_SPEED_SETTLING_TOLERANCE_RPM = 50.0
LOAD_SPEED_TRANSIENT_MARGIN_RPM = 100.0
LOAD_SPEED_TRANSIENT_FRACTION = 0.20
TEST_CURRENT_SETTLING_TOLERANCE_A = 0.5
MINIMUM_BUS_VOLTAGE_V = 15.0
MAXIMUM_BUS_VOLTAGE_V = 32.0
MAXIMUM_FET_OR_MOTOR_TEMPERATURE_C = 80.0
MAXIMUM_ABSOLUTE_SPEED_RPM = 1890.0
FIRMWARE_WATCHDOG_TIMEOUT_S = 15.0
SPEED_SETTLING_TIMEOUT_S = 120.0
STOP_TIMEOUT_S = 120.0
UNLOAD_TIMEOUT_S = 30.0
MINIMUM_COUPLING_PROBE_RPM = 45.0
COUPLING_PROBE_TIMEOUT_S = 20.0

# Motor current and protection limits.
CALIBRATION_CURRENT_A = 10.0
MOTOR_CALIBRATION_STATE = 4
ENCODER_OFFSET_CALIBRATION_STATE = 7
SENSORLESS_STARTUP_CURRENT_A = 4.0
SENSORLESS_STARTUP_CURRENT_RAMP_S = 2.0
# Negative electrical direction corresponds to positive load encoder motion.
SENSORLESS_RAMP_TARGET_EREV_PER_S = -(SENSORLESS_HANDOVER_RPM *
                                        SENSORLESS_POLE_PAIRS / 60)
MOTOR_SOFT_CURRENT_LIMIT_A = 50.0
MOTOR_HARD_CURRENT_TRIP_A = 56.0
MAXIMUM_ALLOWED_MOTOR_HARD_TRIP_A = 56.0

# Per-board DC bus limits. A separate combined guard protects the shared
# 40 A discharge / 30 A charge source.
DC_DISCHARGE_LIMIT_PER_BOARD_A = 30.0
DC_CHARGE_LIMIT_PER_BOARD_A = 30.0
COMBINED_DC_DISCHARGE_GUARD_A = 38.0
COMBINED_DC_CHARGE_GUARD_A = 28.0

# Velocity loop gains verified/restored at runtime.
VELOCITY_GAIN = 1.122
VELOCITY_INTEGRATOR_GAIN = 1.0

def validate():
    """Raise if settings are internally inconsistent."""
    assert MOTOR_HARD_CURRENT_TRIP_A >= MOTOR_SOFT_CURRENT_LIMIT_A, \
        'Hard trip must be >= soft current limit'
    assert MAXIMUM_ALLOWED_MOTOR_HARD_TRIP_A >= MOTOR_HARD_CURRENT_TRIP_A, \
        'Maximum allowed hard trip must be >= default hard trip'
    assert SENSORLESS_STARTUP_CURRENT_A > 0, \
        'Sensorless startup current must be positive'
    assert SENSORLESS_STARTUP_CURRENT_RAMP_S > 0, \
        'Sensorless startup current ramp time must be positive'
    assert SENSORLESS_RAMP_TARGET_EREV_PER_S != 0, \
        'Sensorless ramp target velocity must be non-zero'
    assert SENSORLESS_HANDOVER_RPM == LAST_SWEEP_RPM, \
        'Sensorless handover must be at the highest sweep speed'
    assert SENSORLESS_ARM_STABLE_WINDOW_S > 0 and SENSORLESS_ARM_MIN_SAMPLES >= 2, \
        'Sensorless arm hold needs a positive window and at least two samples'
    assert SENSORLESS_ARM_POLL_S > 0, 'Sensorless arm poll interval must be positive'
    assert SENSORLESS_ARM_MAX_SAMPLE_AGE_S > 0, 'Sensorless arm sample age must be positive'
    assert SENSORLESS_SPEED_FAULT_CONSECUTIVE_SAMPLES >= 2, \
        'Sensorless speed fault requires repeated samples'
    assert SENSORLESS_HANDOVER_WINDOW_SAMPLES >= 2, \
        'Sensorless handover requires multiple samples'
    assert SENSORLESS_SAMPLE_MAX_AGE_S > 0 and SENSORLESS_HANDOVER_MAX_WINDOW_S > 0, \
        'Sensorless sample age and handover window must be positive'
    assert SENSORLESS_CAPTURE_CURRENT_TOLERANCE_A > 0, \
        'Sensorless capture current tolerance must be positive'
    assert DC_DISCHARGE_LIMIT_PER_BOARD_A > 0, \
        'Per-board discharge limit must be positive'
    assert DC_CHARGE_LIMIT_PER_BOARD_A > 0, \
        'Per-board charge limit must be positive'
    assert COMBINED_DC_DISCHARGE_GUARD_A >= DC_DISCHARGE_LIMIT_PER_BOARD_A, \
        'Combined discharge guard must be >= per-board limit'
    assert COMBINED_DC_CHARGE_GUARD_A > 0, \
        'Combined charge guard must be positive'
    assert VELOCITY_GAIN > 0 and VELOCITY_INTEGRATOR_GAIN >= 0, \
        'Velocity gains must be positive (integrator may be zero)'
    assert FIRST_SWEEP_RPM <= LAST_SWEEP_RPM, \
        'First sweep RPM must be <= last sweep RPM'
    assert SWEEP_STEP_RPM > 0, 'Sweep step must be positive'
    assert MINIMUM_BUS_VOLTAGE_V < MAXIMUM_BUS_VOLTAGE_V, \
        'Minimum bus voltage must be below maximum'

# Edit the settings above; cfg keeps the original named access used by the workflow.
cfg = SimpleNamespace(**{name: value for name, value in globals().copy().items()
                         if name.isupper() and not name.startswith('_')})
cfg.validate = validate

# Optional site editor overrides. The standalone defaults above remain the
# source of truth; the local UI stores only explicitly edited values here.
_site_settings_path = Path(__file__).resolve().with_name('site-settings.json')
if _site_settings_path.is_file():
    _site_settings = json.loads(_site_settings_path.read_text(encoding='utf-8'))
    for _setting_name, _setting_value in _site_settings.items():
        if _setting_name in globals() and _setting_name.isupper() and _setting_name != 'DEFAULT_SETTINGS':
            globals()[_setting_name] = _setting_value
            setattr(cfg, _setting_name, _setting_value)
    if 'DEFAULT_SETTINGS' in _site_settings:
        DEFAULT_SETTINGS = _site_settings['DEFAULT_SETTINGS']


# ===== COMMON =====
"""Pure validation and durable artifacts; importing this package never discovers USB."""
import hashlib
import json
import math
import os
from dataclasses import dataclass, asdict
from pathlib import Path

SERIALS = {'test': '007D9BF1D9BB', 'load': '007D9BF1DCB4'}
SENSORLESS_STARTUP_A = cfg.SENSORLESS_STARTUP_CURRENT_A
SENSORLESS_RAMP_VEL_EREV_S = cfg.SENSORLESS_RAMP_TARGET_EREV_PER_S
SENSORLESS_TEST_SIGN = -1
CURRENT_LIMIT_A = cfg.MOTOR_SOFT_CURRENT_LIMIT_A
MOTOR_HARD_TRIP_A = cfg.MOTOR_HARD_CURRENT_TRIP_A
# Host checks the combined bus current independently of per-board limits.
DC_POSITIVE_PER_BOARD_A = cfg.DC_DISCHARGE_LIMIT_PER_BOARD_A
DC_NEGATIVE_PER_BOARD_A = cfg.DC_CHARGE_LIMIT_PER_BOARD_A
DC_COMBINED_POSITIVE_GUARD_A = cfg.COMBINED_DC_DISCHARGE_GUARD_A
DC_COMBINED_NEGATIVE_GUARD_A = cfg.COMBINED_DC_CHARGE_GUARD_A
GAINS = {'axis0.controller.config.vel_gain': cfg.VELOCITY_GAIN,
         'axis0.controller.config.vel_integrator_gain': cfg.VELOCITY_INTEGRATOR_GAIN}
LIMITS = {'config.dc_max_positive_current': DC_POSITIVE_PER_BOARD_A,
          'config.dc_max_negative_current': -DC_NEGATIVE_PER_BOARD_A,
          'axis0.config.motor.current_soft_max': CURRENT_LIMIT_A}
INVARIANTS = {**GAINS, **LIMITS}
MAPPINGS = ('axis0.config.load_encoder', 'axis0.config.commutation_encoder')
MOTOR_VALID = ('axis0.config.motor.phase_resistance_valid',
               'axis0.config.motor.phase_inductance_valid')
OFFSET = 'axis0.commutation_mapper.config.offset_valid'

class SafetyError(RuntimeError):
    pass

def require(ok, message):
    if not ok:
        raise SafetyError(message)

def finite(x):
    return isinstance(x, (int, float)) and math.isfinite(x)

def equal(a, b):
    return isinstance(a, (int, float)) and math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-7)

def clean(value):
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value

def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    with tmp.open('w') as f:
        json.dump(clean(data), f, indent=2, allow_nan=False)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    # Windows cannot open a directory with os.open() for fsync. The temporary
    # file has already been flushed and atomically replaced above.
    if os.name != 'nt':
        fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)

def digest(data):
    return hashlib.sha256(json.dumps(clean(data), sort_keys=True).encode()).hexdigest()

def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def exact_yes(answer):
    return answer.strip().lower() in ('', 'yes')

@dataclass(frozen=True)
class Point:
    mode: str
    rpm: float
    amps: float
    bandwidth: int | None = None

    def __post_init__(self):
        # Normalize numeric identity: 600 and 600.0 must have the same resume key.
        object.__setattr__(self, 'rpm', float(self.rpm))
        object.__setattr__(self, 'amps', float(self.amps))
        require(self.mode in ('sensored', 'sensorless'), 'Invalid feedback mode')
        require(finite(self.rpm) and 0 < self.rpm <= 1800, 'RPM must be > 0 and <= 1800')
        require(finite(self.amps) and 0 <= self.amps <= CURRENT_LIMIT_A,
                f'Opposing test Iq must be 0..{CURRENT_LIMIT_A:g} A')
        require(self.bandwidth in (None, 250, 500, 1000), 'Invalid feedback bandwidth')
        require(self.mode != 'sensorless' or self.rpm >= cfg.SENSORLESS_MINIMUM_CAPTURE_RPM,
                f'Sensorless test points must be >= {cfg.SENSORLESS_MINIMUM_CAPTURE_RPM:g} RPM; '
                f'startup is {cfg.SENSORLESS_HANDOVER_RPM:g} RPM')

    @property
    def key(self):
        return digest(asdict(self))[:20]

class Journal:
    """Completed captures are immutable. Interrupted points need an explicit retry flag."""
    def __init__(self, folder, points, resume=False, retry=False):
        self.folder = Path(folder)
        self.path = self.folder / 'matrix.json'
        plan = [asdict(p) for p in points]
        require(len({p.key for p in points}) == len(points), 'Duplicate matrix points')
        if self.path.exists():
            require(resume, 'Output exists: use --resume; completed points are never repeated')
            self.data = json.loads(self.path.read_text())
            require(self.data['plan'] == plan, 'Resume plan differs from saved plan')
            for key, entry in self.data['points'].items():
                if entry['status'] == 'complete':
                    for name, sha in entry['artifacts'].items():
                        require((self.folder / name).is_file() and file_hash(self.folder / name) == sha,
                                f'Completed artifact missing/changed: {name}; will not repeat it')
                else:
                    require(retry, f'Point {key} is {entry["status"]}; inspect it, then --retry-incomplete')
        else:
            require(not resume, 'No journal to resume')
            self.data = {'schema': 1, 'plan': plan, 'points': {}}
            self.save()

    def save(self):
        save_json(self.path, self.data)

    def pending(self, points):
        return [p for p in points if self.data['points'].get(p.key, {}).get('status') != 'complete']

    def begin(self, point, relative_folder):
        require(self.data['points'].get(point.key, {}).get('status') != 'complete', 'Already completed')
        self.data['points'][point.key] = {'status': 'running', 'folder': relative_folder}
        self.save()

    def finish(self, point, files):
        entry = self.data['points'][point.key]
        entry['artifacts'] = {str(Path(p).relative_to(self.folder)): file_hash(p) for p in files}
        entry['status'] = 'complete'
        self.save()

    def fail(self, point, error):
        self.data['points'][point.key].update(status='failed', error=repr(error))
        self.save()


# ===== HARDWARE =====
"""Bounded RuntimeDevice I/O for odrive 0.6.11.post1 and firmware 0.6.12."""
import asyncio

async def read(board, path):
    return await asyncio.wait_for(board.read(path), 1.0)

async def write(board, path, value):
    await asyncio.wait_for(board.write(path, value), 1.0)

async def checked_write(board, path, value):
    await write(board, path, value)
    require(equal(await read(board, path), value), f'Write/readback failed: {path}')

async def call(board, path, *args, timeout=1.0):
    return await asyncio.wait_for(board.call_function(path, *args), timeout)

async def connect(serial, finder=None):
    if finder is None:
        import odrive
        from odrive.runtime_device import RuntimeDevice
        async def finder(**kwargs):
            return await odrive.find_async(return_type=RuntimeDevice, **kwargs)
    try:
        b = await asyncio.wait_for(finder(serial_number=serial, interfaces=['usb']), 25)
    except Exception as exc:
        if type(exc).__name__ == 'CouldNotClaimInterfaceException':
            raise SafetyError(f'{serial}: USB interface is in use. Close the ODrive Web GUI '
                              'or other program connected to the board, then rerun. '
                              'On Linux, fuser -v /dev/bus/usb/*/* identifies open USB handles.') from exc
        raise
    require(await read(b, 'serial_number') == int(serial, 16), f'Wrong board for {serial}')
    # An awaited list comprehension is intentional: tuple(await ... for ...) is an async generator.
    version = tuple([int(await read(b, 'fw_version_' + p)) for p in ('major', 'minor', 'revision')])
    require(version == (0, 6, 12), f'{serial}: requires firmware 0.6.12, found {version}')
    return b

async def backup(board):
    from odrive.utils import backup_config
    return await asyncio.wait_for(backup_config(board), 30)

async def emergency_idle(boards):
    """Attempt both boards independently; IDLE has priority over clearing commands."""
    async def one(role, b):
        errors = []
        for path, value in [('axis0.requested_state', 1),
                            ('axis0.controller.input_torque', 0.0),
                            ('axis0.controller.input_vel', 0.0)]:
            try:
                await write(b, path, value)
            except Exception as e:
                errors.append(f'{role} {path}: {e!r}')
        try:
            deadline = asyncio.get_running_loop().time() + 3
            while True:
                if await read(b, 'axis0.current_state') == 1 and not await read(b, 'axis0.is_armed'):
                    break
                require(asyncio.get_running_loop().time() < deadline, f'{role}: IDLE/disarmed not confirmed')
                await asyncio.sleep(.05)
        except Exception as e:
            errors.append(f'{role}: {e!r}')
        return errors
    results = await asyncio.gather(*(one(r, b) for r, b in boards.items()))
    return [e for group in results for e in group]


# ===== SAFETY =====
"""Raw safety checks are separate from averaging used to judge settling."""
import asyncio
import math
import time
from collections import deque

SIGNALS = {
    'state': 'axis0.current_state', 'armed': 'axis0.is_armed',
    'errors': 'axis0.active_errors', 'disarm': 'axis0.disarm_reason',
    'rpm': 'axis0.vel_estimate', 'position': 'axis0.pos_estimate',
    'iq': 'axis0.motor.foc.Iq_measured', 'id': 'axis0.motor.foc.Id_measured',
    'iq_setpoint': 'axis0.motor.foc.Iq_setpoint',
    'vbus': 'vbus_voltage', 'ibus': 'ibus',
    'fet_C': 'axis0.motor.fet_thermistor.temperature',
    'phase_a': 'axis0.motor.alpha_beta_controller.current_meas_phA',
    'phase_b': 'axis0.motor.alpha_beta_controller.current_meas_phB',
    'phase_c': 'axis0.motor.alpha_beta_controller.current_meas_phC',
}

STANDSTILL_MEAN_RPM_MAX = 10.0
STANDSTILL_POSITION_RANGE_TURN_MAX = .006
SENSORLESS_MEAN_SPEED_DIFFERENCE_RPM = 50.0
# Raw USB reads can briefly disagree during sensorless startup.  This is an
# abort-only transient guard; the handover and capture checks still require
# tight, averaged 1:1 agreement before continuing.
SENSORLESS_INSTANT_SPEED_DIFFERENCE_RPM = 300.0

def coupled_speed_matches(test_rpm, load_rpm, sensorless):
    """Compare the two shaft estimates without hiding their direction checks.

    The load velocity loop regulates the RPM target. Sensorless speed can
    ripple by 50 RPM, so use an absolute allowance for its settling mean.
    Sensored feedback retains the tighter relative 1:1 check.
    """
    if abs(load_rpm) <= 1:
        return False
    if sensorless:
        return abs(abs(test_rpm) - abs(load_rpm)) <= SENSORLESS_MEAN_SPEED_DIFFERENCE_RPM
    return .94 <= abs(test_rpm / load_rpm) <= 1.06

def stationary_window(history, roles=('load',), window_s=1.0):
    """Reject real shaft travel while tolerating a noisy zero-speed estimate."""
    if not history:
        return False
    end = history[-1][0]
    rows = [(t, samples) for t, samples in history if t >= end - window_s - .2]
    if len(rows) < 5 or rows[-1][0] - rows[0][0] < window_s:
        return False
    for role in roles:
        positions = [samples[role]['position'] for _, samples in rows]
        rpms = [samples[role]['rpm'] for _, samples in rows]
        if not all(finite(v) for v in (*positions, *rpms)):
            return False
        if (max(positions) - min(positions) >= STANDSTILL_POSITION_RANGE_TURN_MAX or
                abs(sum(rpms) / len(rpms)) > STANDSTILL_MEAN_RPM_MAX):
            return False
    return True

def inspect_sample(role, s, expected=None, sensorless_test=False,
                   estimated_speed_grace=False):
    label = f'{role}: state={s["state"]} armed={s["armed"]} Iq={s["iq"]:+.3f} Id={s["id"]:+.3f}'
    idle_disarmed = s['state'] == 1 and not s['armed']
    phases = ('phase_a', 'phase_b', 'phase_c')
    for key, value in s.items():
        if key == 'rpm' and sensorless_test and estimated_speed_grace:
            continue  # The just-armed estimator may report a transient speed.
        if key in ('rpm', 'position') and role == 'test' and idle_disarmed:
            continue  # sensorless estimate can be unavailable at rest; load encoder is authoritative.
        if key == 'position' and role == 'test' and sensorless_test:
            continue  # Sensorless feedback supplies velocity, but no axis position estimate.
        if key in phases and not s['armed']:
            continue  # FOC phase-current telemetry can be unavailable during disarm transition.
        require(finite(value), f'{label}: nonfinite {key}')
    require(not s['errors'] and not s['disarm'], f'{label}: fault active={s["errors"]:#x} disarm={s["disarm"]:#x}')
    require(cfg.MINIMUM_BUS_VOLTAGE_V <= s['vbus'] <= cfg.MAXIMUM_BUS_VOLTAGE_V,
            f'{label}: voltage {s["vbus"]} outside {cfg.MINIMUM_BUS_VOLTAGE_V:g}..{cfg.MAXIMUM_BUS_VOLTAGE_V:g} V')
    require(-DC_NEGATIVE_PER_BOARD_A <= s['ibus'] <= DC_POSITIVE_PER_BOARD_A,
            f'{label}: DC bus current outside -{DC_NEGATIVE_PER_BOARD_A:g}..+{DC_POSITIVE_PER_BOARD_A:g} A')
    # Iq/Id measure motor current, not Ibus, and may be stale in IDLE. Firmware
    # retains the 50 A motor soft limit and 56 A hard trip.
    require(all(abs(s[k]) <= MOTOR_HARD_TRIP_A for k in phases if finite(s[k])),
            f'{label}: instantaneous phase current exceeds {MOTOR_HARD_TRIP_A:g} A')
    require(s['fet_C'] < cfg.MAXIMUM_FET_OR_MOTOR_TEMPERATURE_C and
            s.get('motor_C', 0) < cfg.MAXIMUM_FET_OR_MOTOR_TEMPERATURE_C,
            f'{label}: temperature >= {cfg.MAXIMUM_FET_OR_MOTOR_TEMPERATURE_C:g} C')
    if not (sensorless_test and estimated_speed_grace):
        require(not finite(s['rpm']) or abs(s['rpm']) <= cfg.MAXIMUM_ABSOLUTE_SPEED_RPM,
                f'{label}: overspeed >{cfg.MAXIMUM_ABSOLUTE_SPEED_RPM:g} RPM')
    if expected is not None:
        require(s['state'] == expected, f'{label}: unexpected state, expected {expected}')
        if expected == 8:
            require(s['armed'], f'{label}: closed loop but disarmed')
        elif expected == 1:
            require(not s['armed'], f'{label}: IDLE but armed')

class Monitor:
    def __init__(self, pair):
        self.pair = pair
        # Serialize state-changing commands with monitor samples. A USB batch
        # started under an armed expectation must finish before we request IDLE.
        self.sample_lock = asyncio.Lock()
        self.latest = {}
        self.history = deque()
        self.expected = {}
        self.watchdogs = False
        self.stage = 'idle'
        self.live = None
        self.frozen = False
        self.last_print = 0
        self.latest_read_started_at = None
        self.direction_sign = None
        self.require_steady = None
        self.load_speed_target_rpm = None
        # Disabled only during the brief sensorless-estimator acquisition
        # immediately after the test axis is armed at the already-proven load
        # speed. It is re-enabled before handover acceptance.
        self.sensorless_estimator_guard_active = True
        self.sensorless_arm_grace_remaining = 0
        self.sensorless_arm_grace_started_at = None
        self.sensorless_speed_fault_count = 0
        self.nonfinite_phase_counts = {'test': 0, 'load': 0}

    async def sample(self):
        async with self.sample_lock:
            return await self._sample_unlocked()

    async def request_sensorless_test_idle(self):
        async with self.sample_lock:
            self.expected.pop('test', None)
            await write(self.pair.boards['test'], 'axis0.requested_state', 1)

    async def request_closed_loop(self, role):
        async with self.sample_lock:
            self.expected.pop(role, None)
            await write(self.pair.boards[role], 'axis0.requested_state', 8)

    async def _sample_unlocked(self):
        # State expectations belong to the sample's read interval. An arm or
        # disarm command can complete while a USB batch is in flight; applying
        # the new expectation to a sample begun beforehand creates a false trip.
        expected = self.expected.copy()
        direction_sign = self.direction_sign
        require_steady = self.require_steady
        load_speed_target_rpm = self.load_speed_target_rpm
        sensorless_estimator_guard_active = self.sensorless_estimator_guard_active
        grace_active = (self.sensorless_arm_grace_remaining > 0 and
                        self.sensorless_arm_grace_started_at is not None and
                        time.monotonic() - self.sensorless_arm_grace_started_at <
                        cfg.SENSORLESS_ARM_ESTIMATE_GRACE_MAX_S)
        async def one(role, b):
            start = time.monotonic()
            # read_multiple is a USB batch, NOT a synchronized control-cycle capture.
            vals = await asyncio.wait_for(b.read_multiple(list(SIGNALS.values()), transient=True), 1)
            s = dict(zip(SIGNALS, vals))
            s['rpm'] *= 60
            if self.pair.motor_temp_enabled.get(role):
                s['motor_C'] = await read(b, 'axis0.motor.motor_thermistor.temperature')
            return role, s, start, time.monotonic()
        rows = await asyncio.gather(*(one(r, b) for r, b in self.pair.boards.items()))
        now = time.monotonic()
        samples = {r: s for r, s, _, _ in rows}
        grace_active = grace_active and samples['test']['armed'] and self.pair.meta.get('mode') == 'sensorless'
        if grace_active:
            self.sensorless_arm_grace_remaining -= 1
        raw = {'host_monotonic_s': now, 'stage': self.stage, 'boards': samples,
               'read_intervals_s': {r: [a, z] for r, _, a, z in rows},
               'estimated_combined_ibus_A': sum(s['ibus'] for s in samples.values()),
               'sensorless_estimated_speed_grace': grace_active}
        self.pair.log(raw)  # Write offending raw samples before any check throws.
        phases = ('phase_a', 'phase_b', 'phase_c')
        for role, s in samples.items():
            has_nonfinite_phase = s['armed'] and any(not finite(s[k]) for k in phases)
            if has_nonfinite_phase:
                self.nonfinite_phase_counts[role] += 1
                if self.nonfinite_phase_counts[role] > 3:
                    # Trip on repeated nonfinite phase measurements
                    inspect_sample(role, s, expected.get(role),
                                   sensorless_test=role == 'test' and self.pair.meta.get('mode') == 'sensorless',
                                   estimated_speed_grace=grace_active and role == 'test')
                else:
                    # Patch transient nonfinite phase values with 0.0 for this cycle so inspect_sample passes
                    patched_s = s.copy()
                    for k in phases:
                        if not finite(patched_s[k]):
                            patched_s[k] = 0.0
                    inspect_sample(role, patched_s, expected.get(role),
                                   sensorless_test=role == 'test' and self.pair.meta.get('mode') == 'sensorless',
                                   estimated_speed_grace=grace_active and role == 'test')
            else:
                self.nonfinite_phase_counts[role] = 0
                inspect_sample(role, s, expected.get(role),
                               sensorless_test=role == 'test' and self.pair.meta.get('mode') == 'sensorless',
                               estimated_speed_grace=grace_active and role == 'test')
        require(-DC_COMBINED_NEGATIVE_GUARD_A <= raw['estimated_combined_ibus_A'] <= DC_COMBINED_POSITIVE_GUARD_A,
                f'Estimated combined battery bus current outside -{DC_COMBINED_NEGATIVE_GUARD_A:g}..+{DC_COMBINED_POSITIVE_GUARD_A:g} A')
        if direction_sign is not None and abs(samples['load']['rpm']) > cfg.LOAD_SPEED_TRANSIENT_MARGIN_RPM:
            require(samples['load']['rpm'] * direction_sign > 0, 'Physical shaft direction reversed')
        if (sensorless_estimator_guard_active and self.pair.meta.get('mode') == 'sensorless' and
                samples['test']['armed'] and samples['load']['rpm'] >= cfg.SENSORLESS_MINIMUM_CAPTURE_RPM):
            # USB reads are not simultaneous and the sensorless estimate can
            # jump for one sample. Abort only on consecutive disagreements.
            disagrees = (samples['test']['rpm'] >= 0 or
                         abs(samples['test']['rpm'] + samples['load']['rpm']) >
                         SENSORLESS_INSTANT_SPEED_DIFFERENCE_RPM)
            self.sensorless_speed_fault_count = (self.sensorless_speed_fault_count + 1
                                                 if disagrees else 0)
            require(self.sensorless_speed_fault_count < cfg.SENSORLESS_SPEED_FAULT_CONSECUTIVE_SAMPLES,
                    'Sensorless velocity estimate lost the independent shaft direction/speed '
                    f'for {self.sensorless_speed_fault_count} consecutive samples')
        else:
            self.sensorless_speed_fault_count = 0
        if load_speed_target_rpm is not None:
            ceiling = load_speed_target_rpm + max(cfg.LOAD_SPEED_TRANSIENT_MARGIN_RPM,
                                                 cfg.LOAD_SPEED_TRANSIENT_FRACTION * load_speed_target_rpm)
            require(samples['load']['rpm'] <= ceiling,
                    f'Load exceeded transient target guard: {samples["load"]["rpm"]:.1f} > {ceiling:.1f} RPM')
        self.latest_read_started_at = min(start for _, _, start, _ in rows)
        self.record(now, samples)
        if require_steady:
            require(self.steady(*require_steady), 'Point left settling tolerances during capture')
        # Feed only after a fresh, fully checked pair sample. A stalled USB transfer
        # or dead monitor cannot keep an unobserved board armed indefinitely.
        if self.watchdogs:
            await asyncio.gather(*(call(b, 'axis0.watchdog_feed') for b in self.pair.boards.values()))
        if now - self.last_print >= 1:
            # All measured fields remain in monitor.jsonl. Keep the terminal narrow.
            print(' | '.join(
                f'{label} {samples[role]["rpm"]:+.0f}rpm '
                f'{samples[role]["ibus"]:+.2f}A {samples[role]["vbus"]:.1f}V'
                for role, label in (('test', 'T'), ('load', 'L'))), flush=True)
            self.last_print = now
            if self.live and not self.frozen:
                self.live.update(self.history)
        return samples

    def record(self, now, samples):
        self.latest = samples
        self.history.append((now, samples))
        # A board-local capture can briefly delay USB polling. Keep enough
        # healthy samples for the settling mean across a sub-second gap.
        while self.history and self.history[0][0] < now - 3.6:
            self.history.popleft()

    def means(self):
        # Require at least about 3 s of healthy samples. The retained window
        # can extend to 3.6 s so a brief USB capture gap does not erase the
        # evidence of stable speed and current.
        if len(self.history) < 10 or self.history[-1][0] - self.history[0][0] < 2.95:
            return None
        return {r: {k: sum(s[r][k] for _, s in self.history) / len(self.history)
                    for k in ('rpm', 'iq', 'id')} for r in ('test', 'load')}

    def sensorless_snapshot(self):
        """Short, fresh mean for startup handover confirmation."""
        rows = list(self.history)[-cfg.SENSORLESS_HANDOVER_WINDOW_SAMPLES:]
        if (len(rows) < cfg.SENSORLESS_HANDOVER_WINDOW_SAMPLES or
                rows[-1][0] - rows[0][0] > cfg.SENSORLESS_HANDOVER_MAX_WINDOW_S or
                time.monotonic() - rows[-1][0] > cfg.SENSORLESS_SAMPLE_MAX_AGE_S or
                any(not all(samples[role]['armed'] and samples[role]['state'] == cfg.CLOSED_LOOP_STATE
                            for role in ('test', 'load')) for _, samples in rows)):
            return None
        return {role: {key: sum(samples[role][key] for _, samples in rows) / len(rows)
                       for key in ('rpm', 'iq', 'id')}
                for role in ('test', 'load')}

    def sensorless_point_reading(self):
        """Return the latest checked pair sample when it is fresh and armed."""
        if not self.history or time.monotonic() - self.history[-1][0] > cfg.SENSORLESS_SAMPLE_MAX_AGE_S:
            return None
        samples = self.history[-1][1]
        if any(not samples[role]['armed'] or samples[role]['state'] != cfg.CLOSED_LOOP_STATE
               for role in ('test', 'load')):
            return None
        return {role: {key: samples[role][key] for key in ('rpm', 'iq', 'id')}
                for role in ('test', 'load')}

    def sensorless_speed_on_target(self, rpm):
        """Require the latest independent readings to agree at the target."""
        s = self.latest
        if not s or not s['test']['armed'] or not s['load']['armed']:
            return False
        test_rpm, load_rpm = s['test']['rpm'], s['load']['rpm']
        tolerance = cfg.LOAD_SPEED_SETTLING_TOLERANCE_RPM
        return (abs(test_rpm + rpm) <= tolerance and
                abs(load_rpm - rpm) <= tolerance)

    def stationary(self, roles=('load',), window_s=2.0):
        return stationary_window(self.history, roles, window_s)

    def steady(self, rpm, iq, ratio, test_sign=1):
        sensorless = test_sign == -1
        a = self.sensorless_point_reading() if sensorless else self.means()
        if a is None:
            return False
        if sensorless and not self.sensorless_speed_on_target(rpm):
            return False
        # Check both independent speed estimates against the regulated target.
        minimum = rpm - cfg.LOAD_SPEED_SETTLING_TOLERANCE_RPM
        maximum = rpm + cfg.LOAD_SPEED_SETTLING_TOLERANCE_RPM
        test_rpm = test_sign * a['test']['rpm']
        load_rpm = ratio * test_sign * a['load']['rpm']
        return (minimum <= test_rpm <= maximum and
                minimum <= load_rpm <= maximum and
                abs(a['load']['rpm']) > 1 and
                (sensorless or coupled_speed_matches(a['test']['rpm'], a['load']['rpm'], False)) and
                abs(a['test']['iq'] - iq) <=
                (cfg.SENSORLESS_CAPTURE_CURRENT_TOLERANCE_A if sensorless else
                 cfg.TEST_CURRENT_SETTLING_TOLERANCE_A))

    async def settle(self, rpm, iq, ratio, timeout=90, test_sign=1):
        if test_sign != -1:
            self.history.clear()
        start = time.monotonic()
        end = start + timeout
        since = None
        while time.monotonic() < end:
            fresh_after_command = (test_sign != -1 or
                                   self.latest_read_started_at is not None and
                                   self.latest_read_started_at >= start)
            if fresh_after_command and self.steady(rpm, iq, ratio, test_sign):
                if test_sign == -1:
                    return self.sensorless_point_reading()
                since = since or time.monotonic()
                if time.monotonic() - since >= 2:
                    return self.means()
            else:
                since = None
            await asyncio.sleep(.05)
        raise SafetyError(f'Settling failed: rpm={rpm}, signed test Iq={iq}')

    async def loop(self):
        while True:
            if SITE_STOP_REQUEST.exists():
                try:
                    SITE_STOP_REQUEST.unlink()
                except FileNotFoundError:
                    pass
                errors = await emergency_idle(self.pair.boards)
                raise SafetyError('Workbench E-stop requested; ' +
                                  ('IDLE confirmed on both boards.' if not errors else
                                   'IDLE confirmation errors: ' + '; '.join(errors)))
            await self.sample()
            await asyncio.sleep(.05)

async def supervise(operation, monitor):
    """A monitor failure cancels commands immediately. Caller always disarms in finally."""
    if SITE_STOP_REQUEST.exists():
        try:
            SITE_STOP_REQUEST.unlink()
        except FileNotFoundError:
            pass
        errors = await emergency_idle(monitor.pair.boards)
        raise SafetyError('Workbench E-stop requested before motion; ' +
                          ('IDLE confirmed on both boards.' if not errors else
                           'IDLE confirmation errors: ' + '; '.join(errors)))
    work = asyncio.create_task(operation)
    guard = asyncio.create_task(monitor.loop())
    try:
        done, _ = await asyncio.wait((work, guard), return_when=asyncio.FIRST_COMPLETED)
        if guard in done:
            await guard  # propagate the original safety/communications failure
            raise SafetyError('Monitor unexpectedly stopped')
        return await work
    finally:
        for t in (work, guard):
            if not t.done():
                t.cancel()
        await asyncio.gather(work, guard, return_exceptions=True)


# ===== CONFIGURATION =====
"""All persistent writes happen with both axes IDLE and disarmed, after backup."""
import asyncio
import importlib.metadata
import json
import math
import time
from pathlib import Path

STARTUP = ('startup_motor_calibration', 'startup_encoder_index_search',
           'startup_encoder_offset_calibration', 'startup_closed_loop_control', 'startup_homing')

def setting_matches(path, actual, expected):
    # The default 1.0 gains are exactly float32-representable and stay exact.
    # An operator-edited gain may round in the device's float32 storage.
    return actual == expected if path in GAINS and expected == 1.0 else equal(actual, expected)

class Pair:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.boards = {}
        self.motor_temp_enabled = {}
        self.meta = {'serials': SERIALS, 'firmware': [0, 6, 12], 'events': [], 'shutdown_errors': []}
        self.monitor = Monitor(self)
        self.logfile = (self.folder / 'monitor.jsonl').open('a', buffering=1)
        self.backup_count = 0
        self.watchdog_original = {}

    def log(self, entry):
        self.logfile.write(json.dumps(clean(entry), allow_nan=False) + '\n')

    def save(self):
        save_json(self.folder / 'session.json', self.meta)

    async def connect(self, allow_current_upgrade=False):
        self.meta['odrive_python'] = importlib.metadata.version('odrive')
        require(self.meta['odrive_python'] == '0.6.11.post1', 'Use pinned odrive 0.6.11.post1 package')
        for role, serial in SERIALS.items():
            print(f'Connecting {role} {serial}')
            self.boards[role] = await connect(serial)
        await self.idle_stopped()
        await self.protection_preflight(allow_current_upgrade=allow_current_upgrade)
        await self.monitor.sample()
        for role, s in self.monitor.latest.items():
            print(f'{role} {SERIALS[role]}: state={s["state"]}, armed={s["armed"]}, '
                  f'Iq={s["iq"]:+.3f} A, Id={s["id"]:+.3f} A, |I|={math.hypot(s["iq"], s["id"]):.3f} A, '
                  f'phases=({s["phase_a"]}, {s["phase_b"]}, {s["phase_c"]}) A')
            if any(not finite(s[k]) for k in ('phase_a', 'phase_b', 'phase_c')):
                print('  Phase-current telemetry is nonfinite while IDLE/disarmed. '
                      'It may be unavailable inactive FOC data or a sensor problem. '
                      'All three phases must become finite when the axis is active, or the run stops.')
            if math.hypot(s['iq'], s['id']) > 2:
                print('  IDLE current may be stale FOC data or a current-sensor problem. '
                      'This is not evidence of flowing current; raw phase readings and all guards remain active. '
                      'Resolve persistent implausible readings before authorizing motion.')
        self.save()

    async def idle_stopped(self):
        require(set(self.boards) == set(SERIALS), 'Both boards must be connected')
        for role, b in self.boards.items():
            require(await read(b, 'axis0.current_state') == 1 and not await read(b, 'axis0.is_armed'),
                    f'{role}: must be IDLE and disarmed')
        # Independent sensored load shaft observes the chain even when drive is sensorless.
        b = self.boards['load']
        history = []
        # Require two seconds between actual samples. Elapsed time after the
        # final sleep can reach the deadline before the last sample does.
        while not history or history[-1][0] - history[0][0] < 2.05:
            pos = await read(b, 'axis0.pos_estimate')
            rpm = 60 * await read(b, 'axis0.vel_estimate')
            history.append((time.monotonic(), {'load': {'position': pos, 'rpm': rpm}}))
            await asyncio.sleep(.08)
        positions = [sample['load']['position'] for _, sample in history]
        rpms = [sample['load']['rpm'] for _, sample in history]
        require(stationary_window(history, window_s=2.0),
                'Chain is not stopped or load encoder is invalid '
                f'(2 s window: position range={max(positions)-min(positions):.6f} turn, '
                f'mean velocity={sum(rpms)/len(rpms):+.3f} RPM; '
                'limits: position range <0.006 turn and mean velocity within ±10 RPM)')
        for role, board in self.boards.items():
            require(await read(board, 'axis0.current_state') == 1 and not await read(board, 'axis0.is_armed'),
                    f'{role}: left IDLE during standstill check')

    async def protection_preflight(self, allow_current_upgrade=False):
        from odrive.enums import EncoderId, MotorType
        for role, b in self.boards.items():
            for flag in STARTUP:
                require(not await read(b, 'axis0.config.' + flag), f'{role}: automatic {flag} must be disabled')
            require(await read(b, 'axis0.config.motor.motor_type') == MotorType.PMSM_CURRENT_CONTROL,
                    f'{role}: requires PMSM current control')
            require(await read(b, 'axis0.config.motor.pole_pairs') == cfg.SENSORLESS_POLE_PAIRS,
                    f'{role}: pole_pairs must be {cfg.SENSORLESS_POLE_PAIRS}')
            hard = await read(b, 'axis0.config.motor.current_hard_max')
            require(finite(hard) and (MOTOR_HARD_TRIP_A <= hard <= cfg.MAXIMUM_ALLOWED_MOTOR_HARD_TRIP_A or
                    (allow_current_upgrade and 0 < hard <= cfg.MAXIMUM_ALLOWED_MOTOR_HARD_TRIP_A)),
                    f'{role}: axis0.config.motor.current_hard_max is {hard} A; set it to '
                    f'{MOTOR_HARD_TRIP_A:g}..{cfg.MAXIMUM_ALLOWED_MOTOR_HARD_TRIP_A:g} A in the GUI for the '
                    f'{CURRENT_LIMIT_A:g} A motor soft limit, '
                    'save/reboot, and close the GUI before rerunning. No settings were written.')
            under = await read(b, 'config.dc_bus_undervoltage_trip_level')
            over = await read(b, 'config.dc_bus_overvoltage_trip_level')
            require(0 < under <= 15 and 32 <= over < 58, f'{role}: review firmware voltage trips')
            # FET protection has no enabled property in the 0.6.12 API.
            self.motor_temp_enabled[role] = bool(await read(b, 'axis0.motor.motor_thermistor.config.enabled'))
            for kind in ('fet', 'motor'):
                if kind == 'motor' and not self.motor_temp_enabled[role]:
                    print(f'{role}: motor thermistor disabled in existing configuration; no winding-temperature measurement')
                    continue
                prefix = 'config.inverter0.' if kind == 'fet' else 'axis0.motor.motor_thermistor.config.'
                lo, hi = await read(b, prefix+'temp_limit_lower'), await read(b, prefix+'temp_limit_upper')
                require(finite(lo) and finite(hi) and 0 < lo < hi <= 150, f'{role}: invalid {kind} temperature protection')
            require(await read(b, 'onboard_encoder0.status') == 0, f'{role}: onboard encoder fault')
            if role == 'load':
                for p in MAPPINGS:
                    require(await read(b, p) == EncoderId.ONBOARD_ENCODER0, 'Load must remain on onboard encoder')

    async def verify(self, values=INVARIANTS):
        for role, b in self.boards.items():
            for p, v in values.items():
                require(setting_matches(p, await read(b, p), v), f'{role}: invariant failed: {p} must equal {v}')

    async def snapshot(self, label):
        self.backup_count += 1
        data = {role: await backup(b) for role, b in self.boards.items()}
        path = self.folder / f'backup_{self.backup_count:03d}_{label}.json'
        # Preserve ODrive NaN/Infinity sentinels for compatibility with its restore tools.
        import os
        with path.open('x') as f:
            json.dump(data, f, indent=2, allow_nan=True)
            f.flush()
            os.fsync(f.fileno())
        self.meta['events'].append({'backup': path.name, 'at': time.time()})
        self.save()
        return data

    async def persist(self, role, expected):
        await self.idle_stopped()
        b = self.boards[role]
        # Counter reset demonstrates a reboot even if the board was just powered up.
        old_cycles = await read(b, 'n_evt_control_loop')
        deadline = time.monotonic() + 12
        while old_cycles < 80000:
            require(time.monotonic() < deadline, f'{role}: control-loop counter did not advance')
            await asyncio.sleep(.5)
            old_cycles = await read(b, 'n_evt_control_loop')
        try:
            result = await call(b, 'save_configuration', timeout=5)
        except Exception as e:
            self.meta['events'].append({'save_reply_lost': repr(e), 'role': role})
        else:
            require(result is not False, f'{role}: save_configuration rejected')
        await asyncio.sleep(1)
        fresh = await connect(SERIALS[role])
        self.boards[role] = fresh
        require(await read(fresh, 'n_evt_control_loop') < old_cycles, f'{role}: reboot not verified')
        await self.idle_stopped()
        for p, v in expected.items():
            require(setting_matches(p, await read(fresh, p), v), f'{role}: {p} did not persist')
        await self.verify()
        await self.protection_preflight()
        await self.monitor.sample()
        self.meta['events'].append({'saved_rebooted_verified': role, 'expected': expected})
        self.save()

    async def invariant_setup(self, raise_hard_trip=False):
        await self.idle_stopped()
        await self.snapshot('before_invariants')
        # Set both first; no calibration or test can happen on a half-configured pair.
        for b in self.boards.values():
            if raise_hard_trip and not equal(await read(b, 'axis0.config.motor.current_hard_max'), MOTOR_HARD_TRIP_A):
                await checked_write(b, 'axis0.config.motor.current_hard_max', MOTOR_HARD_TRIP_A)
            for p, v in INVARIANTS.items():
                if not setting_matches(p, await read(b, p), v):
                    await checked_write(b, p, v)
        await self.protection_preflight()
        await self.verify()
        # Persist even if RAM already matched: readback alone doesn't prove NVM.
        for role in SERIALS:
            await self.persist(role, INVARIANTS)

    async def configure_full_sweep(self):
        """Save named settings for the full sweep without changing feedback or calibrating.

        The complete backup precedes every write. Both boards receive runtime
        readback before either is saved; changed boards are then rebooted and
        checked by serial. Repeated runs skip flash saves when values match.
        """
        cfg.validate()
        await self.idle_stopped()
        for role in SERIALS:
            await self.calibrated(role)

        shared = {**INVARIANTS, 'axis0.config.motor.current_hard_max': MOTOR_HARD_TRIP_A}
        # Make the firmware's default sensorless motor model explicit and
        # persistent.  This avoids relying on an unset optional field while
        # retaining the documented relationship to the calibrated motor Kt.
        sensorless_flux_linkage = await self.derived_sensorless_flux_linkage()
        sensorless_ramp = {
            'axis0.config.sensorless_ramp.vel': SENSORLESS_RAMP_VEL_EREV_S,
            'axis0.config.sensorless_ramp.accel': cfg.SENSORLESS_RAMP_ACCEL_EREV_PER_S2,
            'axis0.config.sensorless_ramp.current': SENSORLESS_STARTUP_A,
            'axis0.config.sensorless_ramp.ramp_time': cfg.SENSORLESS_STARTUP_CURRENT_RAMP_S,
            'axis0.config.sensorless_ramp.finish_on_vel': cfg.SENSORLESS_FINISH_ON_VELOCITY,
            'axis0.config.sensorless_ramp.finish_on_distance': cfg.SENSORLESS_FINISH_ON_DISTANCE,
            'axis0.config.use_init_vel_for_sensorless': cfg.SENSORLESS_USE_INITIAL_VELOCITY,
            'axis0.config.motor.sensorless_pm_flux_linkage': sensorless_flux_linkage,
            'axis0.config.motor.sensorless_pm_flux_linkage_valid': True,
        }
        desired = {'load': shared, 'test': {**shared, **sensorless_ramp}}
        differences = {}
        for role, values in desired.items():
            differences[role] = {path: value for path, value in values.items()
                                 if not setting_matches(path, await read(self.boards[role], path), value)}
        if not any(differences.values()):
            print('CONFIGURE: persistent settings already match; no save/reboot needed.', flush=True)
            return differences

        await self.snapshot('before_full_sweep_configuration')
        for role, changes in differences.items():
            for path, value in changes.items():
                await checked_write(self.boards[role], path, value)
        await self.verify()
        await self.protection_preflight()
        for role, changes in differences.items():
            if changes:
                print(f'CONFIGURE: saving and rebooting {role}; changed {list(changes)}', flush=True)
                await self.persist(role, desired[role])
        await self.idle_stopped()
        for role, values in desired.items():
            for path, value in values.items():
                require(setting_matches(path, await read(self.boards[role], path), value),
                        f'{role}: configuration did not persist: {path}')
        self.meta['events'].append({'full_sweep_configuration': differences})
        self.save()
        return differences

    async def change_persistent(self, role, changes, label):
        await self.idle_stopped()
        await self.verify()
        b = self.boards[role]
        actual = {p: await read(b, p) for p in changes}
        if all(setting_matches(p, actual[p], v) for p, v in changes.items()):
            return False
        await self.snapshot(label)
        for p, v in changes.items():
            if not setting_matches(p, actual[p], v):
                await checked_write(b, p, v)
        await self.persist(role, {**INVARIANTS, **changes})
        return True

    async def calibrated(self, role, encoder=True):
        b = self.boards[role]
        for p in (*MOTOR_VALID, *((OFFSET,) if encoder else ())):
            require(await read(b, p), f'{role}: calibration invalid: {p}; commission first')
        require(await read(b, 'axis0.config.motor.direction') in (-1, 1), f'{role}: uncalibrated motor direction')
        kt = await read(b, 'axis0.config.motor.torque_constant')
        require(finite(kt) and kt > 0, f'{role}: invalid torque constant')

    async def derived_sensorless_flux_linkage(self):
        """Return ODrive's documented PM flux-linkage default for the test motor."""
        motor = self.boards['test']
        torque_constant = await read(motor, 'axis0.config.motor.torque_constant')
        pole_pairs = await read(motor, 'axis0.config.motor.pole_pairs')
        require(finite(torque_constant) and torque_constant > 0,
                'test: invalid torque constant for sensorless flux linkage')
        require(isinstance(pole_pairs, (int, float)) and pole_pairs > 0 and float(pole_pairs).is_integer(),
                'test: invalid pole-pair count for sensorless flux linkage')
        return (2.0 / 3.0) * torque_constant / pole_pairs

    async def enable_watchdogs(self):
        for role, b in self.boards.items():
            if role not in self.watchdog_original:
                self.watchdog_original[role] = {p: await read(b, p) for p in
                    ('axis0.config.watchdog_timeout', 'axis0.config.enable_watchdog')}
            await checked_write(b, 'axis0.config.watchdog_timeout', cfg.FIRMWARE_WATCHDOG_TIMEOUT_S)
            await call(b, 'axis0.watchdog_feed')
            await checked_write(b, 'axis0.config.enable_watchdog', True)
        self.monitor.watchdogs = True

    async def restore_watchdogs_idle(self):
        # Restore the caller's idle policy only after positive IDLE/disarmed readback.
        # Never remove watchdog protection from an axis whose state is uncertain.
        errors = []
        for role, values in list(self.watchdog_original.items()):
            try:
                b = self.boards[role]
                require(await read(b, 'axis0.current_state') == 1 and not await read(b, 'axis0.is_armed'),
                        f'{role}: refusing watchdog restore without IDLE/disarmed')
                await checked_write(b, 'axis0.config.watchdog_timeout', values['axis0.config.watchdog_timeout'])
                await call(b, 'axis0.watchdog_feed')
                await checked_write(b, 'axis0.config.enable_watchdog', values['axis0.config.enable_watchdog'])
                del self.watchdog_original[role]
            except Exception as e:
                errors.append(f'{role}: watchdog restoration: {e!r}')
        return errors

    async def wait_stopped(self, timeout=30):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                await self.idle_stopped()
                return
            except SafetyError as e:
                if not str(e).startswith('Chain is not stopped or load encoder is invalid'):
                    raise
        raise TimeoutError('Chain failed to stop after calibration')

    async def calibration(self, role, state):
        await self.idle_stopped()
        await self.verify()
        await self.change_persistent(role, {'axis0.config.motor.calibration_current': cfg.CALIBRATION_CURRENT_A,
                                            'axis0.config.calibration_lockin.current': cfg.CALIBRATION_CURRENT_A}, 'calibration_current')
        await self.snapshot(f'before_calibration_{role}')
        await self.enable_watchdogs()
        await self.monitor.sample()
        async def run():
            b = self.boards[role]
            other = 'load' if role == 'test' else 'test'
            self.monitor.expected = {other: 1}
            self.monitor.stage = f'calibrating {role} state {state}'
            await write(b, 'axis0.requested_state', state)
            started = False
            end = time.monotonic() + 60
            while time.monotonic() < end:
                s, result = await read(b, 'axis0.current_state'), await read(b, 'axis0.procedure_result')
                started |= s == state or result == 1  # ProcedureResult.BUSY
                if started and s == 1 and result != 1:
                    require(result == 0, f'{role}: calibration procedure_result={result}')
                    self.monitor.expected[role] = 1
                    await self.wait_stopped()
                    return
                require(s in (1, state), f'{role}: unexpected calibration state {s}')
                await asyncio.sleep(.05)
            raise TimeoutError(f'{role}: calibration did not complete')
        try:
            await supervise(run(), self.monitor)
        finally:
            self.monitor.watchdogs = False
            self.monitor.expected = {}
            errors = await emergency_idle(self.boards)
            errors += await self.restore_watchdogs_idle()
            self.meta['shutdown_errors'].extend(errors)
        require(not errors, f'Calibration shutdown failed: {errors}')
        await self.idle_stopped()
        checks = {p: True for p in (MOTOR_VALID if state == 4 else (OFFSET,))}
        for p in checks:
            require(await read(self.boards[role], p), f'{role}: calibration result invalid {p}')
        await self.persist(role, {**INVARIANTS, **checks})

    async def prepare_mode(self, mode, authorized):
        from odrive.enums import EncoderId
        require(authorized, 'Motion authorization is required before feedback preparation')
        await self.calibrated('load')
        await self.calibrated('test', encoder=False)
        if mode == 'sensored':
            changed = await self.change_persistent('test', dict.fromkeys(MAPPINGS, EncoderId.ONBOARD_ENCODER0), 'sensored_mapping')
            if changed or not await read(self.boards['test'], OFFSET):
                await self.calibration('test', cfg.ENCODER_OFFSET_CALIBRATION_STATE)
        else:
            require(await read(self.boards['test'], 'axis0.config.motor.sensorless_observer_gain') > 0,
                    'Sensorless observer gain is not configured')
            flux_linkage = await self.derived_sensorless_flux_linkage()
            changes = {**dict.fromkeys(MAPPINGS, EncoderId.SENSORLESS_ESTIMATOR),
                       # The earlier +22.5 trial moved the independent load
                       # encoder backward. Reverse electrical ramp direction;
                       # the independent encoder must verify the result.
                       'axis0.config.sensorless_ramp.vel': SENSORLESS_RAMP_VEL_EREV_S,
                       'axis0.config.sensorless_ramp.accel': cfg.SENSORLESS_RAMP_ACCEL_EREV_PER_S2,
                       'axis0.config.sensorless_ramp.current': SENSORLESS_STARTUP_A,
                       'axis0.config.sensorless_ramp.ramp_time': cfg.SENSORLESS_STARTUP_CURRENT_RAMP_S,
                       'axis0.config.sensorless_ramp.finish_on_vel': cfg.SENSORLESS_FINISH_ON_VELOCITY,
                       'axis0.config.sensorless_ramp.finish_on_distance': cfg.SENSORLESS_FINISH_ON_DISTANCE,
                       'axis0.config.use_init_vel_for_sensorless': cfg.SENSORLESS_USE_INITIAL_VELOCITY,
                       'axis0.config.motor.sensorless_pm_flux_linkage': flux_linkage,
                       'axis0.config.motor.sensorless_pm_flux_linkage_valid': True}
            await self.change_persistent('test', changes, 'sensorless_mapping')
        await self.verify()
        await self.calibrated('test', encoder=mode == 'sensored')
        await self.calibrated('load')
        self.meta['mode'] = mode
        self.save()

    async def verify_prepared_mode(self, mode):
        """Accept GUI-prepared feedback without saving, rebooting or calibrating.

        The load board is always sensored. The test board must already have the
        requested feedback mapping and, for sensorless startup, the exact ramp
        settings that the paired validation and direction checks expect.
        """
        from odrive.enums import EncoderId
        await self.idle_stopped()
        await self.verify()
        await self.calibrated('load')
        await self.calibrated('test', encoder=mode == 'sensored')
        require(mode in ('sensored', 'sensorless'), 'Invalid prepared feedback mode')
        expected = EncoderId.ONBOARD_ENCODER0 if mode == 'sensored' else EncoderId.SENSORLESS_ESTIMATOR
        for path in MAPPINGS:
            require(await read(self.boards['test'], path) == expected,
                    f'test: set {path} to {expected} in GUI before continuing')
        if mode == 'sensorless':
            require(await read(self.boards['test'], 'axis0.config.motor.sensorless_observer_gain') > 0,
                    'test: sensorless observer gain must be configured in GUI')
            flux_linkage = await self.derived_sensorless_flux_linkage()
            settings = {'axis0.config.sensorless_ramp.vel': SENSORLESS_RAMP_VEL_EREV_S,
                        'axis0.config.sensorless_ramp.accel': cfg.SENSORLESS_RAMP_ACCEL_EREV_PER_S2,
                        'axis0.config.sensorless_ramp.current': SENSORLESS_STARTUP_A,
                        'axis0.config.sensorless_ramp.ramp_time': cfg.SENSORLESS_STARTUP_CURRENT_RAMP_S,
                        'axis0.config.sensorless_ramp.finish_on_vel': cfg.SENSORLESS_FINISH_ON_VELOCITY,
                        'axis0.config.sensorless_ramp.finish_on_distance': cfg.SENSORLESS_FINISH_ON_DISTANCE,
                        'axis0.config.use_init_vel_for_sensorless': cfg.SENSORLESS_USE_INITIAL_VELOCITY,
                        'axis0.config.motor.sensorless_pm_flux_linkage': flux_linkage,
                        'axis0.config.motor.sensorless_pm_flux_linkage_valid': True}
            for path, value in settings.items():
                require(setting_matches(path, await read(self.boards['test'], path), value),
                        f'test: set {path} to {value} in GUI before continuing')
        self.meta['mode'] = mode
        self.save()

    async def fingerprint(self):
        # Only mode-independent hardware/observer parameters: mapper offsets legitimately
        # change on mode transitions. Direction changes DO invalidate startup evidence.
        paths = ['axis0.config.motor.' + p for p in ('pole_pairs', 'direction', 'torque_constant',
                 'phase_resistance', 'phase_inductance', 'sensorless_observer_gain',
                 'sensorless_pm_flux_linkage', 'sensorless_pm_flux_linkage_valid')]
        paths += ['axis0.config.sensorless_ramp.' + p for p in ('initial_pos', 'ramp_time', 'ramp_distance')] 
        config = {r: {p: await read(b, p) for p in paths} for r, b in self.boards.items()}
        return digest({'serials': SERIALS, 'firmware': [0, 6, 12], 'config': config,
                       'startup_rpm': cfg.SENSORLESS_HANDOVER_RPM, 'startup_A': SENSORLESS_STARTUP_A,
                       'startup_ramp_vel_erev_s': SENSORLESS_RAMP_VEL_EREV_S,
                       'startup_rpm_s': 20})

    async def close(self):
        self.monitor.watchdogs = False
        errors = await emergency_idle(self.boards)
        errors += await self.restore_watchdogs_idle()
        # ODrive's device manager retains USB ownership until each connection
        # is explicitly released. The full-run wrapper starts the next phase
        # in a subprocess, so merely leaving this coroutine/loop is not enough.
        try:
            from odrive.runtime_device import RuntimeDevice
            from odrive.device_manager import get_device_manager
            live = [(role, board) for role, board in self.boards.items()
                    if isinstance(board, RuntimeDevice)]
            if live:
                manager = get_device_manager()
                for role, board in live:
                    try:
                        manager.release_connection(board)
                    except Exception as exc:
                        errors.append(f'{role}: USB release: {exc!r}')
        except ImportError:
            pass  # Fake boards in offline tests have no ODrive device manager.
        self.meta['shutdown_errors'].extend(errors)
        if errors:
            print('IDLE could not be confirmed: use physical stop. ' + '; '.join(errors))
        self.save()
        self.logfile.close()
        return errors


# ===== CAPTURE =====
"""Single multi-channel recorder per board; independent board clocks."""
import asyncio
import csv
import math
import time

REQUIRED = {
    'position_turn': 'axis0.pos_estimate',
    'velocity_turn_s': 'axis0.vel_estimate',
    'phase_a_A': 'axis0.motor.alpha_beta_controller.current_meas_phA',
    'phase_b_A': 'axis0.motor.alpha_beta_controller.current_meas_phB',
    'phase_c_A': 'axis0.motor.alpha_beta_controller.current_meas_phC',
}
SENSORLESS_REQUIRED = {
    'sensorless_phase_rad': 'axis0.motor.sensorless_estimator.pll_pos',
    'measured_encoder_raw_turn': 'onboard_encoder0.raw',
    **{name: path for name, path in REQUIRED.items() if name != 'position_turn'},
}
OPTIONAL = {
    'iq_A': 'axis0.motor.foc.Iq_measured',
    'ibus_A': 'ibus',
    'vbus_V': 'vbus_voltage',
}

def plan_channels(size, rate, max_inputs, available, minimum_duration=.1, required=REQUIRED):
    require(finite(rate) and rate == HIGH_RATE_SAMPLE_HZ, 'Expected 8 kHz control loop for this pinned capture API')
    require(size > 0 and max_inputs >= 5, 'Insufficient capture buffer/channel slots')
    require(all(p in available for p in required.values()), 'Mandatory high-rate channel unavailable')
    channels = dict(required)
    require(size // (4 * len(channels)) >= math.ceil(rate * minimum_duration),
            'Buffer cannot hold requested minimum duration with all mandatory channels')
    for name, path in OPTIONAL.items():
        n = len(channels) + 1
        if path in available and n <= max_inputs and size // (4*n) >= math.ceil(rate*minimum_duration):
            channels[name] = path
    return channels, size // (4 * len(channels))

# Maximum fraction and minimum count of non-finite samples allowed per phase
# current channel. The FOC pipeline occasionally produces transient NaN
# readings at high speed; a strict all-finite requirement would abort otherwise
# valid captures. Navigation channels (velocity, position, encoder) remain
# strictly all-finite because they feed position and error computations.
_PHASE_NONFINITE_MAX_FRACTION = 0.01   # 1 % of samples
_PHASE_NONFINITE_MAX_COUNT    = 10      # at least this many are always allowed

# Channel names whose paths contain phase current readings.
_PHASE_CHANNEL_NAMES = frozenset(('phase_a_A', 'phase_b_A', 'phase_c_A'))

def validate_raw(raw, channels, samples):
    """Validate downloaded capture buffer.

    Phase current channels tolerate up to _PHASE_NONFINITE_MAX_FRACTION (or
    _PHASE_NONFINITE_MAX_COUNT, whichever is larger) non-finite samples per
    channel; callers must replace those samples with 0.0 before use.

    Returns a dict {channel_path: nonfinite_count} for phase channels; zero
    counts are included so callers can log the result unconditionally.
    """
    t = raw['timestamps']
    require(len(t) == samples and t[0] == 0 and all(b-a == 1 for a, b in zip(t, t[1:])),
            'Capture is truncated, mis-triggered or has invalid control-cycle indices')
    threshold = max(_PHASE_NONFINITE_MAX_COUNT,
                    int(math.ceil(samples * _PHASE_NONFINITE_MAX_FRACTION)))
    phase_nonfinite = {}
    for name, p in channels.items():
        data = raw[p]
        require(len(data) == samples, f'Wrong sample count in capture channel {p}')
        if name in _PHASE_CHANNEL_NAMES:
            nf = sum(1 for v in data if not finite(v))
            require(nf <= threshold,
                    f'Too many non-finite samples in phase channel {p}: '
                    f'{nf}/{samples} exceeds tolerance of {threshold}')
            phase_nonfinite[p] = nf
        else:
            require(all(finite(v) for v in data), f'Invalid capture channel {p}')
    return phase_nonfinite

def sensorless_position_turn(phase_rad, pole_pairs):
    """Relative mechanical turns from wrapped electrical PLL phase samples."""
    require(pole_pairs > 0 and all(finite(v) for v in phase_rad), 'Invalid sensorless phase capture')
    turns = [0.0]
    for previous, current in zip(phase_rad, phase_rad[1:]):
        delta = (current - previous + math.pi) % (2 * math.pi) - math.pi
        turns.append(turns[-1] + delta / (2 * math.pi * pole_pairs))
    return turns

def measured_position_turn(raw_turn):
    """Relative turns from the onboard encoder's wrapped one-turn raw angle."""
    require(raw_turn and all(finite(v) and 0 <= v < 1 for v in raw_turn),
            'Invalid onboard encoder raw capture')
    turns = [0.0]
    for previous, current in zip(raw_turn, raw_turn[1:]):
        delta = (current - previous + .5) % 1 - .5
        turns.append(turns[-1] + delta)
    return turns

class Capture:
    def __init__(self, pair, minimum_duration=.1):
        self.pair = pair
        self.minimum_duration = minimum_duration
        self.groups = {}
        self.plans = {}
        self.configuration_snapshot = None

    async def prepare(self, mode=None):
        from odrive.utils import HighRateCapturer
        self.groups.clear()
        for role, b in self.pair.boards.items():
            rate, size = await read(b, 'control_loop_hz'), await read(b, 'oscilloscope.size')
            slots = len(b.functions['oscilloscope.config'].inputs)
            sensorless = role == 'test' and (mode or self.pair.meta.get('mode')) == 'sensorless'
            channels, count = plan_channels(size, rate, slots, b.properties, self.minimum_duration,
                                            SENSORLESS_REQUIRED if sensorless else REQUIRED)
            if sensorless:
                require(await read(b, 'onboard_encoder0.status') == 0,
                        'Test onboard encoder is not healthy for measured-position capture')
            for p in channels.values():
                await read(b, p)
            cap = await asyncio.wait_for(HighRateCapturer.from_properties(b, list(channels.values()), unsafe=False), 5)
            self.groups[role] = cap
            self.plans[role] = {'channels': channels, 'samples': count, 'sample_rate_hz': rate,
                                'duration_s': count/rate, 'bin_width_Hz': rate/count,
                                'buffer_bytes': size, 'omitted_optional': [k for k in OPTIONAL if k not in channels]}
            if sensorless:
                self.plans[role]['position_derivation'] = {
                    'source': 'sensorless_phase_rad', 'pole_pairs': int(await read(b, 'axis0.config.motor.pole_pairs')),
                    'method': 'unwrap electrical PLL phase / (2*pi*pole_pairs)',
                    'reference': 'relative to first capture sample; no absolute position claim'}
                self.plans[role]['measured_position_derivation'] = {
                    'source': 'measured_encoder_raw_turn', 'method': 'unwrap onboard encoder raw turn angle',
                    'reference': 'relative to first capture sample; independent of active sensorless feedback'}
        # Take the slow full configuration backup before motion so a brief
        # sensorless speed window can trigger its high-rate capture promptly.
        self.configuration_snapshot = {role: await backup(board)
                                       for role, board in self.pair.boards.items()}
        return self.plans

    async def acquire(self, folder, point, ratio, signed_iq):
        from odrive.utils import TimestampFmt
        folder.mkdir(parents=True, exist_ok=False)
        metadata = {'schema': 2, 'point': point.__dict__, 'serials': self.pair.meta['serials'],
                    'firmware': self.pair.meta['firmware'], 'odrive_python': self.pair.meta['odrive_python'],
                    'boards': self.plans, 'test_to_load_velocity_ratio': ratio, 'test_iq_target_A': signed_iq,
                    'control_assignment': {'test': 'opposing_torque', 'load': 'positive_velocity_control'},
                    'speed_note': 'Point RPM is the positive load velocity target; use measured velocity for analysis.',
                    'clock_note': 'Channels share control-cycle indices within a board. USB board clocks are NOT synchronized.',
                    'bandwidth_note': '8 kHz control-loop samples; not a measurement of all PWM ripple.',
                    'status': 'capturing', 'session_folder': str(self.pair.folder.resolve()), 'created_unix_s': time.time()}
        metadata['load_speed_target_rpm'] = 60 * await read(self.pair.boards['load'], 'axis0.controller.input_vel')
        metadata['load_velocity_limit_rpm'] = 60 * await read(self.pair.boards['load'], 'axis0.controller.config.vel_limit')
        metadata['test_opposing_torque_command_Nm'] = await read(self.pair.boards['test'], 'axis0.controller.input_torque')
        # Snapshot before entering the short high-rate window. Monitor stays active.
        require(self.configuration_snapshot is not None, 'High-rate capture was not prepared')
        metadata['configuration'] = self.configuration_snapshot
        metadata['trigger_policy'] = ('first fresh sensorless RPM and TEST Iq reading'
                                      if point.mode == 'sensorless' else 'settled speed/current window')
        if point.mode == 'sensorless':
            metadata['capture_quality_note'] = ('The sensorless trigger is opportunistic. '
                                                'Use measured RPM and current in the CSV to assess the full window.')
        metadata['before'] = self.pair.monitor.latest
        save_json(folder / 'capture.json', metadata)
        self.pair.monitor.frozen = True
        self.pair.monitor.stage = 'high-rate capture'
        # The sensorless window may be short. The continuously running monitor
        # still checks firmware faults, bus current/voltage, phases, temperature,
        # overspeed and repeated estimator disagreement during the capture.
        self.pair.monitor.require_steady = (None if point.mode == 'sensorless' else
                                            (point.rpm, signed_iq, ratio, 1))
        try:
            await asyncio.gather(*(asyncio.wait_for(c.start(), 2) for c in self.groups.values()))
            async def trigger(role, cap):
                before = time.monotonic()
                await asyncio.wait_for(cap.trigger(trigger_point=0.0), 2)
                return role, [before, time.monotonic()]
            metadata['trigger_host_intervals_s'] = dict(await asyncio.gather(*(trigger(r, c) for r, c in self.groups.items())))
            await asyncio.gather(*(asyncio.wait_for(c.wait(timeout=3), 4) for c in self.groups.values()))
            # Both board-local buffers are complete. Release the point-settling
            # check before USB downloads so transport contention cannot make a
            # stale pre-download guard fire during the next speed ramp. Raw
            # current, voltage, speed, and fault checks remain active.
            self.pair.monitor.require_steady = None
            # Feed the firmware watchdogs independently of the monitor during
            # the sequential cap.download() calls. USB bandwidth consumed by the
            # buffer transfer can stall the monitor's read_multiple() batches
            # long enough to expire a short watchdog. The feeder fires every
            # _FEED_INTERVAL_S and stops as soon as all downloads finish.
            # Any individual feed failure is silenced: a missed feed is far less
            # harmful than aborting the download mid-transfer.
            hw_call = call
            _FEED_INTERVAL_S = 2.0
            _feeder_stop = asyncio.Event()
            async def _watchdog_feeder():
                while not _feeder_stop.is_set():
                    await asyncio.sleep(_FEED_INTERVAL_S)
                    if _feeder_stop.is_set():
                        break
                    if self.pair.monitor.watchdogs:
                        for b in self.pair.boards.values():
                            try:
                                await hw_call(b, 'axis0.watchdog_feed')
                            except Exception:
                                pass
            feeder_task = asyncio.create_task(_watchdog_feeder())
            download_results = {}
            try:
                for role, cap in self.groups.items():
                    raw = await asyncio.wait_for(
                        cap.download(return_as=dict, t_fmt=TimestampFmt.CONTROL_CYCLE), 45)
                    download_results[role] = raw
            finally:
                _feeder_stop.set()
                await asyncio.gather(feeder_task, return_exceptions=True)
            for role, raw in download_results.items():
                plan = self.plans[role]
                phase_nonfinite = validate_raw(raw, plan['channels'], plan['samples'])
                # Zero out any tolerated non-finite phase samples in-place so that
                # the CSV never contains NaN and the current-limit check below sees
                # only finite values. The per-channel counts go into metadata.
                if any(n > 0 for n in phase_nonfinite.values()):
                    for p, nf in phase_nonfinite.items():
                        if nf:
                            raw[p] = [v if finite(v) else 0.0 for v in raw[p]]
                    metadata.setdefault('phase_nonfinite_counts', {})[role] = {
                        p: n for p, n in phase_nonfinite.items() if n > 0}
                derived = None
                measured = None
                if 'position_derivation' in plan:
                    derived = sensorless_position_turn(raw[plan['channels']['sensorless_phase_rad']],
                                                        plan['position_derivation']['pole_pairs'])
                    measured = measured_position_turn(raw[plan['channels']['measured_encoder_raw_turn']])
                with (folder / f'{role}.csv').open('x', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['cycle', 'time_s', *plan['channels'],
                                     *(['estimated_position_turn', 'measured_position_turn'] if derived is not None else [])])
                    for i, cycle in enumerate(raw['timestamps']):
                        writer.writerow([cycle, cycle/plan['sample_rate_hz'],
                                         *(raw[p][i] for p in plan['channels'].values()),
                                         *([derived[i], measured[i]] if derived is not None else [])])
                # Offline validation of raw phases also catches transients between host polls.
                # Non-finite values have already been replaced with 0.0 above.
                require(all(abs(v) <= MOTOR_HARD_TRIP_A for name, p in plan['channels'].items() if name.startswith('phase_') for v in raw[p]),
                        f'{role}: captured phase current exceeded {MOTOR_HARD_TRIP_A:g} A')
                captured_rpms = [v * 60 for v in raw[REQUIRED['velocity_turn_s']]]
                require(all(abs(v) <= cfg.MAXIMUM_ABSOLUTE_SPEED_RPM for v in captured_rpms),
                        f'{role}: captured overspeed')
                target_rpm = -point.rpm if role == 'test' and point.mode == 'sensorless' else point.rpm
                metadata.setdefault('measured_capture_speed', {})[role] = {
                    'mean_rpm': sum(captured_rpms) / len(captured_rpms),
                    'minimum_rpm': min(captured_rpms), 'maximum_rpm': max(captured_rpms),
                    'within_target_fraction': sum(abs(v - target_rpm) <=
                                                  cfg.LOAD_SPEED_SETTLING_TOLERANCE_RPM
                                                  for v in captured_rpms) / len(captured_rpms)}
            metadata['after'] = self.pair.monitor.latest
            metadata['status'] = 'captured'
            return metadata
        except BaseException as e:
            metadata['status'] = 'failed'
            metadata['error'] = repr(e)
            raise
        finally:
            self.pair.monitor.require_steady = None
            self.pair.monitor.frozen = False
            metadata['finished_unix_s'] = time.time()
            save_json(folder / 'capture.json', metadata)


# ===== ANALYSIS =====
"""Offline current PSD in A²/Hz, never electrical power. No ODrive imports."""
import json
import math
from pathlib import Path
import numpy as np

ANALYSIS = {'method': 'one-sided full-record periodic Hann periodogram, arithmetic mean removed',
            'DC': 'DC bin excluded from all integrated metrics',
            'integration': 'sum of PSD bin densities times fs/N; no trapezoid endpoint halving',
            'fundamental_band': 'f0 ± max(2*df, 0.05*f0), positive bin centers included',
            'leakage': 'Periodic Hann; ±2 bins spans main lobe. Far sidelobes remain out of band; no fitted subtraction.',
            'trust': 'Requires >=10 fundamental cycles and upper band <1.5*f0 and below Nyquist',
            'pole_pairs': 3, 'harmonics': 8}

def psd(values, fs, rpm, pole_pairs=3):
    x = np.asarray(values, dtype=float)
    require(x.ndim == 1 and len(x) >= 16 and np.all(np.isfinite(x)), 'Invalid PSD data')
    require(math.isfinite(fs) and fs > 0 and math.isfinite(rpm), 'Invalid sample rate/RPM')
    n = len(x)
    w = .5 - .5*np.cos(2*np.pi*np.arange(n)/n)
    centered = x - x.mean()
    p = np.abs(np.fft.rfft(centered*w))**2 / (fs * np.sum(w*w))
    p[1:-1 if n % 2 == 0 else None] *= 2
    f = np.fft.rfftfreq(n, 1/fs)
    df = fs/n
    f0 = abs(rpm)/60*pole_pairs
    half = max(2*df, .05*f0)
    positive = f > 0
    band = positive & (f >= f0-half) & (f <= f0+half)
    total = float(np.sum(p[positive])*df)
    inside = float(np.sum(p[band])*df)
    outside = max(0.0, total-inside)
    ratio = (10*math.log10(inside/outside) if inside > 0 and outside > 0 else
             math.inf if inside > 0 else -math.inf if outside > 0 else None)
    trusted = bool(f0/df >= 10 and f0+half < min(1.5*f0, fs/2))
    metrics = {'expected_fundamental_Hz': f0, 'df_Hz': df, 'duration_s': n/fs,
               'fundamental_cycles': f0*n/fs, 'band_Hz': [max(df, f0-half), min(fs/2, f0+half)],
               'total_current_variance_A2': total, 'in_band_variance_A2': inside,
               'out_of_band_variance_A2': outside,
               'in_band_fraction': inside/total if total else None,
               'out_of_band_fraction': outside/total if total else None,
               'in_to_out_ratio_dB': ratio, 'trustworthy_fundamental_comparison': trusted,
               'warning': None if trusted else 'INADEQUATE FREQUENCY RESOLUTION: band ratio is descriptive only',
               'mean_current_A': float(x.mean()), 'time_domain_variance_A2': float(np.var(x)),
               'window_weighted_variance_A2_including_DC_bin': float(np.sum((centered*w)**2)/np.sum(w*w))}
    return f, p, metrics

def load_csv(path):
    a = np.genfromtxt(path, delimiter=',', names=True)
    require(a.ndim == 1 and len(a) >= 16, 'Capture CSV too short')
    require(np.all(np.isfinite(a['time_s'])) and np.all(np.diff(a['cycle']) == 1), 'Bad capture timestamps')
    dt = np.diff(a['time_s'])
    require(np.all(dt > 0) and np.allclose(dt, dt[0], rtol=1e-6, atol=1e-10), 'Nonuniform samples')
    return a, 1/float(np.mean(dt))

def analyze(folder):
    import matplotlib
    matplotlib.use('Agg', force=True)
    matplotlib.rcParams['svg.hashsalt'] = 'odrive-pair-analysis-v1'
    import matplotlib.pyplot as plt
    folder = Path(folder)
    meta = json.loads((folder/'capture.json').read_text())
    require(meta['status'] == 'captured', 'Cannot analyze failed/incomplete capture')
    report = {'parameters': ANALYSIS, 'clock_note': meta['clock_note'],
              'speed_target_rpm': meta.get('load_speed_target_rpm'), 'boards': {}}
    datasets, spectra = {}, {}
    outputs = []
    for role in ('test', 'load'):
        a, fs = load_csv(folder/f'{role}.csv')
        require(math.isclose(fs, meta['boards'][role]['sample_rate_hz'], rel_tol=1e-6), 'Metadata sample-rate mismatch')
        rpm = float(np.mean(a['velocity_turn_s'])*60)
        datasets[role] = a
        report['boards'][role] = {'mean_rpm': rpm, 'mean_iq_A': float(np.mean(a['iq_A'])) if 'iq_A' in a.dtype.names else None,
                                  'phases': {}}
        for phase in ('a', 'b', 'c'):
            f, p, m = psd(a[f'phase_{phase}_A'], fs, rpm)
            report['boards'][role]['phases'][phase] = m
            spectra[role, phase] = (f, p, m)
            path = folder / f'{role}_phase_{phase}_psd.csv'
            np.savetxt(path, np.column_stack((f, p)), delimiter=',', header='frequency_Hz,current_PSD_A2_per_Hz', comments='')
            outputs.append(path)
    def marks(ax, m):
        f0 = m['expected_fundamental_Hz']
        if f0:
            for h in range(1, 9):
                ax.axvline(f0*h, color='orange', alpha=.7 if h == 1 else .25, lw=.8)
        ax.axvspan(*m['band_Hz'], color='green', alpha=.10)
        ax.grid(True, alpha=.2)
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('Current PSD (A²/Hz)')
    for detail in (False, True):
        suffix = 'low_frequency' if detail else 'full_range'
        fig, axes = plt.subplots(3, 2, figsize=(13, 9), squeeze=False)
        for col, role in enumerate(('test', 'load')):
            for row, phase in enumerate(('a', 'b', 'c')):
                f, p, m = spectra[role, phase]
                ax = axes[row, col]
                ax.semilogy(f[1:], np.maximum(p[1:], 1e-30))
                marks(ax, m)
                upper = min(f[-1], max(200, 6*m['expected_fundamental_Hz'])) if detail else f[-1]
                ax.set_xlim(0, upper)
                ax.set_title(f'{role} phase {phase.upper()}' + (' — resolution inadequate' if not m['trustworthy_fundamental_comparison'] else ''))
                single, sax = plt.subplots(figsize=(8, 4))
                sax.semilogy(f[1:], np.maximum(p[1:], 1e-30))
                marks(sax, m)
                sax.set_xlim(0, upper)
                sax.set_title(ax.get_title())
                single.tight_layout()
                path = folder/f'{role}_phase_{phase}_{suffix}.svg'
                single.savefig(path, metadata={'Date': None})
                plt.close(single)
                outputs.append(path)
        fig.suptitle('Independent board clocks; orange: electrical fundamental and harmonics')
        fig.tight_layout()
        path = folder/f'pair_psd_{suffix}.svg'
        fig.savefig(path, metadata={'Date': None})
        plt.close(fig)
        outputs.append(path)
    for roles in (('test',), ('load',), ('test', 'load')):
        fig, axes = plt.subplots(4, len(roles), figsize=(7*len(roles), 10), squeeze=False)
        for col, role in enumerate(roles):
            a = datasets[role]
            t = a['time_s']
            if meta['boards'][role].get('position_derivation'):
                axes[0, col].plot(t, a['estimated_position_turn'], label='sensorless PLL estimate')
                axes[0, col].plot(t, a['measured_position_turn'], label='onboard encoder measurement')
                axes[0, col].legend()
                axes[0, col].set_ylabel('Relative position (turn)')
            else:
                axes[0, col].plot(t, a['position_turn'])
                axes[0, col].set_ylabel('Position (turn)')
            axes[1, col].plot(t, a['velocity_turn_s']*60)
            axes[1, col].set_ylabel('Velocity (RPM)')
            if 'ibus_A' in a.dtype.names:
                axes[2, col].plot(t, a['ibus_A'], lw=.7, label='Ibus')
                axes[2, col].legend()
            else:
                axes[2, col].text(.5, .5, 'Ibus was not captured', ha='center', va='center',
                                  transform=axes[2, col].transAxes)
            axes[2, col].set_ylabel('Ibus (A)')
            if 'vbus_V' in a.dtype.names:
                axes[3, col].plot(t, a['vbus_V'], lw=.7, label='Vbus')
                axes[3, col].legend()
            else:
                axes[3, col].text(.5, .5, 'Vbus was not captured', ha='center', va='center',
                                  transform=axes[3, col].transAxes)
            axes[3, col].set_ylabel('Vbus (V)')
            axes[3, col].set_xlabel('Board-local time from trigger (s)')
            axes[0, col].set_title(role)
        fig.tight_layout()
        path = folder/f'{"_".join(roles)}_timeseries.svg'
        fig.savefig(path, metadata={'Date': None})
        plt.close(fig)
        outputs.append(path)
    save_json(folder/'analysis.json', report)
    outputs.append(folder/'analysis.json')
    return outputs


# ===== RUNNER =====
"""Authorized motion state machine, including sign probe and cold-start validation."""
import asyncio
import json
import math
import threading
import sys
import time
from dataclasses import asdict

async def prompt(text):
    # A daemon input thread cannot hold Python alive after a watchdog/fault cancellation.
    loop = asyncio.get_running_loop()
    future = loop.create_future()
    def deliver(value, error):
        if not future.done():
            if error:
                future.set_exception(error)
            else:
                future.set_result(value)
    def worker():
        try:
            value, error = input(text), None
        except BaseException as e:
            value, error = None, e
        if not loop.is_closed():
            loop.call_soon_threadsafe(deliver, value, error)
    threading.Thread(target=worker, daemon=True).start()
    return await future

def opposing_command(load_rpm, kt, direction, amps):
    require(abs(load_rpm) > .1 and kt > 0 and direction in (-1, 1), 'Cannot determine opposing torque at rest')
    torque = -math.copysign(amps*kt, load_rpm)  # input_torque lives in axis/encoder coordinates
    signed_iq = torque / kt * direction       # FOC coordinate conversion
    return torque, signed_iq

def feedback_bandwidth_path(mode):
    return 'axis0.config.motor.sensorless_pll_bandwidth' if mode == 'sensorless' else 'axis0.config.encoder_bandwidth'

CONTROL_ASSIGNMENT = {'test': 'opposing_torque_after_load_start',
                      'load': 'positive_velocity_control_starts_chain'}

def load_validation_receipt(path):
    require(path.is_file(), 'Sensorless not validated: run validate_sensorless.py --run first')
    receipt = json.loads(path.read_text())
    require(receipt.get('status') == 'validated',
            f'Sensorless validation is {receipt.get("status", "missing")}; '
            'complete validate_sensorless.py --run before a sensorless test')
    require(receipt.get('serials') == SERIALS, 'Sensorless validation receipt has different board serials')
    require(receipt.get('control_assignment') == CONTROL_ASSIGNMENT,
            'Sensorless validation used a different control assignment; repeat validation')
    return receipt

async def receipt_check(pair, path, bandwidth):
    receipt = load_validation_receipt(path)
    require(receipt.get('fingerprint') == await pair.fingerprint(), 'Configuration changed: repeat sensorless validation')
    desired = bandwidth if bandwidth is not None else await read(pair.boards['test'], feedback_bandwidth_path('sensorless'))
    require(receipt.get('bandwidth') == desired, 'Sensorless bandwidth differs from validation; validate selected bandwidth first')
    require(receipt.get('ratio') == 1, 'This rig requires a positive sensored coupling sign')
    require(receipt.get('sensorless_ratio') == -1, 'Sensorless direction evidence missing or changed')
    # External rewiring and mechanics cannot be authenticated by a configuration hash.
    return receipt

class Run:
    def __init__(self, pair, live=True, minimum_duration=.1):
        self.pair = pair
        self.mon = pair.monitor
        self.live_enabled = live
        self.minimum_duration = minimum_duration
        self.ratio = None
        self.iq_target = 0
        self.torque_target = 0
        self.mode = 'sensored'
        self.test_sign = 1
        self.runtime_original = {}

    async def configure(self, point):
        await self.pair.idle_stopped()
        await self.pair.verify()
        self.mode = point.mode
        self.test_sign = SENSORLESS_TEST_SIGN if point.mode == 'sensorless' else 1
        for role, b in self.pair.boards.items():
            kt = await read(b, 'axis0.config.motor.torque_constant')
            tolerance = await read(b, 'axis0.controller.config.vel_limit_tolerance')
            require(math.isfinite(tolerance) and tolerance > 1, f'{role}: invalid velocity limit tolerance')
            values = {'axis0.controller.input_vel': 0.0, 'axis0.controller.input_torque': 0.0,
                      'axis0.config.init_vel': 0.0, 'axis0.config.init_torque': 0.0,
                      'axis0.controller.config.control_mode': (cfg.TORQUE_CONTROL_MODE if role == 'test'
                                                                else cfg.VELOCITY_CONTROL_MODE),
                      'axis0.controller.config.input_mode': (cfg.TORQUE_RAMP_INPUT_MODE if role == 'test'
                                                              else cfg.VELOCITY_RAMP_INPUT_MODE),
                      'axis0.controller.config.torque_ramp_rate': kt * cfg.TEST_TORQUE_RAMP_A_PER_S,
                      'axis0.controller.config.vel_ramp_rate': cfg.LOAD_VELOCITY_RAMP_RPM_PER_S / 60,
                      'axis0.controller.config.vel_limit': (cfg.TEST_VELOCITY_LIMIT_RPM if role == 'test'
                                                             else cfg.LOAD_VELOCITY_LIMIT_RPM) / 60,
                      'axis0.controller.config.vel_limit_tolerance': (cfg.TEST_VELOCITY_LIMIT_TOLERANCE if role == 'test'
                                                                        else cfg.LOAD_VELOCITY_LIMIT_TOLERANCE),
                      'axis0.controller.config.enable_vel_limit': True,
                      'axis0.controller.config.enable_overspeed_error': True,
                      'axis0.controller.config.enable_torque_mode_vel_limit': role == 'test'}
            self.runtime_original[role] = {p: await read(b, p) for p in values if '.config.' in p}
            for p, v in values.items():
                await checked_write(b, p, v)
        if point.bandwidth is not None:
            path = feedback_bandwidth_path(point.mode)
            self.runtime_original['test'][path] = await read(self.pair.boards['test'], path)
            await checked_write(self.pair.boards['test'], path, point.bandwidth)
        self.mon.expected = {'test': 1, 'load': 1}
        self.mon.direction_sign = None
        self.mon.load_speed_target_rpm = None
        self.mon.frozen = False
        self.mon.history.clear()
        # Terminal telemetry replaces the live plot.


    async def arm(self, role, startup=False):
        b = self.pair.boards[role]
        await self.mon.request_closed_loop(role)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if await read(b, 'axis0.current_state') == 8 and await read(b, 'axis0.is_armed'):
                if not startup:
                    self.mon.expected[role] = 8
                return
            await asyncio.sleep(.05)
        raise TimeoutError(f'{role}: failed to enter armed CLOSED_LOOP_CONTROL')

    async def start_load_first(self, target_rpm):
        """Prove positive shaft motion before energizing the test torque axis."""
        self.mon.stage = f'load starts chain in velocity control to {target_rpm:g} RPM'
        self.mon.direction_sign = 1
        start_pos = await read(self.pair.boards['load'], 'axis0.pos_estimate')
        await self.arm('load')
        await self.set_load_velocity(target_rpm)
        deadline = time.monotonic() + cfg.SPEED_SETTLING_TIMEOUT_S
        while time.monotonic() < deadline:
            sample = self.mon.latest.get('load')
            if sample and (sample['state'] == cfg.CLOSED_LOOP_STATE and sample['armed'] and
                           sample['rpm'] > 0 and
                           abs(sample['rpm'] - target_rpm) <= cfg.LOAD_FIRST_SPEED_TOLERANCE_RPM and
                           sample['position'] - start_pos >= cfg.LOAD_FIRST_MINIMUM_TRAVEL_TURN):
                return {'target_rpm': target_rpm, 'load_position_turn': sample['position'],
                        'travel_turn': sample['position'] - start_pos, 'load_rpm': sample['rpm']}
            await asyncio.sleep(.1)
        raise TimeoutError(f'Load did not start positive shaft motion to {target_rpm:g} RPM '
                           'before test torque axis was armed')

    async def wait_load_stable_before_sensorless_arm(self, target_rpm):
        """Use the independent LOAD encoder to prove a sustained high-speed hold."""
        self.mon.stage = f'holding LOAD at {target_rpm:g} RPM before sensorless TEST arm'
        deadline = time.monotonic() + cfg.SPEED_SETTLING_TIMEOUT_S
        while time.monotonic() < deadline:
            history = list(self.mon.history)
            if history:
                end = history[-1][0]
                window = [(t, samples['load']) for t, samples in history
                          if t >= end - cfg.SENSORLESS_ARM_STABLE_WINDOW_S]
                rpms = [sample['rpm'] for _, sample in window]
                if (len(window) >= cfg.SENSORLESS_ARM_MIN_SAMPLES and
                        time.monotonic() - end <= cfg.SENSORLESS_ARM_MAX_SAMPLE_AGE_S and
                        window[-1][0] - window[0][0] >=
                        cfg.SENSORLESS_ARM_STABLE_WINDOW_S * cfg.SENSORLESS_ARM_WINDOW_MIN_FRACTION and
                        all(sample['state'] == cfg.CLOSED_LOOP_STATE and sample['armed']
                            for _, sample in window) and
                        all(abs(rpm - target_rpm) <= cfg.SENSORLESS_ARM_INSTANT_TOLERANCE_RPM
                            for rpm in rpms) and
                        abs(sum(rpms) / len(rpms) - target_rpm) <=
                        cfg.SENSORLESS_ARM_MEAN_TOLERANCE_RPM):
                    return {'target_rpm': target_rpm, 'mean_rpm': sum(rpms) / len(rpms),
                            'sample_count': len(window), 'window_s': window[-1][0] - window[0][0]}
            await asyncio.sleep(cfg.SENSORLESS_ARM_POLL_S)
        raise TimeoutError(f'LOAD did not hold {target_rpm:g} RPM before sensorless TEST arm')

    async def startup(self, receipt):
        self.mon.stage = 'sensorless startup: load starts positive shaft motion'
        require(receipt['ratio'] == 1 and receipt.get('sensorless_ratio', -1) == -1,
                'This rig needs a positive sensored coupling and negative sensorless estimate ratio')
        self.ratio = -1  # test sensorless velocity / independent load velocity
        self.mon.direction_sign = 1  # independent load encoder must move positive
        load_start = await self.start_load_first(cfg.SENSORLESS_LOAD_FIRST_ARM_RPM)
        load_hold = await self.wait_load_stable_before_sensorless_arm(cfg.SENSORLESS_HANDOVER_RPM)
        print(f'LOAD held {cfg.SENSORLESS_HANDOVER_RPM:g} RPM; arming TEST in sensorless mode '
              f'with a {cfg.SENSORLESS_STARTUP_CURRENT_RAMP_S:g} s startup-current ramp.', flush=True)
        self.mon.stage = 'sensorless startup: test torque axis joining moving chain'
        self.mon.sensorless_estimator_guard_active = False
        self.mon.sensorless_arm_grace_remaining = cfg.SENSORLESS_ARM_ESTIMATE_GRACE_SAMPLES
        self.mon.sensorless_arm_grace_started_at = time.monotonic()
        await self.arm('test', startup=True)
        start = time.monotonic()
        direction_since = None
        handover_commanded = False
        last_handover_vel = None
        while time.monotonic() - start < cfg.STARTUP_TIMEOUT_S:
            s = self.mon.latest
            if s:
                require(s['load']['rpm'] > 0, 'Load lost positive rotation during sensorless startup')
                # Wait for the sensorless estimate to agree with the already
                # established high-speed load motion before accepting handover.
                if not handover_commanded:
                    matching_direction = (
                        s['load']['state'] == cfg.CLOSED_LOOP_STATE and s['load']['armed'] and
                        s['test']['state'] == cfg.CLOSED_LOOP_STATE and s['test']['armed'] and
                        s['load']['rpm'] >= cfg.SENSORLESS_LOAD_FIRST_ARM_RPM - cfg.LOAD_FIRST_SPEED_TOLERANCE_RPM and
                        s['test']['rpm'] < 0 and
                        abs(s['test']['rpm'] + s['load']['rpm']) <=
                        cfg.LOAD_SPEED_TRANSIENT_MARGIN_RPM)
                    if matching_direction:
                        direction_since = direction_since or time.monotonic()
                        if time.monotonic() - direction_since >= cfg.STARTUP_DIRECTION_HOLD_S:
                            # The load was already commanded to the handover
                            # target before test-axis arming. Do not rewrite an
                            # identical velocity command while validating the
                            # sensorless estimate.
                            if self.mon.load_speed_target_rpm != cfg.SENSORLESS_HANDOVER_RPM:
                                await self.set_load_velocity(cfg.SENSORLESS_HANDOVER_RPM)
                            handover_commanded = True
                            self.mon.sensorless_estimator_guard_active = True
                            self.mon.stage = (f'sensorless handover: load velocity target '
                                              f'{cfg.SENSORLESS_HANDOVER_RPM:g} RPM')
                    else:
                        direction_since = None
                # Firmware overwrites input_vel at the end of its open-loop ramp.
                handover_vel = await read(self.pair.boards['test'], 'axis0.controller.input_vel')
                last_handover_vel = handover_vel
                means = self.mon.sensorless_snapshot()
                good = (handover_commanded and
                        s['load']['state'] == cfg.CLOSED_LOOP_STATE and s['load']['armed'] and
                        s['test']['state'] == cfg.CLOSED_LOOP_STATE and s['test']['armed'] and
                        self.mon.sensorless_speed_on_target(cfg.SENSORLESS_HANDOVER_RPM) and
                        s['load']['rpm'] > cfg.HANDOVER_INSTANT_MIN_RPM and
                        s['test']['rpm'] < -cfg.HANDOVER_INSTANT_MIN_RPM and
                        means is not None and
                        cfg.HANDOVER_MEAN_MIN_RPM <= means['load']['rpm'] <= cfg.HANDOVER_MEAN_MAX_RPM and
                        -cfg.HANDOVER_MEAN_MAX_RPM <= means['test']['rpm'] <= -cfg.HANDOVER_MEAN_MIN_RPM and
                        cfg.HANDOVER_COUPLING_RATIO_MIN <= -means['test']['rpm']/means['load']['rpm'] <=
                        cfg.HANDOVER_COUPLING_RATIO_MAX and
                        abs(handover_vel + cfg.SENSORLESS_HANDOVER_RPM / 60) <
                        cfg.HANDOVER_INPUT_VELOCITY_TOLERANCE_TURN_S)
                if good:
                    self.mon.expected['test'] = cfg.CLOSED_LOOP_STATE
                    return {'elapsed_s': time.monotonic()-start, 'samples': s, 'input_vel': handover_vel,
                            'load_speed_armed': True, 'load_started_first': load_start,
                            'load_speed_before_test_arm': load_hold,
                            'speed_window': means}
            await asyncio.sleep(.1)
        raise TimeoutError(f'Sensorless handover not verified (last input_vel={last_handover_vel}, '
                           f'load_armed=True, means={self.mon.means()})')

    async def set_load_velocity(self, target_rpm):
        """Command a positive load RPM; ODrive's VEL_RAMP limits the slew rate."""
        require(0 <= target_rpm <= cfg.LOAD_VELOCITY_LIMIT_RPM, 'Invalid load velocity target')
        # During downward ramps the previous target is no longer a useful
        # transient bound. The independent absolute overspeed guard remains.
        previous = self.mon.load_speed_target_rpm
        self.mon.load_speed_target_rpm = (target_rpm if target_rpm > 0 and
                                          (previous is None or target_rpm >= previous)
                                          else None)
        await checked_write(self.pair.boards['load'], 'axis0.controller.input_vel', target_rpm / 60)

    async def unloaded_speed(self, rpm):
        self.mon.stage = 'ramping positive load velocity, test unloaded'
        require(self.ratio in (None, -1, 1), 'Invalid coupling direction')
        if self.ratio is None:
            self.mon.direction_sign = 1
            await self.set_load_velocity(rpm)
            probe_end = time.monotonic()+cfg.COUPLING_PROBE_TIMEOUT_S
            last_observed = None
            while time.monotonic() < probe_end:
                a = self.mon.means()
                if a and a['load']['rpm'] >= cfg.MINIMUM_COUPLING_PROBE_RPM and abs(a['test']['rpm']) >= cfg.MINIMUM_COUPLING_PROBE_RPM:
                    last_observed = a['test']['rpm']/a['load']['rpm']
                    # This low-speed probe establishes direction only. The
                    # strict 1:1 check follows at the settled test speed.
                    if .8 <= abs(last_observed) <= 1.2:
                        self.ratio = 1 if last_observed > 0 else -1
                        break
                await asyncio.sleep(.1)
            else:
                raise TimeoutError(f'Could not establish coupling direction during load velocity ramp '
                                   f'(last velocity ratio {last_observed})')
        require(self.ratio == 1 or self.test_sign == -1,
                'Positive load rotation requires positive sensored coupling')
        load_rpm = rpm  # positive physical direction, including sensorless tests
        await self.set_load_velocity(load_rpm)
        if self.mode == 'sensorless':
            # One fresh pair sample at the requested RPM and unloaded TEST Iq
            # is enough to continue. Sensorless feedback does not remain settled.
            await self.mon.settle(rpm, 0, self.ratio, test_sign=self.test_sign,
                                  timeout=cfg.SPEED_SETTLING_TIMEOUT_S)
            return
        end = time.monotonic()+cfg.SPEED_SETTLING_TIMEOUT_S
        self.mon.history.clear()
        while time.monotonic() < end:
            a = self.mon.means()
            if a and abs(a['load']['rpm'] - load_rpm) <= cfg.LOAD_SPEED_SETTLING_TOLERANCE_RPM:
                observed = a['test']['rpm']/a['load']['rpm']
                require(coupled_speed_matches(a['test']['rpm'], a['load']['rpm'], self.mode == 'sensorless'),
                        f'Chain speeds disagree: test={a["test"]["rpm"]:.1f}, '
                        f'load={a["load"]["rpm"]:.1f} RPM; ratio={observed:.3f}')
                sign = 1 if observed > 0 else -1
                require(self.ratio is None or sign == self.ratio, 'Coupling direction differs from validated direction')
                self.ratio = sign
                self.mon.direction_sign = 1
                await self.mon.settle(rpm, 0, sign, test_sign=self.test_sign)
                return
            await asyncio.sleep(.1)
        raise TimeoutError('Unloaded speed did not settle')

    async def load_probe(self, rpm):
        b = self.pair.boards['test']
        kt = await read(b, 'axis0.config.motor.torque_constant')
        direction = await read(b, 'axis0.config.motor.direction')
        baseline = (self.mon.sensorless_point_reading() if self.mode == 'sensorless'
                    else self.mon.means())
        require(baseline is not None, 'No fresh baseline before opposing-current sign probe')
        torque, iq = opposing_command(baseline['test']['rpm'], kt, direction, cfg.TEST_SIGN_PROBE_CURRENT_A)
        self.mon.stage = f'{cfg.TEST_SIGN_PROBE_CURRENT_A:g} A opposing-load sign verification'
        await write(b, 'axis0.controller.input_torque', torque)
        a = await self.mon.settle(rpm, iq, self.ratio, test_sign=self.test_sign)
        # Verify incremental current polarity, not just a broad tolerance around zero.
        delta = a['test']['iq'] - baseline['test']['iq']
        require(delta*iq > 0 and abs(delta) >= cfg.MINIMUM_SIGN_PROBE_DELTA_A, 'Test current response did not verify commanded polarity')
        require(a['test']['iq']*kt*direction*a['test']['rpm'] < 0,
                'Measured test torque aids motion instead of opposing it')
        load = self.pair.boards['load']
        load_kt = await read(load, 'axis0.config.motor.torque_constant')
        load_direction = await read(load, 'axis0.config.motor.direction')
        require(a['load']['iq']*load_kt*load_direction*a['load']['rpm'] > 0,
                'Measured load torque does not drive positive motion')
        evidence = {'baseline': baseline, 'probe': a, 'delta_iq_A': delta,
                    'motor_direction': direction, 'torque_constant_Nm_A': kt,
                    'probe_torque_Nm': torque, 'probe_iq_A': iq, 'ratio': self.ratio}
        self.pair.meta['events'].append({'test_sign_verified': evidence})
        self.pair.save()
        return kt, direction, evidence

    async def normal_stop(self):
        self.mon.require_steady = None
        self.mon.frozen = False
        self.mon.stage = 'unloading'
        await write(self.pair.boards['test'], 'axis0.controller.input_torque', 0.0)
        end = time.monotonic()+cfg.UNLOAD_TIMEOUT_S
        self.mon.history.clear()
        while time.monotonic() < end:
            a = self.mon.means()
            if a and abs(a['test']['iq']) < .5:
                break
            await asyncio.sleep(.1)
        else:
            raise TimeoutError('Failed to unload')
        self.mon.stage = 'decelerating'
        if self.mode == 'sensorless':
            # Disarm the sensorless torque axis before decelerating through
            # the speed where its estimator can no longer track reliably.
            # Finish any monitor sample that began with the armed expectation
            # before requesting IDLE, so its readback cannot be misclassified.
            await self.mon.request_sensorless_test_idle()
            deadline = time.monotonic()+3
            while time.monotonic() < deadline:
                state = await read(self.pair.boards['test'], 'axis0.current_state')
                armed = await read(self.pair.boards['test'], 'axis0.is_armed')
                if state == 1 and not armed:
                    break
                await asyncio.sleep(.05)
            else:
                raise TimeoutError('Sensorless test axis failed to disarm before load deceleration')
            self.mon.expected['test'] = 1
        # VEL_RAMP brings the commanded load velocity to zero at the named
        # deceleration rate; the positive firmware velocity limit stays put.
        await self.set_load_velocity(0.0)
        end = time.monotonic()+cfg.STOP_TIMEOUT_S
        while time.monotonic() < end:
            roles = ('load',) if self.mode == 'sensorless' else ('load', 'test')
            if self.mon.stationary(roles):
                break
            await asyncio.sleep(.1)
        else:
            raise TimeoutError('Chain did not stop within 120 seconds')
        self.mon.expected = {}
        errors = await emergency_idle(self.pair.boards)
        require(not errors, f'IDLE failed: {errors}')
        self.mon.direction_sign = None
        self.mon.load_speed_target_rpm = None
        self.mon.expected = {'test': 1, 'load': 1}
        # Removing torque can release a little stored chain motion. Confirm a
        # fresh stopped window after disarming before changing feedback modes.
        await self.pair.wait_stopped(timeout=30)

    async def point(self, point, folder, receipt=None, capture_decision=False, do_capture=True,
                    capture_mode=None):
        await self.configure(point)
        capture = Capture(self.pair, self.minimum_duration)
        if do_capture:
            await capture.prepare(mode=capture_mode)  # fresh proxies after any mode reboot
        await self.pair.enable_watchdogs()
        await self.mon.sample()
        result = {}
        async def motion():
            if point.mode == 'sensorless':
                require(receipt is not None, 'Sensorless requires measured startup direction evidence')
                result['handover'] = await self.startup(receipt)
            else:
                result['load_started_first'] = await self.start_load_first(cfg.LOAD_FIRST_SENSORED_ARM_RPM)
                await self.arm('test')
            await self.unloaded_speed(point.rpm)
            kt, direction, result['sign_evidence'] = await self.load_probe(point.rpm)
            torque, iq = opposing_command(self.mon.latest['test']['rpm'], kt, direction, point.amps)
            self.torque_target, self.iq_target = torque, iq
            self.mon.stage = ('waiting for first qualifying sensorless reading' if point.mode == 'sensorless'
                              else 'ramping load and settling')
            await write(self.pair.boards['test'], 'axis0.controller.input_torque', torque)
            result['settled'] = await self.mon.settle(point.rpm, iq, self.ratio,
                                                       test_sign=self.test_sign)
            take = do_capture
            if capture_decision:
                self.mon.stage = 'awaiting capture decision (motors running)'
                take = exact_yes(await asyncio.wait_for(prompt('Begin high-rate capture? Press Enter or type yes; any other answer stops: '), 60))
            if take:
                await capture.acquire(folder, point, self.ratio, iq)
                result['capture'] = str(folder)
            await self.normal_stop()
            return result
        try:
            result = await supervise(motion(), self.mon)
            await self.pair.idle_stopped()
            return result
        finally:
            await self.cleanup(sys.exc_info()[1])

    async def sweep(self, points, group_folder, receipt=None):
        """Capture a continuous speed ladder, descending in sensorless mode."""
        require(points and all(p.mode == points[0].mode for p in points),
                'Sweep group must use one feedback mode')
        require(all(p.amps == points[0].amps and p.bandwidth == points[0].bandwidth for p in points),
                'Sweep group must use one torque target and bandwidth')
        ordered = (all(a.rpm > b.rpm for a, b in zip(points, points[1:]))
                   if points[0].mode == 'sensorless' else
                   all(a.rpm < b.rpm for a, b in zip(points, points[1:])))
        require(ordered, 'Sweep speeds must ascend when sensored and descend when sensorless')
        if points[0].mode == 'sensorless':
            require(receipt is not None, 'Sensorless sweep requires a validated receipt')
        group_folder.mkdir(parents=True, exist_ok=False)
        progress = {'mode': points[0].mode, 'points': [asdict(p) for p in points],
                    'completed': [], 'status': 'preparing'}
        save_json(group_folder/'sweep_progress.json', progress)
        await self.configure(points[0])
        capture = Capture(self.pair, self.minimum_duration)
        await capture.prepare()
        await self.pair.enable_watchdogs()
        await self.mon.sample()
        result = {'mode': points[0].mode, 'captures': [], 'settled': []}
        async def motion():
            if points[0].mode == 'sensorless':
                result['handover'] = await self.startup(receipt)
            else:
                result['load_started_first'] = await self.start_load_first(cfg.LOAD_FIRST_SENSORED_ARM_RPM)
                await self.arm('test')
            await self.unloaded_speed(points[0].rpm)
            kt, direction, result['sign_evidence'] = await self.load_probe(points[0].rpm)
            torque, iq = opposing_command(self.mon.latest['test']['rpm'], kt, direction, points[0].amps)
            self.torque_target, self.iq_target = torque, iq
            self.mon.stage = 'ramping test opposing torque and settling'
            await write(self.pair.boards['test'], 'axis0.controller.input_torque', torque)
            for index, point in enumerate(points):
                global CURRENT_TEST
                CURRENT_TEST = (0 if point.mode == 'sensored' else len(RPMS) + 1) + index + 1
                if index:
                    self.mon.stage = f'ramping load velocity target to {point.rpm:g} RPM'
                    self.mon.history.clear()
                    await self.set_load_velocity(point.rpm)
                capture_wait = ('waiting for first qualifying reading' if point.mode == 'sensorless'
                                else 'settling first')
                print(f'CAPTURE PHASE: {point.mode} {point.rpm:g} RPM target '
                      f'({index + 1}/{len(points)} in this feedback mode); {capture_wait}.', flush=True)
                self.mon.stage = f'{point.mode} {point.rpm:g} RPM: {capture_wait}'
                settled = await self.mon.settle(point.rpm, iq, self.ratio,
                                                test_sign=self.test_sign)
                result['settled'].append({'point': asdict(point), 'means': settled})
                folder = group_folder / f'{int(point.rpm):04d}rpm'
                self.mon.stage = f'{point.mode} {point.rpm:g} RPM: high-rate capture'
                await capture.acquire(folder, point, self.ratio, iq)
                print(f'CAPTURE COMPLETE: {point.mode} {point.rpm:g} RPM -> {folder}', flush=True)
                result['captures'].append(str(folder))
                progress['completed'].append({'rpm': point.rpm, 'folder': str(folder)})
                progress['status'] = 'running'
                save_json(group_folder/'sweep_progress.json', progress)
                self.pair.meta['events'].append({'sweep_capture': str(folder)})
                self.pair.save()
            await self.normal_stop()
            return result
        try:
            result = await supervise(motion(), self.mon)
            await self.pair.idle_stopped()
            progress['status'] = 'captured_and_stopped'
            save_json(group_folder/'sweep_progress.json', progress)
            return result
        except BaseException as e:
            progress['status'] = 'failed'
            progress['error'] = repr(e)
            save_json(group_folder/'sweep_progress.json', progress)
            raise
        finally:
            await self.cleanup(sys.exc_info()[1])

    async def cleanup(self, original_error):
        self.mon.watchdogs = False
        self.mon.expected = {}
        self.mon.direction_sign = None
        self.mon.load_speed_target_rpm = None
        errors = await emergency_idle(self.pair.boards)
        errors += await self.pair.restore_watchdogs_idle()
        self.pair.meta['shutdown_errors'].extend(errors)
        if self.mon.live:
            self.mon.live.close()
            self.mon.live = None
        if original_error is not None:
            # Fault recovery never continues to another point or restores
            # controller settings while the chain may still be coasting.
            self.pair.meta['events'].append({'runtime_restore_skipped_after_fault': repr(original_error)})
            if errors:
                original_error.add_note(f'Shutdown errors: {errors}')
        else:
            require(not errors, f'Shutdown could not be verified: {errors}')
            await self.pair.idle_stopped()
            for role, values in self.runtime_original.items():
                for p, v in values.items():
                    await checked_write(self.pair.boards[role], p, v)

async def preflight_summary(pair, points, plans, validation=False, sweep=False, gui_prepared=False):
    print('\nPREFLIGHT: both boards IDLE/disarmed; no motion until exact yes.')
    print('Serials:', SERIALS)
    gains = f'{cfg.VELOCITY_GAIN:g}/{cfg.VELOCITY_INTEGRATOR_GAIN:g}'
    print(f'Firmware 0.6.12; gains {gains} verified on both boards.' if gui_prepared else
          f'Firmware 0.6.12; gains {gains} persisted and reboot-verified on both boards.')
    print(f'Motor soft limit {CURRENT_LIMIT_A:g} A; per-board DC limits '
          f'{LIMITS["config.dc_max_negative_current"]:+g}/+{LIMITS["config.dc_max_positive_current"]:g} A; '
          f'estimated combined guards -{DC_COMBINED_NEGATIVE_GUARD_A:g}/+{DC_COMBINED_POSITIVE_GUARD_A:g} A.')
    for role, b in pair.boards.items():
        print(f'{role}: motor hard trip {await read(b, "axis0.config.motor.current_hard_max")} A; '
              f'feedback mappings {[await read(b, p) for p in ("axis0.config.load_encoder", "axis0.config.commutation_encoder")]}')
    print('Host voltage 15..32 V; temperature <80 C; speed trip 1890 RPM; 2 s firmware watchdogs during motion.')
    print('Load motor: positive velocity control with VEL_RAMP. Test motor: opposing torque control with TORQUE_RAMP.')
    print(f'Runtime load velocity limit {cfg.LOAD_VELOCITY_LIMIT_RPM:g} RPM, tolerance '
          f'{cfg.LOAD_VELOCITY_LIMIT_TOLERANCE:g}; velocity ramp {cfg.LOAD_VELOCITY_RAMP_RPM_PER_S:g} RPM/s.')
    if sweep:
        print(f'Each mode: one unloaded startup and {cfg.TEST_SIGN_PROBE_CURRENT_A:g} A test-torque sign probe; '
              f'hold {cfg.TEST_OPPOSING_CURRENT_A:g} A opposing torque while load speed targets rise in '
              f'{cfg.SWEEP_STEP_RPM:g} RPM steps without stopping.')
    else:
        print(f'Each point: {cfg.LOAD_VELOCITY_RAMP_RPM_PER_S:g} RPM/s load velocity ramp, measured 1:1 sign, '
              f'{cfg.TEST_SIGN_PROBE_CURRENT_A:g} A opposing-torque probe, '
              f'{cfg.TEST_TORQUE_RAMP_A_PER_S:g} A/s test-torque ramp.')
    print('Test feedback bandwidth: encoder_bandwidth in sensored mode; sensorless_pll_bandwidth in sensorless mode (1/s).')
    print(f'Settling: 3 s means then 2 s hold; load RPM within '
          f'±{cfg.LOAD_SPEED_SETTLING_TOLERANCE_RPM:g} of target, test Iq '
          f'±{cfg.TEST_CURRENT_SETTLING_TOLERANCE_A:g} A; raw guards always active.')
    if gui_prepared:
        print('GUI owns persistent feedback settings and calibration; this script checks them without changing them.')
    else:
        print('Feedback changes are saved/rebooted. Returning to sensored may MOVE THE CHAIN for offset calibration.')
    print(f'Sensorless: {SENSORLESS_STARTUP_A:g} A startup, ramp velocity {SENSORLESS_RAMP_VEL_EREV_S:g} electrical rev/s, '
          f'load motor starts positive rotation to {cfg.SENSORLESS_HANDOVER_RPM:g} RPM before the test motor joins; '
          'the test firmware then performs its open-loop sensorless ramp. No automatic retry or current increase.')
    print('Sensorless stop: unload and disarm test motor, then ramp load velocity target to zero.')
    print('Final standstill: load-encoder position range <0.006 turn and mean velocity within ±10 RPM over 2 s.')
    print('Battery-current sum is asynchronous telemetry, not a battery-side hardware limiter. Physical stop must be accessible.')
    if validation:
        print('VALIDATION WILL attempt one unvalidated sensorless cold start using the completed sensored sweep as direction evidence.'
              if gui_prepared else
              'VALIDATION WILL attempt one unvalidated sensorless cold start after measuring the sensored direction.')
    for point in points:
        print('POINT', asdict(point))
    print('CAPTURE', json.dumps(plans, indent=2))


# ===== CLI =====
import argparse
import asyncio
import itertools
import json
import signal
import sys
import time
from pathlib import Path

# ROOT is the directory containing this standalone file.
DEFAULT_SETTINGS = {'mode': 'sensored', 'rpm': 600, 'load_iq_A': 1, 'feedback_bandwidth': 'keep', 'matrix': {'rpms': [600], 'loads_A': [1], 'modes': ['sensored'], 'feedback_bandwidth': 'keep'}}

def load_defaults():
    """One editable source for the Enter defaults and no-flag matrix values."""
    settings = DEFAULT_SETTINGS
    require(settings['feedback_bandwidth'] in ('keep', '250', '500', '1000'),
            'Invalid feedback_bandwidth in DEFAULT_SETTINGS')
    Point(settings['mode'], float(settings['rpm']), float(settings['load_iq_A']),
          bandwidth(settings['feedback_bandwidth']))
    matrix = settings['matrix']
    require(matrix['feedback_bandwidth'] in ('keep', '250', '500', '1000'),
            'Invalid matrix feedback_bandwidth in DEFAULT_SETTINGS')
    require(matrix['rpms'] and matrix['loads_A'] and matrix['modes'],
            'Matrix defaults must have at least one RPM, load and mode')
    for mode, rpm, amps in itertools.product(matrix['modes'], matrix['rpms'], matrix['loads_A']):
        Point(mode, float(rpm), float(amps), bandwidth(matrix['feedback_bandwidth']))
    return settings

def parser(kind):
    defaults = load_defaults()
    matrix = defaults['matrix']
    p = argparse.ArgumentParser(description=f'ODrive pair {kind}; hardware is accessed only with --run')
    p.add_argument('--run', action='store_true', help='Connect by serial; still requires interactive exact yes before any motion')
    p.add_argument('--output', type=Path, default=ROOT/'results')
    p.add_argument('--no-live', action='store_true', help='Explicitly disable live window for headless operation; console/raw logs remain')
    p.add_argument('--minimum-capture-seconds', type=float, default=DEFAULT_MINIMUM_CAPTURE_SECONDS)
    p.add_argument('--validation', type=Path, default=ROOT/'sensorless_validation.json')
    if kind == 'matrix':
        p.add_argument('--rpms', default=','.join(map(str, matrix['rpms'])),
                       help='Comma-separated RPM values, <=1800; defaults from DEFAULT_SETTINGS')
        p.add_argument('--loads', default=','.join(map(str, matrix['loads_A'])),
                       help=f'Comma-separated Iq magnitudes in A, 0..{CURRENT_LIMIT_A:g}; defaults from DEFAULT_SETTINGS')
        p.add_argument('--modes', default=','.join(matrix['modes']),
                       help='sensored,sensorless or either alone; defaults from DEFAULT_SETTINGS')
        p.add_argument('--bandwidth', choices=('keep','250','500','1000'),
                       default=matrix['feedback_bandwidth'])
        p.add_argument('--resume', action='store_true')
        p.add_argument('--retry-incomplete', action='store_true')
    elif kind in ('validate', 'sweep'):
        p.add_argument('--bandwidth', choices=('keep','250','500','1000'),
                       default=defaults['feedback_bandwidth'])
        p.add_argument('--gui-prepared', action='store_true',
                       help='Verify GUI settings; do not save configuration, switch feedback or calibrate')
        if kind == 'validate':
            p.add_argument('--sensorless-only', action='store_true',
                           help='Use a completed sensored sweep as the direction reference')
            p.add_argument('--reference-session', type=Path,
                           help='Completed sensored sweep session for --sensorless-only')
        if kind == 'sweep':
            p.add_argument('--modes', choices=('sensored', 'sensorless', 'sensored,sensorless'),
                           default='sensored,sensorless',
                           help='Run one feedback group or both; sensorless requires a valid receipt')
            p.add_argument('--start-rpm', type=int, default=600,
                           help='First load velocity target, 600..1600 in 100 RPM steps')
            p.add_argument('--end-rpm', type=int, default=1600,
                           help='Last load velocity target, 600..1600 in 100 RPM steps')
    elif kind == 'commission':
        p.add_argument('--mode', choices=('sensored', 'sensorless'),
                       help='Final test-board feedback mode; omit for the interactive mode prompt')
    return p

def bandwidth(value):
    return None if value == 'keep' else int(value)

def matrix_points(args):
    return [Point(m, float(r), float(a), bandwidth(args.bandwidth))
            for m, r, a in itertools.product(args.modes.split(','), args.rpms.split(','), args.loads.split(','))]

def sweep_points(args):
    require(cfg.FIRST_SWEEP_RPM <= args.start_rpm <= args.end_rpm <= cfg.LAST_SWEEP_RPM and
            args.start_rpm % cfg.SWEEP_STEP_RPM == 0 and args.end_rpm % cfg.SWEEP_STEP_RPM == 0,
            f'Sweep RPM targets must be {cfg.FIRST_SWEEP_RPM}..{cfg.LAST_SWEEP_RPM} '
            f'in ascending {cfg.SWEEP_STEP_RPM} RPM steps')
    speeds = range(args.start_rpm, args.end_rpm + 1, cfg.SWEEP_STEP_RPM)
    return [Point(mode, rpm, cfg.TEST_OPPOSING_CURRENT_A, bandwidth(args.bandwidth))
            for mode in args.modes.split(',')
            for rpm in (reversed(speeds) if mode == 'sensorless' else speeds)]

async def interactive_point():
    defaults = load_defaults()
    mode = (await prompt(f'Test motor feedback [sensored/sensorless] (Enter: {defaults["mode"]}): ')).strip() or defaults['mode']
    rpm = float((await prompt(f'Target RPM (>0, up to 1800; sensorless >={cfg.SENSORLESS_MINIMUM_CAPTURE_RPM:g}) (Enter: {defaults["rpm"]}): ')).strip() or defaults['rpm'])
    amps = float((await prompt(f'Test motor opposing Iq magnitude [0..{CURRENT_LIMIT_A:g} A] (Enter: {defaults["load_iq_A"]}): ')).strip() or defaults['load_iq_A'])
    bw = (await prompt(f'Test feedback bandwidth [keep/250/500/1000 1/s] (Enter: {defaults["feedback_bandwidth"]}): ')).strip() or defaults['feedback_bandwidth']
    require(bw in ('keep','250','500','1000'), 'Invalid bandwidth')
    return Point(mode, rpm, amps, bandwidth(bw))

def stamp():
    return time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + f'_{time.time_ns()%1000000000:09d}'

async def workflow(kind, args):
    require(0 < args.minimum_capture_seconds <= 10, 'Invalid minimum capture duration')
    journal = None
    if kind == 'matrix':
        points = matrix_points(args)
        journal = Journal(args.output, points, args.resume, args.retry_incomplete)
        points = journal.pending(points)
        if not points:
            print('All points already completed and artifacts verified. No hardware connection.')
            return
    elif kind == 'interactive':
        points = [await interactive_point()]
    elif kind == 'validate':
        bw = bandwidth(args.bandwidth)
        if getattr(args, 'sensorless_only', False):
            require(args.gui_prepared and args.reference_session,
                    '--sensorless-only requires --gui-prepared and --reference-session')
            points = [Point('sensorless', cfg.SENSORLESS_HANDOVER_RPM, 0, bw)]
        else:
            points = [Point('sensored', 600, 0, bw),
                      Point('sensorless', cfg.SENSORLESS_HANDOVER_RPM, 0, bw)]
    elif kind == 'sweep':
        points = sweep_points(args)
    else:
        default_mode = load_defaults()['mode']
        mode = getattr(args, 'mode', None)
        if mode is None:
            mode = (await prompt(f'Test motor feedback AFTER commissioning [sensored/sensorless] (Enter: {default_mode}): ')).strip() or default_mode
        require(mode in ('sensored', 'sensorless'), 'Invalid mode')
        points = []
    if kind != 'validate' and any(point.mode == 'sensorless' for point in points):
        # Reject an incomplete receipt before USB connection or persistent saves.
        # The full fingerprint and bandwidth check still follows connection.
        load_validation_receipt(args.validation)
    session = args.output / ('session_' + stamp())
    pair = Pair(session)
    try:
        gui_prepared = bool(getattr(args, 'gui_prepared', False))
        await pair.connect(allow_current_upgrade=not gui_prepared)
        if gui_prepared:
            print('GUI-PREPARED: checking saved gains, limits, calibration and feedback; '
                  'no persistent settings will be changed or calibration run.')
            await pair.verify()
            await pair.snapshot('gui_prepared_before_motion')
            await pair.verify_prepared_mode(points[0].mode)
        else:
            print(f'CONFIGURE: preparing persistent gains {cfg.VELOCITY_GAIN:g}/{cfg.VELOCITY_INTEGRATOR_GAIN:g}, motor soft {CURRENT_LIMIT_A:g} A, '
                  f'DC {INVARIANTS["config.dc_max_negative_current"]:+g}/+{INVARIANTS["config.dc_max_positive_current"]:g} A '
                  'while IDLE; complete backups precede writes.')
            await pair.invariant_setup(raise_hard_trip=True)
        if kind == 'commission':
            print('CONFIGURE complete: both boards saved, rebooted, and verified.')
            print(f'Persistent changes: gains/limits, {cfg.CALIBRATION_CURRENT_A:g} A calibration current, '
                  'onboard mappings, motor/encoder calibration, '
                  f'and test feedback={mode}. Load stays sensored. Both calibration procedures can move the chain.')
            if not exact_yes(await prompt('Ready to calibrate both motors and move the chain? Press Enter or type yes: ')):
                print('Cancelled before calibration; gains and limits remain persisted.')
                pair.meta['status'] = 'cancelled'
                pair.save()
                return
            from odrive.enums import EncoderId
            print('CALIBRATE: applying onboard encoder mappings and calibration currents.')
            for role in SERIALS:
                await pair.change_persistent(role, {
                    **dict.fromkeys(MAPPINGS, EncoderId.ONBOARD_ENCODER0),
                    'axis0.config.motor.calibration_current': cfg.CALIBRATION_CURRENT_A,
                    'axis0.config.calibration_lockin.current': cfg.CALIBRATION_CURRENT_A}, 'commission_setup')
            # Both gains verified again immediately before each calibration.
            for role in SERIALS:
                print(f'CALIBRATE: {role} motor electrical calibration (state 4).')
                await pair.calibration(role, cfg.MOTOR_CALIBRATION_STATE)
                print(f'CALIBRATE: {role} encoder offset calibration (state 7).')
                await pair.calibration(role, cfg.ENCODER_OFFSET_CALIBRATION_STATE)
                await pair.calibrated(role)
            await pair.prepare_mode(mode, authorized=True)
            await pair.snapshot('commissioned')
            pair.meta['status'] = 'complete'
            pair.save()
            print('CALIBRATE complete: both motors and encoder offsets verified.')
            print('Commissioned. Sensorless startup remains unvalidated; validation is the next phase.')
            return
        for role in SERIALS:
            await pair.calibrated(role, encoder=role == 'load')
        # Probe capture support before the operator authorizes motion.
        cap = Capture(pair, args.minimum_capture_seconds)
        plans = ({mode: await cap.prepare(mode=mode) for mode in dict.fromkeys(p.mode for p in points)}
                 if kind == 'sweep' else await cap.prepare())
        if any(p.mode == 'sensorless' for p in points) and kind != 'validate':
            for p in points:
                if p.mode == 'sensorless':
                    await receipt_check(pair, args.validation, p.bandwidth)
                    if kind == 'sweep':
                        break  # the whole sensorless group uses one bandwidth and receipt
        await preflight_summary(pair, points, plans, validation=kind == 'validate',
                                sweep=kind == 'sweep', gui_prepared=gui_prepared)
        if kind == 'sweep':
            print(f'SWEEP: {args.start_rpm}..{args.end_rpm} RPM targets in '
                  f'{cfg.SWEEP_STEP_RPM} RPM steps at {cfg.TEST_OPPOSING_CURRENT_A:g} A '
                  'opposing test load. Selected modes:', args.modes)
            print('Within each mode the chain stays running between captures. A full verified stop is required before changing feedback mode.')
            print('Each point must settle before capture; any fault or failed capture stops the entire sweep. No automatic retries.')
        if not exact_yes(await prompt('Ready to start the test? Press Enter or type yes: ')):
            print('Cancelled before motion. Persisted gains and limits remain in place.')
            pair.meta['status'] = 'cancelled'
            pair.save()
            return
        if kind == 'validate':
            # Invalidate old proof BEFORE any new attempt, so a failed attempt cannot leave stale permission.
            save_json(args.validation, {'status': 'validation_in_progress', 'session': str(session)})
            run = Run(pair, False, args.minimum_capture_seconds)
            if getattr(args, 'sensorless_only', False):
                reference = args.reference_session.resolve()
                metadata = json.loads((reference/'session.json').read_text())
                require(metadata.get('status') == 'complete' and metadata.get('serials') == SERIALS,
                        'Sensored reference session is incomplete or belongs to other boards')
                baseline = json.loads((reference/'sensored'/'motion.json').read_text())
                ratio = baseline['sign_evidence']['ratio']
                require(ratio == 1, 'Sensored reference did not verify positive 1:1 coupling')
                trial_point = points[0]
            else:
                await pair.prepare_mode('sensored', authorized=True)
                baseline = await run.point(points[0], session/'baseline_unused', do_capture=False)
                ratio = baseline['sign_evidence']['ratio']
                await pair.prepare_mode('sensorless', authorized=True)
                trial_point = points[1]
            trial = await run.point(trial_point, session/'validation_capture', receipt={'ratio': ratio})
            await offline_analysis(session/'validation_capture')
            actual_bw = trial_point.bandwidth or await read(pair.boards['test'], feedback_bandwidth_path('sensorless'))
            receipt = {'schema': 2, 'status': 'validated', 'serials': SERIALS,
                       'fingerprint': await pair.fingerprint(), 'bandwidth': actual_bw,
                       'control_assignment': CONTROL_ASSIGNMENT,
                       'ratio': ratio, 'sensorless_ratio': -1,
                       'baseline': baseline, 'trial': trial, 'session': str(session),
                       'created_unix_s': time.time(),
                       'scope': (f'One startup and unloaded {trial_point.rpm:g} RPM point; '
                                 'not a guarantee for all speeds/loads')}
            save_json(args.validation, receipt)
            pair.meta['status'] = 'complete'
            pair.save()
            print(f'Startup direction and handover validated. Receipt: {args.validation}')
            return
        if kind == 'sweep':
            for mode in dict.fromkeys(p.mode for p in points):
                print(f'SWEEP PHASE: {mode.upper()} {args.start_rpm}–{args.end_rpm} RPM '
                      f'at {cfg.TEST_OPPOSING_CURRENT_A:g} A opposing test torque.', flush=True)
                if gui_prepared:
                    await pair.verify_prepared_mode(mode)
                else:
                    await pair.prepare_mode(mode, authorized=True)
                receipt = await receipt_check(pair, args.validation, bandwidth(args.bandwidth)) if mode == 'sensorless' else None
                run = Run(pair, False, args.minimum_capture_seconds)
                group_points = [p for p in points if p.mode == mode]
                group_folder = session/mode
                result = await run.sweep(group_points, group_folder, receipt)
                save_json(group_folder/'motion.json', result)
                for captured in result['captures']:
                    await offline_analysis(Path(captured))
                    print(f'Completed: {captured}')
                progress = json.loads((group_folder/'sweep_progress.json').read_text())
                progress['status'] = 'complete'
                save_json(group_folder/'sweep_progress.json', progress)
            pair.meta['status'] = 'complete'
            pair.save()
            print(f'Sweep complete: {session}')
            return
        for point in points:
            point_folder = args.output / ('point_' + point.key + '_' + stamp())
            if journal:
                journal.begin(point, str(point_folder.relative_to(args.output)))
            try:
                await pair.prepare_mode(point.mode, authorized=True)
                receipt = await receipt_check(pair, args.validation, point.bandwidth) if point.mode == 'sensorless' else None
                run = Run(pair, False, args.minimum_capture_seconds)
                result = await run.point(point, point_folder, receipt, capture_decision=kind == 'interactive')
                pair.meta['events'].append({'point': point.__dict__, 'result': result})
                pair.save()
                if 'capture' in result:
                    # Analysis runs only after verified stop; expensive SVG rendering cannot starve watchdogs.
                    await offline_analysis(point_folder)
                    save_json(point_folder/'motion.json', result)
                    if journal:
                        journal.finish(point, list(point_folder.iterdir()))
                    print(f'Completed: {point_folder}')
            except BaseException as e:
                if journal:
                    journal.fail(point, e)
                raise  # stop the ENTIRE matrix
        pair.meta['status'] = 'complete'
    except BaseException as e:
        pair.meta['status'] = 'failed'
        pair.meta['error'] = repr(e)
        if kind == 'validate' and args.validation.is_file():
            receipt = json.loads(args.validation.read_text())
            if receipt.get('status') == 'validation_in_progress' and receipt.get('session') == str(session):
                save_json(args.validation, {'status': 'failed', 'session': str(session), 'error': repr(e)})
        raise
    finally:
        original_error = sys.exc_info()[1]
        errors = await pair.close()
        if original_error is not None and errors:
            original_error.add_note(f'Unable to confirm shutdown: {errors}')
        elif original_error is None:
            require(not errors, f'Unable to confirm shutdown: {errors}')

async def offline_analysis(folder):
    await asyncio.to_thread(analyze, folder)

async def managed(kind, args):
    task = asyncio.current_task()
    loop = asyncio.get_running_loop()
    # Windows event loops do not implement add_signal_handler; asyncio.run()
    # still handles Ctrl+C there, and the monitor supervises the active run.
    signal_handler_installed = False
    try:
        loop.add_signal_handler(signal.SIGTERM, task.cancel)
        signal_handler_installed = True
    except NotImplementedError:
        pass
    try:
        await workflow(kind, args)
    finally:
        if signal_handler_installed:
            loop.remove_signal_handler(signal.SIGTERM)

def workflow_main(kind):
    global CURRENT_TEST
    if kind == 'validate':
        CURRENT_TEST = len(RPMS) + 1
    args = parser(kind).parse_args()
    if not args.run:
        print('No hardware accessed. Read README.md, then add --run for an operator-supervised session.')
        if kind in ('matrix', 'sweep'):
            for p in (matrix_points(args) if kind == 'matrix' else sweep_points(args)):
                print(p)
        return
    try:
        with (ROOT/'.pair.lock').open('a+b') as lock:
            if os.name == 'nt':
                import msvcrt
                if lock.tell() == 0:
                    lock.write(b'\0')
                    lock.flush()
                lock.seek(0)
                try:
                    msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
                except OSError as exc:
                    raise RuntimeError('Another package workflow owns this ODrive pair') from exc
                try:
                    asyncio.run(managed(kind, args))
                finally:
                    lock.seek(0)
                    msvcrt.locking(lock.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as exc:
                    raise RuntimeError('Another package workflow owns this ODrive pair') from exc
                asyncio.run(managed(kind, args))
    except (Exception, KeyboardInterrupt, asyncio.CancelledError) as e:
        print(f'STOPPED: {e!r}. Inspect session logs and confirm physical standstill.', file=sys.stderr)
        raise SystemExit(1)


# ===== COMPLETE SEQUENCE AND SUMMARY PLOTS =====
"""One-command configuration, sensored/sensorless captures and summary plots.

Offline use: --plot-session PATH reads existing captures without opening USB.
"""
import argparse
import asyncio
import csv
import inspect
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

RPMS = tuple(range(cfg.FIRST_SWEEP_RPM, cfg.LAST_SWEEP_RPM + 1, cfg.SWEEP_STEP_RPM))
SENSORLESS_RPMS = tuple(rpm for rpm in RPMS if rpm >= cfg.SENSORLESS_MINIMUM_CAPTURE_RPM)


def capture_metrics(folder):
    meta = json.loads((folder / 'capture.json').read_text())
    if meta.get('status') != 'captured':
        raise ValueError(f'{folder}: capture is not complete')
    with (folder / 'test.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 16 or not {'velocity_turn_s', 'iq_A'}.issubset(rows[0]):
        raise ValueError(f'{folder}: measured velocity or Iq capture missing')
    rpm = np.array([float(row['velocity_turn_s']) * 60 for row in rows])
    iq = np.array([float(row['iq_A']) for row in rows])
    if not np.isfinite(rpm).all() or not np.isfinite(iq).all():
        raise ValueError(f'{folder}: nonfinite measured RPM or Iq')
    result = {
        'measured_rpm': float(abs(np.mean(rpm))),
        'iq_ripple_rms_A': float(np.std(iq)),
        'iq_mean_A': float(np.mean(iq)),
        'samples': len(rows),
    }
    if meta['point']['mode'] == 'sensorless':
        if not {'estimated_position_turn', 'measured_position_turn'}.issubset(rows[0]):
            raise ValueError(f'{folder}: simultaneous sensorless and encoder position data missing')
        estimate = np.array([float(row['estimated_position_turn']) for row in rows])
        measured = np.array([float(row['measured_position_turn']) for row in rows])
        if not np.isfinite(estimate).all() or not np.isfinite(measured).all():
            raise ValueError(f'{folder}: nonfinite position samples')
        # Relative traces start at zero. The raw encoder and PLL can use opposite
        # coordinate directions; align their measured motion without fitting away drift.
        estimate -= estimate[0]
        measured -= measured[0]
        if abs(measured[-1]) < .1 or abs(estimate[-1]) < .1:
            raise ValueError(f'{folder}: insufficient shaft travel for position comparison')
        sign = 1 if estimate[-1] * measured[-1] > 0 else -1
        error = estimate - sign * measured
        result.update({
            'measured_rpm': float(abs((measured[-1] - measured[0]) * 60 /
                                      (float(rows[-1]['time_s']) - float(rows[0]['time_s'])))),
            'position_error_rms_turn': float(np.sqrt(np.mean(error**2))),
            'position_error_peak_turn': float(np.max(np.abs(error))),
            'encoder_to_estimator_sign': sign,
        })
    return result


def plot_session(session, show=False):
    import matplotlib
    if not show:
        matplotlib.use('Agg', force=True)
    import matplotlib.pyplot as plt

    session = Path(session).resolve()
    if not session.is_dir():
        raise ValueError(f'No session folder: {session}')
    sources_file = session / 'sources.json'
    if sources_file.is_file():
        sources = json.loads(sources_file.read_text())
        mode_roots = {mode: Path(sources[f'{mode}_session']) / mode
                      for mode in ('sensored', 'sensorless')}
    else:
        mode_roots = {mode: session / mode for mode in ('sensored', 'sensorless')}
    points = []
    for mode in ('sensored', 'sensorless'):
        for target in RPMS:
            folder = mode_roots[mode] / f'{target:04d}rpm'
            if not folder.exists():
                continue
            metric = capture_metrics(folder)
            metric.update({'mode': mode, 'speed_target_rpm': target,
                           'folder': str(folder)})
            points.append(metric)
    if not points:
        raise ValueError(f'No complete sweep captures in {session}')
    out = session / 'sweep_plots'
    out.mkdir(exist_ok=True)
    complete = all(any(p['mode'] == mode and p['speed_target_rpm'] == rpm
                       for p in points)
                   for mode, speeds in (('sensored', RPMS), ('sensorless', SENSORLESS_RPMS))
                   for rpm in speeds)
    title_suffix = '' if complete else ' — INCOMPLETE SWEEP'

    fig, ax = plt.subplots(figsize=(9, 5))
    for mode in ('sensored', 'sensorless'):
        group = sorted((p for p in points if p['mode'] == mode), key=lambda p: p['speed_target_rpm'])
        if group:
            ax.plot([p['measured_rpm'] for p in group],
                    [p['iq_ripple_rms_A'] for p in group], 'o-', label=mode)
    ax.set(title='Test motor current ripple vs measured RPM' + title_suffix,
           xlabel='Measured test speed magnitude (RPM)',
           ylabel='Iq ripple RMS about mean (A)')
    ax.grid(alpha=.3)
    ax.legend()
    fig.tight_layout()
    ripple_path = out / 'current_ripple_vs_rpm.png'
    fig.savefig(ripple_path, dpi=170)
    plt.close(fig)

    sensorless = sorted((p for p in points if p['mode'] == 'sensorless'),
                        key=lambda p: p['speed_target_rpm'])
    position_path = None
    if sensorless:
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.plot([p['measured_rpm'] for p in sensorless],
                [p['position_error_rms_turn'] for p in sensorless], 'o-', label='RMS')
        ax.plot([p['measured_rpm'] for p in sensorless],
                [p['position_error_peak_turn'] for p in sensorless], 's--', label='Peak absolute')
        ax.set(title='Sensorless position error vs measured RPM' + title_suffix,
               xlabel='Measured test speed magnitude (RPM)',
               ylabel='Estimator − onboard encoder error (turn)')
        ax.grid(alpha=.3)
        ax.legend()
        fig.tight_layout()
        position_path = out / 'sensorless_position_error_vs_rpm.png'
        fig.savefig(position_path, dpi=170)
        plt.close(fig)
    report = {'complete': complete, 'definition': {
        'current_ripple': 'RMS of test-board measured Iq minus its capture mean; bandwidth is 8 kHz sampling, not PWM ripple',
        'position_error': 'Relative sensorless PLL minus direction-aligned onboard encoder, same test-board buffer; initial offsets removed',
        'x_axis': 'absolute mean of test-board measured velocity during each capture, not commanded target'},
        'points': points, 'plots': {'current_ripple': str(ripple_path),
                                   'sensorless_position_error': str(position_path) if position_path else None}}
    (out / 'metrics.json').write_text(json.dumps(report, indent=2) + '\n')
    print(f'Current ripple plot: {ripple_path}')
    print(f'Sensorless position-error plot: {position_path or "unavailable: no sensorless captures"}')
    if not complete:
        print('INCOMPLETE SWEEP: missing mode/RPM captures; plots contain measured points only.')
    if show:
        for path in (ripple_path, position_path):
            if path:
                fig, ax = plt.subplots(figsize=(12, 7))
                ax.imshow(plt.imread(path))
                ax.axis('off')
        plt.show()
    return report


def phase(number, title):
    global CURRENT_STAGE, CURRENT_TEST
    CURRENT_STAGE, CURRENT_TEST = number, 0
    """Make long hardware operations easy to follow in a plain terminal."""
    print(f'\n{"=" * 72}\nPHASE {number}/{TOTAL_PHASES}: {title}\n{"=" * 72}', flush=True)


async def configure_boards(calibrate=False):
    """Set persistent settings and, when requested, recalibrate before testing.

    Pair.connect checks standstill and all non-current protections. The current
    hard trip may be below its desired value during this preparation phase.
    Calibration is intentionally only requested by a new full sensored run. It
    can move the coupled chain and therefore keeps the workflow's explicit
    exact-``yes`` confirmation. Reusing a completed sensored session only
    refreshes static configuration; recalibration would invalidate its
    reference data.
    """
    cfg.validate()
    folder = ROOT / 'results' / ('configuration_' + stamp())
    pair = Pair(folder)
    error = None
    try:
        await pair.connect(allow_current_upgrade=True)
        from odrive.enums import EncoderId
        # The velocity gains/limits must be persistent before calibration:
        # calibration() verifies the same invariants immediately before it can
        # request a state that moves the shaft.
        await pair.invariant_setup(raise_hard_trip=True)
        if calibrate:
            print('CALIBRATION will electrically calibrate both motors and calibrate both encoder offsets. '
                  'The coupled chain can move.', flush=True)
            if not exact_yes(input('Ready to calibrate both motors and move the chain? Press Enter or type yes: ')):
                raise RuntimeError('Calibration cancelled before any calibration state was requested')
            test_mappings = [await read(pair.boards['test'], path) for path in MAPPINGS]
            if any(m != EncoderId.ONBOARD_ENCODER0 for m in test_mappings):
                print('AUTO CONFIGURE: restoring TEST board feedback to ONBOARD_ENCODER0 for calibration.', flush=True)
                await pair.change_persistent('test', dict.fromkeys(MAPPINGS, EncoderId.ONBOARD_ENCODER0),
                                             'calibration_sensored_mapping')
            for role in ('test', 'load'):
                print(f'CALIBRATE: {role} motor electrical calibration.', flush=True)
                await pair.calibration(role, cfg.MOTOR_CALIBRATION_STATE)
                print(f'CALIBRATE: {role} encoder offset calibration.', flush=True)
                await pair.calibration(role, cfg.ENCODER_OFFSET_CALIBRATION_STATE)
                await pair.calibrated(role)
        changes = await pair.configure_full_sweep()
        pair.meta['status'] = 'complete'
        pair.save()
        print(f'CONFIGURE COMPLETE: {folder}; changed settings: '
              f'{ {role: list(paths) for role, paths in changes.items()} }', flush=True)
        return folder
    except BaseException as exc:
        error = exc
        pair.meta['status'] = 'failed'
        pair.meta['error'] = repr(exc)
        pair.save()
        raise
    finally:
        shutdown_errors = await pair.close()
        if shutdown_errors and error is None:
            pair.meta['status'] = 'failed'
            pair.meta['error'] = f'Configuration shutdown: {shutdown_errors}'
            pair.save()
            raise RuntimeError(f'Configuration shutdown could not be verified: {shutdown_errors}')


def new_session_after(before):
    """Identify the exact session a child workflow just created.

    Checking the session prevents an operator's `no` (which exits normally)
    from accidentally allowing the next, more involved phase to begin.
    """
    created = set((ROOT / 'results').glob('session_*')) - before
    if len(created) != 1:
        raise RuntimeError(f'Expected one new session; found {len(created)}')
    return created.pop()


def run_stage(kind, *arguments):
    """Run a guarded workflow from this file with live terminal output/input.

    Each child keeps its own preflight, exact-yes motion prompt, watchdogs,
    fault handling and verified standstill. The wrapper never invents a `yes`.
    """
    before = set((ROOT / 'results').glob('session_*'))
    subprocess.run([sys.executable, str(Path(__file__).resolve()), '--internal-stage', kind,
                    str(CURRENT_STAGE), '--run', *arguments, '--no-live'], check=True)
    return new_session_after(before)


def require_complete_session(session, label):
    """Treat a cancelled or incomplete child workflow as a stopped pipeline."""
    meta = json.loads((session / 'session.json').read_text())
    if meta.get('status') != 'complete':
        raise RuntimeError(f'{label} did not complete: status={meta.get("status", "missing")}; '
                           f'logs: {session}')


async def auto_switch_to_sensorless():
    """Programmatically switch TEST board feedback mapping to SENSORLESS_ESTIMATOR."""
    from odrive.enums import EncoderId
    folder = ROOT / 'results' / ('auto_switch_sensorless_' + stamp())
    pair = Pair(folder)
    try:
        await pair.connect(allow_current_upgrade=True)
        print('AUTO SWITCH: switching TEST board feedback to SENSORLESS_ESTIMATOR...', flush=True)
        await pair.prepare_mode('sensorless', authorized=True)
        print('AUTO SWITCH: saved and verified SENSORLESS_ESTIMATOR mapping on test board.', flush=True)
    finally:
        await pair.close()


def find_latest_completed_sensored_session():
    """Locate the most recent completed sensored session in results."""
    sessions = sorted((ROOT / 'results').glob('session_*'), key=lambda p: p.stat().st_mtime, reverse=True)
    for s in sessions:
        meta_file = s / 'session.json'
        if meta_file.is_file():
            try:
                meta = json.loads(meta_file.read_text())
                if meta.get('status') == 'complete' and meta.get('mode') == 'sensored':
                    return s
            except Exception:
                continue
    raise RuntimeError('No completed sensored session found to reuse. Run without --skip-sensored first.')


def gui_handoff(auto=True, run_async=asyncio.run):
    """Switch TEST board to sensorless feedback, automatically or via GUI prompt."""
    if auto:
        print('\n--- AUTO SWITCH TO SENSORLESS FEEDBACK ---', flush=True)
        run_async(auto_switch_to_sensorless())
        return

    print('Sensored captures are complete and both boards are stopped. In the GUI, '
          'switch only the TEST board load and commutation feedback to '
          f'SENSORLESS_ESTIMATOR. Phase 1 already saved the sensorless ramp '
          f'({SENSORLESS_RAMP_VEL_EREV_S:g} electrical rev/s, '
          f'{cfg.SENSORLESS_RAMP_ACCEL_EREV_PER_S2:g} electrical rev/s², '
          f'{SENSORLESS_STARTUP_A:g} A). '
          'Save/reboot the mapping, then close the GUI USB connection. Keep the '
          'LOAD board on its onboard encoder.', flush=True)
    resp = input('After the GUI is closed, type ready to continue (or enter for ready): ').strip().lower()
    if resp not in ('ready', 'yes', 'y', ''):
        raise RuntimeError('GUI handoff cancelled; sensorless validation and sweep were not started')


def combined_session(sensored, sensorless):
    """Reference completed capture groups without copying or linking raw data."""
    folder = ROOT / 'results' / ('combined_' + stamp())
    folder.mkdir()
    (folder / 'sources.json').write_text(json.dumps({
        'sensored_session': str(sensored.resolve()),
        'sensorless_session': str(sensorless.resolve())}, indent=2) + '\n')
    return folder


def run_complete_sequence(no_show, handoff=gui_handoff, skip_sensored=False, sensored_session=None,
                          run_async=asyncio.run):
    """Configure, run both feedback groups, and stop after any failed phase."""
    global CURRENT_TEST_TOTAL
    CURRENT_TEST_TOTAL = len(RPMS) + len(SENSORLESS_RPMS) + 1
    results = ROOT / 'results'
    results.mkdir(exist_ok=True)

    # Configuration is always applied before the feedback handoff.  In
    # particular, --skip-sensored may reuse an old capture whose board values
    # predate current source settings.
    phase(1, ('CONFIGURE both boards: backup, save, reboot and verify named settings'
              if skip_sensored else
              'CALIBRATE then CONFIGURE both boards: backup, save, reboot and verify'))
    configured = configure_boards(calibrate=not skip_sensored)
    if inspect.isawaitable(configured):
        run_async(configured)

    if skip_sensored:
        if sensored_session is not None:
            sensored = Path(sensored_session)
        else:
            sensored = find_latest_completed_sensored_session()
        require_complete_session(sensored, 'Reused sensored sweep')
        print(f'\nSKIPPING Phases 1 & 2: Reusing completed sensored session at {sensored}', flush=True)
    else:
        phase(2, f'SENSORED {cfg.FIRST_SWEEP_RPM}–{cfg.LAST_SWEEP_RPM} RPM sweep')
        # The child verifies the new persistent settings and the GUI's existing
        # calibration and sensored feedback before motion.
        sensored_args = ['--gui-prepared', '--modes', 'sensored', '--bandwidth', 'keep',
                        '--start-rpm', str(cfg.FIRST_SWEEP_RPM), '--end-rpm', str(cfg.LAST_SWEEP_RPM)]
        sensored = run_stage('sweep', *sensored_args)
        require_complete_session(sensored, 'Sensored sweep')

    phase(3, 'FEEDBACK SWITCH: switch the test board to sensorless feedback')
    if handoff is gui_handoff:
        handoff(run_async=run_async)
    else:
        handoff()

    phase(4, 'VALIDATE sensorless cold start against the sensored sweep')
    # The earlier sweep supplies measured positive 1:1 coupling evidence.
    # Validation checks the GUI-prepared sensorless mapping and then makes a
    # fresh cold-start attempt without saving or calibrating either board.
    validation_args = ['--gui-prepared', '--sensorless-only',
                       '--reference-session', str(sensored), '--bandwidth', 'keep']
    validated = run_stage('validate', *validation_args)
    require_complete_session(validated, 'Sensorless validation')
    receipt_path = ROOT / 'sensorless_validation.json'
    receipt = json.loads(receipt_path.read_text())
    if receipt.get('status') != 'validated' or Path(receipt.get('session', '')).resolve() != validated.resolve():
        raise RuntimeError(f'Fresh sensorless validation receipt missing; logs: {validated}')

    sensorless_start_rpm = SENSORLESS_RPMS[0]
    phase(5, f'SENSORLESS {cfg.LAST_SWEEP_RPM}→{sensorless_start_rpm} RPM sweep')
    # The sensorless group rechecks the newly issued receipt and its matching
    # hardware fingerprint. It remains one continuous motion between captures.
    sweep_args = ['--gui-prepared', '--modes', 'sensorless',
                  '--start-rpm', str(sensorless_start_rpm), '--end-rpm', str(cfg.LAST_SWEEP_RPM),
                  '--bandwidth', 'keep']
    sensorless = run_stage('sweep', *sweep_args)
    require_complete_session(sensorless, 'Sensorless sweep')

    phase(6, 'PLOT measured current ripple and sensorless position error')
    # The two feedback groups live in separate raw sessions because the GUI
    # must take USB control between them. A source manifest lets the plotter
    # read both groups without changing or duplicating any capture.
    summary = combined_session(sensored, sensorless)
    report = plot_session(summary, show=not no_show)
    if not report['complete']:
        raise RuntimeError(f'Sweeps reported complete but one or more captures are missing: {summary}')
    print(f'ALL PHASES COMPLETE: {summary}', flush=True)


def show_plan(skip_sensored=False):
    """Show the exact default sequence and capture fields without opening USB."""
    global CURRENT_TEST_TOTAL
    CURRENT_TEST_TOTAL = len(RPMS) + len(SENSORLESS_RPMS) + 1
    cfg.validate()
    print(f'PLAN: {CURRENT_TEST_TOTAL} measurement tests in 6 stages')
    print('Stage 1: ' + ('configure both boards; reuse sensored calibration' if skip_sensored
                         else 'calibrate and configure both motors and encoders'))
    print(f'Stage 2: {len(RPMS)} sensored captures at {RPMS[0]}–{RPMS[-1]} RPM '
          f'in {cfg.SWEEP_STEP_RPM} RPM steps' + (' (reused)' if skip_sensored else ''))
    print('Stage 3: switch TEST feedback to sensorless; LOAD stays on onboard encoder')
    print(f'Stage 4: hold LOAD at {cfg.SENSORLESS_HANDOVER_RPM:g} RPM before arming TEST; '
          'then 1 unloaded sensorless validation')
    print(f'Stage 5: {len(SENSORLESS_RPMS)} sensorless captures at '
          f'{SENSORLESS_RPMS[-1]}→{SENSORLESS_RPMS[0]} RPM in '
          f'{cfg.SWEEP_STEP_RPM} RPM steps; opposing torque starts after highest speed is verified')
    print('Stage 6: save current-ripple and position-error summary plots')
    print('Motor modes: LOAD=positive velocity control with velocity ramp, onboard encoder; '
          'TEST=opposing torque control with torque ramp, onboard encoder in Stage 2 and '
          'sensorless estimator in Stages 4–5')
    print(f'Test settings: opposing current {cfg.TEST_OPPOSING_CURRENT_A:g} A, '
          f'load ramp {cfg.LOAD_VELOCITY_RAMP_RPM_PER_S:g} RPM/s, '
          f'test torque ramp {cfg.TEST_TORQUE_RAMP_A_PER_S:g} A/s, '
          f'sensorless startup current {cfg.SENSORLESS_STARTUP_CURRENT_A:g} A '
          f'over {cfg.SENSORLESS_STARTUP_CURRENT_RAMP_S:g} s')
    print(f'Sensorless arm: LOAD held at {cfg.SENSORLESS_HANDOVER_RPM:g} RPM; '
          f'firmware ramp {cfg.SENSORLESS_RAMP_TARGET_EREV_PER_S:g} electrical rev/s; '
          f'host speed-estimate grace up to {cfg.SENSORLESS_ARM_ESTIMATE_GRACE_SAMPLES} armed samples '
          f'or {cfg.SENSORLESS_ARM_ESTIMATE_GRACE_MAX_S:g} s')
    print(f'Sensorless capture: trigger on the first fresh reading within '
          f'±{cfg.LOAD_SPEED_SETTLING_TOLERANCE_RPM:g} RPM on both motors and '
          f'±{cfg.SENSORLESS_CAPTURE_CURRENT_TOLERANCE_A:g} A measured TEST Iq; '
          f'estimator disagreement trips after '
          f'{cfg.SENSORLESS_SPEED_FAULT_CONSECUTIVE_SAMPLES} consecutive samples')
    print(f'Configuration: calibration current {cfg.CALIBRATION_CURRENT_A:g} A, '
          f'soft/hard motor current {cfg.MOTOR_SOFT_CURRENT_LIMIT_A:g}/'
          f'{cfg.MOTOR_HARD_CURRENT_TRIP_A:g} A, DC discharge/charge per board '
          f'{cfg.DC_DISCHARGE_LIMIT_PER_BOARD_A:g}/{cfg.DC_CHARGE_LIMIT_PER_BOARD_A:g} A')
    print(f'Controller gains: velocity {cfg.VELOCITY_GAIN:g}, '
          f'integrator {cfg.VELOCITY_INTEGRATOR_GAIN:g}; bus guard '
          f'{cfg.MINIMUM_BUS_VOLTAGE_V:g}–{cfg.MAXIMUM_BUS_VOLTAGE_V:g} V')
    print(f'High-rate capture: {HIGH_RATE_SAMPLE_HZ} samples/s per motor, '
          f'at least {DEFAULT_MINIMUM_CAPTURE_SECONDS:g} s; mandatory sensored fields: '
          + ', '.join(REQUIRED))
    print('High-rate sensorless TEST fields: ' + ', '.join(SENSORLESS_REQUIRED) +
          '; derived: estimated_position_turn, measured_position_turn')
    print('High-rate optional fields when slots/buffer allow: ' + ', '.join(OPTIONAL))
    print('Live log monitor.jsonl: state, armed, errors, disarm, rpm, position, '
          'Iq, Id, Iq setpoint, Vbus, Ibus, FET temperature and three phase currents '
          'per motor; motor temperature when enabled')
    print('Terminal live display: RPM, Ibus and Vbus for TEST and LOAD; no live plots')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true',
                        help='Configure both boards, run sensored tests, switch feedback, validate sensorless, finish tests and plot')
    parser.add_argument('--plan', action='store_true',
                        help='Print planned tests, modes, settings and capture fields without hardware access')
    parser.add_argument('--skip-sensored', action='store_true',
                        help='Skip sensored test run and reuse the most recent completed sensored session')
    parser.add_argument('--sensored-session', type=Path, default=None,
                        help='Explicit sensored session directory to reuse when --skip-sensored is used')
    parser.add_argument('--prompt-handoff', action='store_true',
                        help='Prompt operator for manual GUI feedback handoff instead of automatically configuring it')
    parser.add_argument('--plot-session', type=Path, help='Only plot an existing session; no hardware access')
    parser.add_argument('--no-show', action='store_true', help='Save plots without opening windows')
    args = parser.parse_args()
    if args.plot_session:
        plot_session(args.plot_session, show=not args.no_show)
        return
    if args.plan:
        show_plan(args.skip_sensored)
        return
    if not args.run:
        print('No hardware accessed. Add --run after GUI calibration and sensored feedback selection.')
        return
    show_plan(args.skip_sensored)
    # ODrive's device manager is bound to the event loop that first opens USB.
    # Retain one loop for configuration and the automatic feedback handoff.
    with asyncio.Runner() as runner:
        handoff_fn = (lambda: gui_handoff(auto=False, run_async=runner.run)) if args.prompt_handoff else gui_handoff
        run_complete_sequence(args.no_show, handoff=handoff_fn,
                              skip_sensored=args.skip_sensored,
                              sensored_session=args.sensored_session,
                              run_async=runner.run)


if __name__ == '__main__':
    try:
        if len(sys.argv) >= 4 and sys.argv[1] == '--internal-stage':
            kind = sys.argv[2]
            CURRENT_STAGE = int(sys.argv[3])
            CURRENT_TEST_TOTAL = len(RPMS) + len(SENSORLESS_RPMS) + 1
            sys.argv[1:4] = []
            workflow_main(kind)
        else:
            main()
    except (ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f'STOPPED: {exc}', file=sys.stderr)
        raise SystemExit(1)
