"""Reproducible, conservative analysis; missing data stays missing."""
import math
import numpy as np

VERSION = '3.1.0'


def wrap_degrees(x):
    return (np.asarray(x, dtype=float) + 180) % 360 - 180


def timing_report(t, requested_hz=None, sequence=None):
    t = np.asarray(t, dtype=float)
    delta = np.diff(t)
    positive = delta[np.isfinite(delta) & (delta > 0)]
    nominal = 1 / requested_hz if requested_hz and requested_hz > 0 else (float(np.median(positive)) if len(positive) else None)
    result = {'samples': len(t), 'duplicate_timestamps': int(np.sum(delta == 0)),
              'out_of_order': int(np.sum(delta < 0)), 'invalid_timestamps': int(np.sum(~np.isfinite(t))),
              'actual_mean_hz': (len(t)-1)/(t[-1]-t[0]) if len(t)>1 and t[-1]>t[0] else None,
              'median_interval_s': float(np.median(positive)) if len(positive) else None,
              'max_interval_s': float(np.max(positive)) if len(positive) else None,
              'gap_count': int(np.sum(delta > nominal*1.5)) if nominal else 0,
              'estimated_missing_samples': int(np.sum(np.maximum(0, np.rint(positive/nominal)-1))) if nominal else None,
              'sequence_missing': None, 'sequence_duplicates': None, 'sequence_reversed': None, 'sequence_invalid': None}
    if sequence is not None:
        seq = np.asarray(sequence, dtype=float)
        if np.all(np.isfinite(seq)):
            result['sequence_missing'] = int(np.sum(np.maximum(0, np.diff(seq)-1)))
            result['sequence_duplicates'] = int(np.sum(np.diff(seq)==0))
            result['sequence_reversed'] = int(np.sum(np.diff(seq)<0))
            result['sequence_invalid'] = int(np.sum(seq!=np.floor(seq)))
        elif np.any(np.isfinite(seq)):
            result['sequence_invalid'] = int(np.sum(~np.isfinite(seq)))
    return result


def stats(x):
    x = np.asarray(x, dtype=float)
    return {'mean': float(np.mean(x)), 'rms': float(np.sqrt(np.mean(x*x))),
            'min': float(np.min(x)), 'max': float(np.max(x)),
            'peak_to_peak': float(np.ptp(x)), 'abs_peak': float(np.max(np.abs(x)))}


def analyze(rows, metadata, settings=None):
    settings = dict(settings or {})
    flags, metrics, derived = [], {}, {}
    def flag(code, message, severity='warning'):
        if not any(f['code'] == code for f in flags):
            flags.append(dict(code=code, message=message, severity=severity))
    start, end = settings.get('start_s'), settings.get('end_s')
    if start is not None and (not math.isfinite(float(start))):
        raise ValueError('Window start must be finite.')
    if end is not None and (not math.isfinite(float(end))):
        raise ValueError('Window end must be finite.')
    if start is not None and end is not None and float(end) <= float(start):
        raise ValueError('Window end must follow start.')
    def timestamp(row):
        try:return float(row.get('time_s'))
        except (ValueError,TypeError):return float('nan')
    # Retain invalid clock evidence: it cannot safely be assigned to either side of a window.
    window = [dict(r,time_s=timestamp(r)) for r in rows if not math.isfinite(timestamp(r)) or ((start is None or timestamp(r) >= float(start)) and (end is None or timestamp(r) <= float(end)))]
    t = np.asarray([r['time_s'] for r in window], dtype=float)
    acquisition = metadata.get('acquisition', {})
    timing = timing_report(t, acquisition.get('requested_hz'), [r.get('sample', np.nan) for r in window])
    result = {'analysis_version': VERSION, 'settings': settings, 'source': metadata.get('source'),
              'metrics': metrics, 'quality_flags': flags, 'timing': timing,
              'window_samples': len(window), 'valid_acquisition': False,
              'pwm_ripple_supported': False, 'derived': derived}
    if len(window) < 3:
        flag('too_short', 'At least three samples are required.', 'error')
        return result
    def col(name):
        try:
            a = np.asarray([r.get(name, np.nan) if r.get(name) is not None else np.nan for r in window], dtype=float)
            return a if np.all(np.isfinite(a)) else None
        except (ValueError, TypeError):
            return None
    invalid_time = timing['duplicate_timestamps'] or timing['out_of_order'] or timing['invalid_timestamps']
    if invalid_time:
        flag('timestamps', 'Duplicate, invalid or reversed timestamps; timing-sensitive metrics withheld.', 'error')
    if timing.get('sequence_duplicates') or timing.get('sequence_reversed') or timing.get('sequence_invalid'):
        flag('sample_sequence', 'Duplicate, reversed or invalid sample sequence; timing-sensitive metrics withheld.', 'error')
    if timing['gap_count'] or timing.get('sequence_missing'):
        flag('timing_gaps', 'Missing samples or timing gaps; no interpolation performed.', 'error')
    if invalid_time:return result
    if metadata.get('capture_interrupted'):
        flag('capture_interrupted', 'Capture was interrupted or its paired acquisition failed.', 'error')
    if acquisition.get('partial'):
        flag('partial_capture', 'Capture ended before the requested post-trigger window completed.', 'error')
    if acquisition.get('actual_pre_s', 1) + .001 < acquisition.get('requested_pre_s', 0):
        flag('pretrigger_short', 'Insufficient pre-trigger history retained.', 'error')
    if metadata.get('source') == 'SIMULATION':
        flag('simulation', 'Synthetic data: not experimental evidence.', 'info')
    for field in ('saturation', 'derating', 'clipped'):
        if any(bool(r.get(field)) for r in window):
            flag(field, 'Acquisition clipping detected.' if field == 'clipped' else f'{field.title()} active in this window.', 'error' if field == 'clipped' else 'warning')
    ranges = acquisition.get('ranges', {})
    for field, limits in ranges.items():
        a = col(field)
        if a is not None and (np.any(a <= limits[0]) or np.any(a >= limits[1])):
            flag('clipped', f'{field} reaches the declared input range.', 'error')
    if any(r.get('fault') not in (None, '', 0, '0', 'NONE') for r in window):
        flag('control_fault', 'A control fault occurred. This is distinct from acquisition failure.')
    kind = settings.get('window_type', metadata.get('plan', {}).get('test_type', 'steady state'))
    steady = kind == 'steady state'
    load_measured,load_command=col('load_iq_a'),col('load_command_a')
    if load_measured is not None and load_command is not None:
        load_direction=metadata.get('profile',{}).get('load_direction',-1)
        load_error=load_measured-load_command*load_direction
        metrics['load_current_tracking_error_a']=stats(load_error)
        if steady and (np.max(np.abs(load_error)) > metadata.get('plan',{}).get('settle_load_a',.2) or np.ptp(load_command)>.001):
            flag('unsettled','Load is unsettled or its command changes in this steady-state window.')
    if steady and any(r.get('control_stage') not in (None,'closed_loop') for r in window):
        flag('unsettled','The steady-state window contains idle, startup, handover, stopping or fault operation.')
    rpm, cmd = col('encoder_rpm'), col('command_rpm')
    if rpm is not None and cmd is not None:
        e = rpm-cmd
        metrics['speed_error_rpm'] = stats(e)
        metrics['speed_variation_rpm'] = float(np.std(rpm))
        tolerance = metadata.get('plan', {}).get('settle_rpm', 10)
        if steady and (np.max(np.abs(e)) > tolerance or np.ptp(cmd) > tolerance):
            flag('unsettled', 'Speed is outside the settling tolerance or the command changes in the selected steady-state window.')
        if not steady and not invalid_time:
            changes = np.flatnonzero(np.abs(np.diff(cmd)) > 1e-6)+1
            if len(changes):
                i = int(changes[-1]); final = float(cmd[-1]); initial = float(cmd[i-1])
                sign = 1 if final >= initial else -1
                metrics['last_speed_step'] = {'time_s': float(t[i]), 'overshoot_rpm': float(max(0, np.max(sign*(rpm[i:]-final))))}
                hold = metadata.get('plan', {}).get('settle_s', 1)
                settled_at = None
                for j in range(i, len(t)):
                    if t[-1]-t[j] >= hold and np.all(np.abs(e[j:]) <= tolerance):
                        settled_at = float(t[j]-t[i]); break
                metrics['last_speed_step']['settling_time_s'] = settled_at
    for field in ['drive_dc_voltage_v','load_dc_voltage_v','drive_dc_current_a','load_dc_current_a',
                  'drive_motor_temp_c','load_motor_temp_c','drive_controller_temp_c','load_controller_temp_c',
                  'ia_a','ib_a','ic_a','drive_iq_a','drive_id_a']:
        a = col(field)
        if a is not None:
            metrics[field] = stats(a)
    for axis in ('id','iq'):
        measured, command = col('drive_'+axis+'_a'), col(axis+'_command_a')
        if measured is not None and command is not None:
            metrics[axis+'_tracking_error_a'] = stats(measured-command)

    calibration = metadata.get('calibration', {})
    sync = acquisition.get('synchronization', {})
    encoder = col('encoder_mech_rad')
    estimate = col('estimated_electrical_rad')
    theta = None
    def finite_number(value):return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)
    valid_cal = calibration.get('verified') is True and isinstance(calibration.get('pole_pairs'), int) and not isinstance(calibration.get('pole_pairs'),bool) and calibration['pole_pairs'] > 0 and calibration.get('id') and calibration.get('encoder_direction') in (-1,1) and finite_number(calibration.get('offset_rad'))
    valid_sync = sync.get('simultaneous') is True and finite_number(sync.get('uncertainty_s')) and sync['uncertainty_s']>=0 and sync.get('method') and not any(f['severity']=='error' for f in flags)
    budget=settings.get('max_timing_angle_deg',2)
    if not finite_number(budget) or budget<0:raise ValueError('Timing angle budget must be finite and nonnegative.')
    if valid_cal and encoder is not None:
        theta = encoder*calibration['pole_pairs']*calibration['encoder_direction'] + calibration['offset_rad']
    else:
        flag('angle_reference', 'Independent encoder angle and a fixed verified pole-pair/offset calibration are required.')
    if not valid_sync:
        flag('synchronization', 'Simultaneous channels and quantified timing uncertainty are unverified; angle and reference-frame metrics withheld.')
    if theta is not None and valid_sync:
        # Bound electrical-angle uncertainty using measured shaft speed where available.
        angular_speed = np.max(np.abs(rpm))*2*np.pi/60*calibration['pole_pairs'] if rpm is not None else np.max(np.abs(np.diff(np.unwrap(theta))/np.diff(t)))
        timing_deg = float(angular_speed*sync['uncertainty_s']*180/np.pi)
        metrics['timing_angle_uncertainty_deg'] = timing_deg
        if timing_deg > settings.get('max_timing_angle_deg', 2):
            valid_sync = False
            flag('angle_timing_uncertainty', 'Timing uncertainty exceeds the selected angle-error budget; angle/reference-frame metrics withheld.')
    if theta is not None and valid_sync and estimate is not None:
        errors = wrap_degrees(np.rad2deg(estimate-theta))
        circular_bias = math.degrees(math.atan2(float(np.mean(np.sin(np.deg2rad(errors)))), float(np.mean(np.cos(np.deg2rad(errors))))))
        metrics['angle_error_deg'] = {'bias': circular_bias, 'rms': float(np.sqrt(np.mean(errors**2))),
                                      'p95_absolute': float(np.percentile(np.abs(errors),95)), 'max_absolute': float(np.max(np.abs(errors)))}
        derived['angle_error_deg'] = errors.tolist()
        crossings = int(np.sum(np.abs(np.diff(errors)) > 180))
        metrics['error_wrap_crossings'] = crossings
        if crossings:
            flag('wrap_crossings', f'{crossings} wrapped-error boundary crossings; inspect for loss of lock.')
        if np.any(np.abs(errors)>90):
            flag('possible_loss_of_lock', 'Absolute angle error exceeds 90 electrical degrees; inspect observer state.')
    elif estimate is None:
        flag('missing_estimated_angle', 'Estimated electrical angle unavailable; no angle error calculated.')

    phases = [col(x) for x in ('ia_a','ib_a','ic_a')]
    phase_available = all(x is not None for x in phases)
    bandwidth = acquisition.get('bandwidth_hz')
    fs = timing['actual_mean_hz']
    if not bandwidth or not acquisition.get('filtering'):
        flag('bandwidth_unknown', 'Measurement bandwidth/filtering unspecified; current-ripple metrics withheld.')
    if phase_available and theta is not None and valid_sync:
        ia, ib, ic = phases
        alpha = (2/3)*(ia-.5*ib-.5*ic)
        beta = (2/3)*(np.sqrt(3)/2)*(ib-ic)
        derived['id_reference_a'] = (alpha*np.cos(theta)+beta*np.sin(theta)).tolist()
        derived['iq_reference_a'] = (-alpha*np.sin(theta)+beta*np.cos(theta)).tolist()
        metrics['id_reference_a'] = stats(derived['id_reference_a'])
        metrics['iq_reference_a'] = stats(derived['iq_reference_a'])
    if phase_available and steady and not any(f['code']=='unsettled' or f['severity']=='error' for f in flags) and bandwidth and acquisition.get('filtering'):
        # Require the independent angle or an explicitly declared constant fundamental.
        phi = theta if theta is not None and valid_sync else None
        if phi is None and settings.get('fundamental_hz'):
            f = float(settings['fundamental_hz'])
            if not math.isfinite(f) or f <= 0:
                raise ValueError('Fundamental frequency must be positive and finite.')
            phi = 2*np.pi*f*t
        if phi is not None:
            turns = float(abs(np.unwrap(phi)[-1]-np.unwrap(phi)[0])/(2*np.pi))
            fundamental = turns/(t[-1]-t[0])
            if turns < 3 or fs < 10*fundamental or bandwidth < fundamental or bandwidth >= fs/2:
                flag('insufficient_bandwidth', 'Need at least 3 cycles, 10 samples/cycle and a documented bandwidth below Nyquist.')
            else:
                basis = np.column_stack((np.ones(len(phi)),np.sin(phi),np.cos(phi)))
                if np.linalg.matrix_rank(basis) == 3:
                    for name,a in zip(('ia','ib','ic'),phases):
                        coef = np.linalg.lstsq(basis,a,rcond=None)[0]
                        residual = a-basis@coef
                        metrics[name+'_residual_a'] = {'rms':float(np.sqrt(np.mean(residual**2))), 'peak_to_peak':float(np.ptp(residual)), 'abs_peak':float(np.max(np.abs(residual))),
                            'bandwidth_hz':bandwidth, 'method':'Least-squares DC + sin/cos fundamental removed; no per-run angle bias fit.'}
                        derived[name+'_residual_a']=residual.tolist()
                    metrics['maximum_current_ripple_a']={'peak_to_peak':max(metrics[n+'_residual_a']['peak_to_peak'] for n in ('ia','ib','ic')),
                        'abs_peak':max(metrics[n+'_residual_a']['abs_peak'] for n in ('ia','ib','ic')),
                        'definition':'Largest phase residual after DC + fitted fundamental removal; within declared acquisition bandwidth.'}
        else:
            flag('fundamental_unknown','Provide synchronized calibrated encoder angle or a justified constant fundamental frequency.')
    elif not phase_available:
        flag('missing_phase_currents', 'Three phase-current channels unavailable; phase residual and common-reference Id/Iq withheld.')
    switching = acquisition.get('switching_hz')
    pwm_ok = bool(not any(f['severity']=='error' for f in flags) and switching and bandwidth and fs and fs >= 10*switching and bandwidth >= 3*switching and bandwidth < fs/2 and acquisition.get('anti_alias_verified') and acquisition.get('source') == 'EXTERNAL_DAQ')
    result['pwm_ripple_supported'] = pwm_ok
    flag('pwm_scope', 'PWM bandwidth prerequisites declared; residual metrics still include non-switching content and are not an isolated PWM metric.' if pwm_ok else 'PWM switching ripple is not resolved/verified by this acquisition.', 'info' if pwm_ok else 'warning')
    result['valid_acquisition'] = not any(f['severity']=='error' for f in flags)
    # Derived time series share only this dataset's original clock. No cross-device alignment.
    derived['time_s'] = t.tolist()
    return result
