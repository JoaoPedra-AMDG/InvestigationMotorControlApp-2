"""Allowlisted configuration changes for the two-motor workbench."""
import math


def number(value,name,low,high):
    if isinstance(value,bool):raise ValueError(name+' must be a number.')
    try:value=float(value)
    except (ValueError,TypeError):raise ValueError(name+' must be a number.') from None
    if not math.isfinite(value) or not low<=value<=high:raise ValueError(f'{name} must be between {low:g} and {high:g}.')
    return value


def proposed_settings(values,polling_hz):
    expected={'test_watchdog_s','load_watchdog_s','test_ramp_rpm_s','load_ramp_nm_s'}
    if set(values)!=expected:raise ValueError('Provide both watchdog timeouts and both validated ramp rates.')
    minimum=max(.5,6/polling_hz)
    if minimum>2:raise ValueError('Set board polling to at least 3 Hz in Connections before configuring watchdogs.')
    result={}
    for role in ('test','load'):
        wd=number(values[role+'_watchdog_s'],role.title()+' watchdog timeout',minimum,2)
        ramp=number(values['test_ramp_rpm_s' if role=='test' else 'load_ramp_nm_s'],role.title()+' ramp rate',.000001,100000)
        result[role]={
            'axis0.controller.input_vel':0.,'axis0.controller.input_torque':0.,
            'axis0.config.init_vel':0.,'axis0.config.init_torque':0.,
            'axis0.config.watchdog_timeout':wd,'axis0.config.enable_watchdog':True,
            'axis0.controller.config.'+('vel_ramp_rate' if role=='test' else 'torque_ramp_rate'):ramp/60 if role=='test' else ramp,
            'axis0.controller.config.input_mode':2 if role=='test' else 6,
            'axis0.controller.config.control_mode':2 if role=='test' else 1}
    return result


def same(a,b):
    if isinstance(a,bool) or isinstance(b,bool):return type(a)==type(b) and a==b
    return isinstance(a,(int,float)) and math.isfinite(a) and math.isclose(a,b,rel_tol=1e-6,abs_tol=1e-8)
