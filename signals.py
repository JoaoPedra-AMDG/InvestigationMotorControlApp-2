"""Canonical acquisition channels and units; no generated or default readings."""
UNITS={
 'command_rpm':'rpm','encoder_rpm':'rpm','estimated_rpm':'rpm','drive_rotor_rpm':'rpm','load_rotor_rpm':'rpm',
 'drive_position_turns':'turn','load_position_turns':'turn','encoder_mech_rad':'rad mechanical','estimated_electrical_rad':'rad electrical',
 'ia_a':'A','ib_a':'A','ic_a':'A','drive_id_a':'A','drive_iq_a':'A','id_command_a':'A','iq_command_a':'A',
 'load_command_a':'A','load_iq_a':'A','drive_torque_nm':'Nm','load_torque_nm':'Nm',
 'drive_dc_voltage_v':'V','load_dc_voltage_v':'V','drive_dc_current_a':'A','load_dc_current_a':'A',
 'drive_motor_temp_c':'degC','load_motor_temp_c':'degC','drive_controller_temp_c':'degC','load_controller_temp_c':'degC',
 'saturation':'bool','derating':'bool','clipped':'bool','fault':'code','state':'state','control_stage':'state'}
