function variables = foctwin_model_variables(config)
%FOCTWIN_MODEL_VARIABLES Convert the stable nested API to legacy model variables.

variables.step_time = config.trajectory.step_time;
variables.step_final_angle = config.trajectory.angle_rad;
variables.step_final_speed = config.trajectory.velocity_rad_s;
variables.step_final_Iq = config.trajectory.iq_a;
variables.mode_calibration = config.trajectory.mode_calibration;
variables.mode_error = config.trajectory.mode_error;
variables.ramp_rising = config.trajectory.ramp_rising;
variables.ramp_falling = config.trajectory.ramp_falling;
variables.settling_tolerance = config.trajectory.settling_tolerance;

variables.p = config.motor.pole_pairs;
variables.Rs = config.motor.rs_ohm;
variables.Ld = config.motor.ld_h;
variables.Lq = config.motor.lq_h;
variables.Ke = config.motor.ke_v_per_krpm;
variables.lambda_pm = config.motor.flux_linkage_wb;
variables.J = config.motor.inertia_kg_m2;
variables.B = config.motor.viscous_friction_nm_s_rad;
variables.V_bus = config.motor.bus_voltage_v;

variables.angle_P = config.controllers.angle.p;
variables.angle_I = config.controllers.angle.i;
variables.angle_D = config.controllers.angle.d;
variables.PID_angle_limit = min(config.controllers.angle.output_limit, config.safety.velocity_rad_s);
variables.LPF_pos = config.controllers.angle.lpf_tf;
variables.kc = config.controllers.angle.anti_windup_kc;

variables.PID_vel_P = config.controllers.velocity.p;
variables.PID_vel_I = config.controllers.velocity.i;
variables.PID_vel_D = config.controllers.velocity.d;
variables.PID_vel_output_ramp = config.controllers.velocity.output_ramp;
variables.PID_vel_limit = min(config.controllers.velocity.output_limit, config.safety.voltage_v);
variables.LPF_speed = config.controllers.velocity.lpf_tf;

% The supplied current model currently shares q/d gains. Separate q/d fields
% remain in the public API so a later model migration does not break projects.
variables.PID_curr_P = config.controllers.current_q.p;
variables.PID_curr_I = config.controllers.current_q.i;
variables.PID_curr_D = config.controllers.current_q.d;
variables.PID_curr_output_ramp = config.controllers.current_q.output_ramp;
variables.PID_curr_limit = min(config.controllers.current_q.output_limit, config.safety.voltage_v);
variables.LPF_I = config.controllers.current_q.lpf_tf;
variables.LPF_q = config.controllers.current_q.lpf_tf;
variables.LPF_d = config.controllers.current_d.lpf_tf;

variables.current_limit = config.safety.current_a;
variables.voltage_limit = config.safety.voltage_v;
variables.Ts_control = config.sample_time_s;
variables.PID_vel_Ts = config.sample_time_s;
variables.PID_curr_q_Ts = config.sample_time_s;
variables.PID_curr_d_Ts = config.sample_time_s;
end

