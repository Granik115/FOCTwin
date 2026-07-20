function config = foctwin_default_config()
%FOCTWIN_DEFAULT_CONFIG Canonical baseline for the supplied R2022b models.

config.schema_version = 1;
config.mode = 'current';
config.stop_time_s = 3;

config.trajectory.step_time = 0;
config.trajectory.angle_rad = 1;
config.trajectory.velocity_rad_s = 0.5;
config.trajectory.iq_a = 0.5;
config.trajectory.mode_calibration = 1;
config.trajectory.mode_error = 3;
config.trajectory.ramp_rising = 200;
config.trajectory.ramp_falling = -200;
config.trajectory.settling_tolerance = 0.05;

config.motor.pole_pairs = 15;
config.motor.rs_ohm = 0.675;
config.motor.ld_h = 0.0013;
config.motor.lq_h = 0.0013;
config.motor.ke_v_per_krpm = 92.6;
config.motor.flux_linkage_wb = 92.6 * 60 / (15 * 2 * pi * 1000);
config.motor.inertia_kg_m2 = 7e-2;
config.motor.viscous_friction_nm_s_rad = 1e-5;
config.motor.bus_voltage_v = 48;

config.friction.breakaway_nm = 0;
config.friction.coulomb_nm = 0;
config.friction.breakaway_velocity_rad_s = 0.01;

config.safety.current_a = 1;
config.safety.voltage_v = 12;
config.safety.velocity_rad_s = 0.7;
config.safety.angle_min_rad = -2*pi;
config.safety.angle_max_rad = 2*pi;

config.controllers.angle = pidConfig(35, 0, 0, 0, 0.7, 0, 0.8);
config.controllers.velocity = pidConfig(20.4, 470, 0, 1000, 12, 0.01, 0.8);
config.controllers.current_q = pidConfig(8.4222, 814, 0, 3000, 12, 0.005, 0.8);
config.controllers.current_d = pidConfig(8.4222, 814, 0, 3000, 12, 0.005, 0.8);
config.sample_time_s = 1e-4;
end

function value = pidConfig(p, i, d, ramp, limit, lpf, kc)
value = struct('p', p, 'i', i, 'd', d, 'output_ramp', ramp, ...
    'output_limit', limit, 'lpf_tf', lpf, 'anti_windup_kc', kc);
end

