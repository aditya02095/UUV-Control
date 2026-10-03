clear;

wb_robot_init();

TIME_STEP = 10;
dt = TIME_STEP / 1000.0;

gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);

transmitter = wb_robot_get_device('emitter');

imu = wb_robot_get_device('imu');
wb_inertial_unit_enable(imu, TIME_STEP);

motors = [
    wb_robot_get_device('motor_1')
    wb_robot_get_device('motor_2')
    wb_robot_get_device('motor_3')
    wb_robot_get_device('motor_4')
];

for i = 1:4
    wb_motor_set_position(motors(i), inf);
    wb_motor_set_velocity(motors(i), 0.0);
end

KP = 3.0;
KI = 0.5;
KD = 0.2;

TARGET_SPEED = 3.2;

speed_integral = 0.0;
prev_err = 0.0;
prev_pos = [];


while wb_robot_step(TIME_STEP) ~= -1

    pos = wb_gps_get_values(gps);
    rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    wb_emitter_send(transmitter, pos);

    if ~isempty(prev_pos)

        vx = (pos(1) - prev_pos(1)) / dt;
        vz = (pos(3) - prev_pos(3)) / dt;

        yaw = rpy(3);

        actual_speed = ...
            vx * sin(yaw) + ...
            vz * cos(yaw);

    else

        actual_speed = 0.0;

    end

    prev_pos = pos;


    err = TARGET_SPEED - actual_speed;

    speed_integral = speed_integral + err * dt;

    speed_integral = max(-20, min(20, speed_integral));

    derivative = (err - prev_err) / dt;

    prev_err = err;

    thrust = ...
        KP * err + ...
        KI * speed_integral + ...
        KD * derivative;

    thrust = max(-20, min(20, thrust));

    wb_motor_set_velocity(motors(1), thrust);
    wb_motor_set_velocity(motors(2), thrust);

    wb_motor_set_velocity(motors(3), 0.0);
    wb_motor_set_velocity(motors(4), 0.0);

end


for i = 1:4
    wb_motor_set_velocity(motors(i), 0.0);
end

wb_robot_cleanup();