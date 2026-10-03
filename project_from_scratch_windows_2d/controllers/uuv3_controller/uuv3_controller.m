clear;

wb_robot_init();

TIME_STEP = 10;
dt = TIME_STEP / 1000.0;

gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);

imu = wb_robot_get_device('imu');
wb_inertial_unit_enable(imu, TIME_STEP);

receiver = wb_robot_get_device('receiver');
wb_receiver_enable(receiver, TIME_STEP);

DESIRED_OFFSET = [-10.0, 0.0, 10.0];

KP = 0.6;
KI = 0.18;
KD = 0.10;

INTEGRAL_LIMIT = 20.0;


integral_error = [0.0, 0.0, 0.0];

previous_error = [0.0, 0.0, 0.0];

derivative_initialized = false;

MIN_CONTROL = -20.0;
MAX_CONTROL = 20.0;

uuv1_position = [];

motors = [
    wb_robot_get_device('motor_1')
    wb_robot_get_device('motor_2')
    wb_robot_get_device('motor_3')
    wb_robot_get_device('motor_4')
    wb_robot_get_device('motor_5')
    wb_robot_get_device('motor_6')
];


for i = 1:6
    wb_motor_set_position(motors(i), inf);
    wb_motor_set_velocity(motors(i), 0.0);
end

fprintf('\n');
fprintf('========================================\n');
fprintf('UUV3 3D FORMATION CONTROLLER\n');
fprintf('========================================\n');

fprintf('Desired offset [N U E] = [%.2f %.2f %.2f]\n', ...
    DESIRED_OFFSET(1), ...
    DESIRED_OFFSET(2), ...
    DESIRED_OFFSET(3));

fprintf('========================================\n');

while wb_robot_step(TIME_STEP) ~= -1

    own_position = wb_gps_get_values(gps);
    own_position = reshape(own_position, 1, 3);

    if wb_receiver_get_queue_length(receiver) > 0
        while wb_receiver_get_queue_length(receiver) > 0
            data = wb_receiver_get_data(receiver);
            setdatatype(data, 'doublePtr', 1, 3);
            uuv1_position = data.Value;
            wb_receiver_next_packet(receiver);
        end
    end

    if isempty(uuv1_position)
        continue;
    end

    uuv1_position = reshape(uuv1_position, 1, 3);
    relative_position = own_position - uuv1_position;

    position_error = ...
        relative_position - DESIRED_OFFSET;

    error_N = position_error(1);
    error_U = position_error(2);
    error_E = position_error(3);

    integral_error = ...
        integral_error + position_error * dt;


    integral_error = max( ...
        -INTEGRAL_LIMIT, ...
        min(INTEGRAL_LIMIT, integral_error) ...
    );

    if ~derivative_initialized
        derivative_error = [0.0, 0.0, 0.0];
        previous_error = position_error;
        derivative_initialized = true;

    else
        derivative_error = ...
            (position_error - previous_error) / dt;
        previous_error = position_error;
    end

    P = KP * position_error;
    I = KI * integral_error;
    D = KD * derivative_error;


    control_vector = P + I + D;

    control_vector = max( ...
        MIN_CONTROL, ...
        min(MAX_CONTROL, control_vector) ...
    );

    control_N = control_vector(1);
    control_U = control_vector(2);
    control_E = control_vector(3);


    motor_1_command = control_N;
    motor_2_command = control_N;

    motor_3_command = control_E;
    motor_4_command = control_E;

    motor_5_command = control_U;
    motor_6_command = control_U;

    motor_1_command = max( ...
        MIN_CONTROL, ...
        min(MAX_CONTROL, motor_1_command) ...
    );

    motor_2_command = max( ...
        MIN_CONTROL, ...
        min(MAX_CONTROL, motor_2_command) ...
    );

    motor_3_command = max( ...
        MIN_CONTROL, ...
        min(MAX_CONTROL, motor_3_command) ...
    );

    motor_4_command = max( ...
        MIN_CONTROL, ...
        min(MAX_CONTROL, motor_4_command) ...
    );

    motor_5_command = max( ...
        MIN_CONTROL, ...
        min(MAX_CONTROL, motor_5_command) ...
    );

    motor_6_command = max( ...
        MIN_CONTROL, ...
        min(MAX_CONTROL, motor_6_command) ...
    );

    wb_motor_set_velocity( ...
        motors(1), motor_1_command);

    wb_motor_set_velocity( ...
        motors(2), motor_2_command);

    wb_motor_set_velocity( ...
        motors(3), motor_3_command);

    wb_motor_set_velocity( ...
        motors(4), motor_4_command);

    wb_motor_set_velocity( ...
        motors(5), motor_5_command);

    wb_motor_set_velocity( ...
        motors(6), motor_6_command);

    distance_3D = norm(relative_position);

    fprintf([ ...
        'UUV3 | ' ...
        'Rel[N U E]=[%.2f %.2f %.2f] | ' ...
        'Err[N U E]=[%.2f %.2f %.2f] | ' ...
        'Dist=%.2f m | ' ...
        'Ctrl[N U E]=[%.2f %.2f %.2f] | ' ...
        'Motors=[%.2f %.2f %.2f %.2f %.2f %.2f]\n' ...
        ], ...
        relative_position(1), ...
        relative_position(2), ...
        relative_position(3), ...
        error_N, ...
        error_U, ...
        error_E, ...
        distance_3D, ...
        control_N, ...
        control_U, ...
        control_E, ...
        motor_1_command, ...
        motor_2_command, ...
        motor_3_command, ...
        motor_4_command, ...
        motor_5_command, ...
        motor_6_command);

end

for i = 1:6
    wb_motor_set_velocity(motors(i), 0.0);
end

wb_robot_cleanup();