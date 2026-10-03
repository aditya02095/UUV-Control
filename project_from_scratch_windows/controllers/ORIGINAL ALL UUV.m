clear;

webots_path = '/usr/local/webots/lib/controller/matlab';
if isempty(strfind(path, webots_path))
    addpath(webots_path);
end
setenv('LD_LIBRARY_PATH', ['/usr/local/webots/lib/controller:', getenv('LD_LIBRARY_PATH')]);
setenv('WEBOTS_HOME', '/usr/local/webots');

wb_robot_init();

TIME_STEP = 32;
dt = TIME_STEP / 1000.0;

% --- Supervisor setup for all three UUVs ---
uuv1_node = wb_supervisor_node_get_from_def('DRONE_UUV_1');
uuv2_node = wb_supervisor_node_get_from_def('DRONE_UUV_2');
uuv3_node = wb_supervisor_node_get_from_def('DRONE_UUV_3');

uuv_nodes = [uuv1_node, uuv2_node, uuv3_node];

% Fields used for resetting each UUV
trans_fields = [
    wb_supervisor_node_get_field(uuv1_node, 'translation'),
    wb_supervisor_node_get_field(uuv2_node, 'translation'),
    wb_supervisor_node_get_field(uuv3_node, 'translation')
];

rot_fields = [
    wb_supervisor_node_get_field(uuv1_node, 'rotation'),
    wb_supervisor_node_get_field(uuv2_node, 'rotation'),
    wb_supervisor_node_get_field(uuv3_node, 'rotation')
];

lin_vel_fields = [
    wb_supervisor_node_get_field(uuv1_node, 'linearVelocity'),
    wb_supervisor_node_get_field(uuv2_node, 'linearVelocity'),
    wb_supervisor_node_get_field(uuv3_node, 'linearVelocity')
];

ang_vel_fields = [
    wb_supervisor_node_get_field(uuv1_node, 'angularVelocity'),
    wb_supervisor_node_get_field(uuv2_node, 'angularVelocity'),
    wb_supervisor_node_get_field(uuv3_node, 'angularVelocity')
];

% --- Disturbance parameters ---
WAVE_AMPLITUDE    = 1.5;   % m/s²
WAVE_OFFSET       = -1.5;  % m/s² — always negative = always backward
WAVE_FREQUENCY    = 0.5;   % Hz
ROBOT_MASS        = 60.0;  % kg
DISTURBANCE_DELAY = 0.0;   % seconds before disturbance starts

% --- 2. Device Setup

% UUV 1 is the leader and has GPS
gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);

% The controller's own UUV has an IMU
imu = wb_robot_get_device('imu');
wb_inertial_unit_enable(imu, TIME_STEP);

% Motors belonging to the UUV running this controller
motors = [ ...
    wb_robot_get_device('motor_1'), ...
    wb_robot_get_device('motor_2'), ...
    wb_robot_get_device('motor_3'), ...
    wb_robot_get_device('motor_4') ...
];

% Initialize motors
for i = 1:4
    wb_motor_set_position(motors(i), inf);
    wb_motor_set_velocity(motors(i), 0.0);
end

% --- 3. PID Sweep Ranges

KP_range = 0:1:10;
KI_range = 0:0.25:4.25;
KD_range = 0.1:0.25:4.0;

TARGET_SPEED  = 3.2;
TEST_DURATION = 75.0;

% --- 4. Output Folder

project_dir = '/home/aditya/Desktop/BITS/SOP/project_from_scratch_windows';

output_dir = fullfile(project_dir, 'pid_graphs');

if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

fprintf('Saving graphs to: %s\n', output_dir);

% --- 5. Build Full PID Combination List

combos = [];

for kp = KP_range
    for ki = KI_range
        for kd = KD_range

            if kp == 0 && ki == 0
                continue;
            end

            combos(end+1, :) = [kp, ki, kd];

        end
    end
end

total_combos = size(combos, 1);

fprintf('Total PID combinations: %d\n', total_combos);

fprintf('Estimated time: %.1f minutes\n', ...
        total_combos * TEST_DURATION / 60);

% --- 6. Loop Over All PID Combinations

for c = 1:total_combos

    KP = combos(c, 1);
    KI = combos(c, 2);
    KD = combos(c, 3);

    fprintf('\n[%d/%d] Testing KP=%.1f  KI=%.1f  KD=%.1f\n', ...
            c, total_combos, KP, KI, KD);

    % Reset PID state

    prev_pos       = [];
    speed_integral = 0.0;
    prev_err       = 0.0;

    % Reset logging arrays

    time_history         = [];
    actual_speed_history = [];
    target_history       = [];
    thrust_history       = [];
    wave_history         = [];

    % Stop motors before beginning the next test

    for i = 1:4
        wb_motor_set_velocity(motors(i), 0.0);
    end

    % Start test timer

    start_time = wb_robot_get_time();

    % --- Inner Simulation Loop ---

    while wb_robot_step(TIME_STEP) ~= -1

        curr_time = wb_robot_get_time();

        elapsed = curr_time - start_time;

        % End current PID test
        if elapsed >= TEST_DURATION
            break;
        end

        % Common Wave Disturbance

        % Start disturbance after motors have had time to spool up
        if elapsed > DISTURBANCE_DELAY

            wave_accel = WAVE_OFFSET + ...
                         WAVE_AMPLITUDE * ...
                         sin(2 * pi * WAVE_FREQUENCY * elapsed);

        else
            wave_accel = 0.0;
        end

        wave_force = wave_accel * ROBOT_MASS;

        % Apply EXACTLY the same disturbance to all 3 UUVs

        for uuv = 1:3

            wb_supervisor_node_add_force( ...
                uuv_nodes(uuv), ...
                [0, 0, wave_force], ...
                false);

        end

        % Read Leader Sensors

        pos = wb_gps_get_values(gps);

        rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);

        if isempty(pos) || any(isnan(pos)) || ...
           isempty(rpy) || any(isnan(rpy))

            continue;
        end

        % Calculate Leader Speed Using GPS + IMU

        if ~isempty(prev_pos)

            vx = (pos(1) - prev_pos(1)) / dt;

            vz = (pos(3) - prev_pos(3)) / dt;

            yaw = rpy(3);

            % Project world velocity onto the UUV's
            % forward direction
            actual_speed = ...
                vx * sin(yaw) + ...
                vz * cos(yaw);

        else

            actual_speed = 0.0;

        end

        prev_pos = pos;

        % PID Controller

        err = TARGET_SPEED - actual_speed;


        % Integral term
        speed_integral = speed_integral + err * dt;

        % Anti-windup
        speed_integral = max( ...
            -20, ...
            min(20, speed_integral));


        % Derivative term
        derivative = (err - prev_err) / dt;

        prev_err = err;


        % PID output
        thrust = ...
            KP * err + ...
            KI * speed_integral + ...
            KD * derivative;


        % Limit motor command
        clamped_thrust = max( ...
            -20, ...
            min(20, thrust));

        % Print Status

        if mod(elapsed, 0.25) < (TIME_STEP / 1000.0)

            fprintf([ ...
                '  t=%.1fs | actual: %.3f m/s | ' ...
                'thrust: %.3f | wave: %.2f N\n'], ...
                elapsed, ...
                actual_speed, ...
                clamped_thrust, ...
                wave_force);

        end

        % Apply Motor Commands

        wb_motor_set_velocity(motors(1), clamped_thrust);
        wb_motor_set_velocity(motors(2), clamped_thrust);

        wb_motor_set_velocity(motors(3), 0.0);
        wb_motor_set_velocity(motors(4), 0.0);

        % Log Data

        time_history(end+1) = elapsed;

        actual_speed_history(end+1) = actual_speed;

        target_history(end+1) = TARGET_SPEED;

        thrust_history(end+1) = clamped_thrust;

        wave_history(end+1) = wave_accel;

    end

    % Stop Motors

    for i = 1:4
        wb_motor_set_velocity(motors(i), 0.0);
    end

    % --- Save Plot ---

    fig = figure( ...
        'Name', sprintf( ...
            'KP=%.1f KI=%.1f KD=%.1f', ...
            KP, KI, KD), ...
        'Color', 'w', ...
        'Visible', 'off');


    % ---------------- Speed plot ----------------

    subplot(2, 1, 1);

    plot(time_history, ...
         target_history, ...
         'r--', ...
         'LineWidth', 1.5);

    hold on;

    plot(time_history, ...
         actual_speed_history, ...
         'b-', ...
         'LineWidth', 2);

    plot(time_history, ...
         wave_history, ...
         'm:', ...
         'LineWidth', 1.5);

    title(sprintf( ...
        'Speed Tracking — KP=%.1f KI=%.1f KD=%.1f', ...
        KP, KI, KD));

    xlabel('Time (s)');

    ylabel('Speed (m/s)');

    legend( ...
        'Target speed', ...
        'Actual speed', ...
        'Disturbance (m/s^2)');

    grid on;


    % ---------------- Thrust plot ----------------

    subplot(2, 1, 2);

    plot(time_history, ...
         thrust_history, ...
         'g-', ...
         'LineWidth', 2);

    title('Thrust Command');

    xlabel('Time (s)');

    ylabel('Motor velocity');

    grid on;

    % Save graph

    fname = sprintf( ...
        'KP%.1f_KI%.1f_KD%.1f.png', ...
        KP, KI, KD);

    fname = strrep(fname, '.', 'p');

    saveas(fig, ...
           fullfile(output_dir, fname), ...
           'png');

    close(fig);

    fprintf('  Saved: %s\n', fname);


    % Reset All Three UUVs

    % Stop all motors on the controlled UUV
    for i = 1:4
        wb_motor_set_velocity(motors(i), 0.0);
    end


    % Reset UUV 1 to its initial position
    wb_supervisor_field_set_sf_vec3f( ...
        trans_fields(1), ...
        [-10, 1, 88.30446625192333]);


    % Reset UUV 2 to its initial position
    wb_supervisor_field_set_sf_vec3f( ...
        trans_fields(2), ...
        [0, 1, 88.30446625192333]);


    % Reset UUV 3 to its initial position
    wb_supervisor_field_set_sf_vec3f( ...
        trans_fields(3), ...
        [10, 1, 88.30446625192333]);


    % Reset orientation and velocities
    for uuv = 1:3

        wb_supervisor_field_set_sf_rotation( ...
            rot_fields(uuv), ...
            [0, 0, 1, 0]);


        wb_supervisor_field_set_sf_vec3f( ...
            lin_vel_fields(uuv), ...
            [0, 0, 0]);


        wb_supervisor_field_set_sf_vec3f( ...
            ang_vel_fields(uuv), ...
            [0, 0, 0]);

    end


    % Reset Webots physics engine
    wb_supervisor_simulation_reset_physics();


    % Allow Webots to update after reset
    wb_robot_step(TIME_STEP);
    wb_robot_step(TIME_STEP);

end

% Final Cleanup

for i = 1:4
    wb_motor_set_velocity(motors(i), 0.0);
end

wb_robot_cleanup();

fprintf('\nAll %d graphs saved to:\n%s\n', ...
        total_combos, output_dir);