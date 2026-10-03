clear;

wb_robot_init();

TIME_STEP = 10;
dt = TIME_STEP / 1000.0;

output_dir = '/home/aditya/Desktop/BITS/SOP/project_from_scratch_windows/pid_graphs';

if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

uuv1_node = wb_supervisor_node_get_from_def('DRONE_UUV_1');
uuv2_node = wb_supervisor_node_get_from_def('DRONE_UUV_2');
uuv3_node = wb_supervisor_node_get_from_def('DRONE_UUV_3');

if uuv1_node == 0
    error('Could not find DRONE_UUV_1');
end

if uuv2_node == 0
    error('Could not find DRONE_UUV_2');
end

if uuv3_node == 0
    error('Could not find DRONE_UUV_3');
end

uuv_nodes = [uuv1_node, uuv2_node, uuv3_node];

TEST_DURATION = 50.0;
MAX_STEPS = round(TEST_DURATION / dt);
step_count = 0;

DISTURBANCE_DELAY = 10.0;

WAVE_AMPLITUDE = 1.0;
WAVE_OFFSET = 0.0;
WAVE_FREQUENCY = 0.5;

ROBOT_MASS = 60.0;

time_history = [];

wave_accel_history = [];
wave_force_history = [];

uuv1_position_history = [];
uuv2_position_history = [];
uuv3_position_history = [];

uuv1_speed_history = [];
uuv2_speed_history = [];
uuv3_speed_history = [];

distance_12_history = [];
distance_13_history = [];

prev_pos1 = [];
prev_pos2 = [];
prev_pos3 = [];

fprintf('Experiment controller started\n');
fprintf('Test duration: %.1f seconds\n', TEST_DURATION);
fprintf('Disturbance delay: %.1f seconds\n', DISTURBANCE_DELAY);

while wb_robot_step(TIME_STEP) ~= -1

    step_count = step_count + 1;
    elapsed = step_count * dt;

    if elapsed >= DISTURBANCE_DELAY

        wave_accel = WAVE_OFFSET + ...
            WAVE_AMPLITUDE * ...
            sin(2 * pi * WAVE_FREQUENCY * ...
            (elapsed - DISTURBANCE_DELAY));

    else

        wave_accel = 0.0;

    end

    wave_force = wave_accel * ROBOT_MASS;

    for i = 1:3
        wb_supervisor_node_add_force(uuv_nodes(i), [0, 0, wave_force], false);
    end

    pos1 = wb_supervisor_node_get_position(uuv1_node);
    pos2 = wb_supervisor_node_get_position(uuv2_node);
    pos3 = wb_supervisor_node_get_position(uuv3_node);

    if ~isempty(prev_pos1)

        speed1 = norm(pos1 - prev_pos1) / dt;
        speed2 = norm(pos2 - prev_pos2) / dt;
        speed3 = norm(pos3 - prev_pos3) / dt;

    else

        speed1 = 0.0;
        speed2 = 0.0;
        speed3 = 0.0;

    end

    prev_pos1 = pos1;
    prev_pos2 = pos2;
    prev_pos3 = pos3;

    distance_12 = norm(pos1 - pos2);
    distance_13 = norm(pos1 - pos3);

    time_history(end + 1) = elapsed;

    wave_accel_history(end + 1) = wave_accel;
    wave_force_history(end + 1) = wave_force;

    uuv1_position_history(end + 1, :) = pos1;
    uuv2_position_history(end + 1, :) = pos2;
    uuv3_position_history(end + 1, :) = pos3;

    uuv1_speed_history(end + 1) = speed1;
    uuv2_speed_history(end + 1) = speed2;
    uuv3_speed_history(end + 1) = speed3;
    
    distance_12_history(end + 1) = distance_12;
    distance_13_history(end + 1) = distance_13;
    
    if step_count >= MAX_STEPS
        fprintf('TEST DURATION REACHED: %.2f s\n', elapsed);
        break;
      end

    if mod(elapsed, 1.0) < dt
        fprintf('t = %.1f | Wave = %.2f N | UUV speeds = [%.2f %.2f %.2f] m/s\n', ...
            elapsed, wave_force, speed1, speed2, speed3);
    end

end

for i = 1:3
    wb_supervisor_node_add_force(uuv_nodes(i), [0, 0, 0], false);
end

figure('Name', 'UUV Speeds', 'Color', 'w');

plot(time_history, uuv1_speed_history, 'LineWidth', 1.5);
hold on;
plot(time_history, uuv2_speed_history, 'LineWidth', 1.5);
plot(time_history, uuv3_speed_history, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Speed of All UUVs');

legend('UUV 1 - Master', ...
       'UUV 2 - Slave', ...
       'UUV 3 - Slave');

grid on;

figure('Name', 'Wave Disturbance', 'Color', 'w');

plot(time_history, wave_force_history, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Disturbance Force (N)');
title('Applied Wave Disturbance');

grid on;

figure('Name', 'Formation Distances', 'Color', 'w');

plot(time_history, distance_12_history, 'LineWidth', 1.5);
hold on;
plot(time_history, distance_13_history, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Distance (m)');
title('Distance Between Master and Slave UUVs');

legend('UUV 1 to UUV 2', ...
       'UUV 1 to UUV 3');

grid on;

experiment_data = table( ...
    time_history', ...
    uuv1_position_history(:,1), ...
    uuv1_position_history(:,2), ...
    uuv1_position_history(:,3), ...
    uuv2_position_history(:,1), ...
    uuv2_position_history(:,2), ...
    uuv2_position_history(:,3), ...
    uuv3_position_history(:,1), ...
    uuv3_position_history(:,2), ...
    uuv3_position_history(:,3), ...
    uuv1_speed_history', ...
    uuv2_speed_history', ...
    uuv3_speed_history', ...
    distance_12_history', ...
    distance_13_history', ...
    wave_accel_history', ...
    wave_force_history', ...
    'VariableNames', { ...
        'Time_s', ...
        'UUV1_X_m', ...
        'UUV1_Y_m', ...
        'UUV1_Z_m', ...
        'UUV2_X_m', ...
        'UUV2_Y_m', ...
        'UUV2_Z_m', ...
        'UUV3_X_m', ...
        'UUV3_Y_m', ...
        'UUV3_Z_m', ...
        'UUV1Speed_mps', ...
        'UUV2Speed_mps', ...
        'UUV3Speed_mps', ...
        'Distance_UUV1_UUV2_m', ...
        'Distance_UUV1_UUV3_m', ...
        'WaveAcceleration_mps2', ...
        'WaveForce_N' ...
    } ...
);

output_file = fullfile(output_dir, 'experiment_results.csv');

fprintf('Saving experiment data...\n');
fprintf('Output directory: %s\n', output_dir);
fprintf('Output file: %s\n', output_file);

writetable(experiment_data, output_file);

if exist(output_file, 'file')
    file_info = dir(output_file);

    fprintf('CSV successfully created.\n');
    fprintf('File size: %d bytes\n', file_info.bytes);
else
    fprintf('ERROR: CSV file was NOT created.\n');
end

fprintf('Experiment complete.\n');

while wb_robot_step(TIME_STEP) ~= -1
end