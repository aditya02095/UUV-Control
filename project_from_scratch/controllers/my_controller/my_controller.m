% Clear workspace but keep the system path if possible
clear; 

% --- 0. Environment Setup (Standard .deb Path) ---
webots_path = '/usr/local/webots/lib/controller/matlab';
if isempty(strfind(path, webots_path))
    addpath(webots_path);
end

% Set the library path for the current session
setenv('LD_LIBRARY_PATH', '/usr/local/webots/lib/controller');

% --- 1. Webots Initialization ---
try
    wb_robot_init(); 
catch
    error('Webots API not found. Please ensure Webots is installed at /usr/local/webots');
end

TIME_STEP = wb_robot_get_basic_time_step();
dt = TIME_STEP / 1000.0;

% --- 2. Device Setup ---
gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);
imu = wb_robot_get_device('imu');
wb_inertial_unit_enable(imu, TIME_STEP);

% Get all 4 motors
motors = [wb_robot_get_device('motor_1'), wb_robot_get_device('motor_2'), ...
          wb_robot_get_device('motor_3'), wb_robot_get_device('motor_4')];

% Initialize Motors
for i = 1:4
    wb_motor_set_position(motors(i), inf);
    wb_motor_set_velocity(motors(i), 0.0);
end

% --- 3. Parameters (As requested) ---
KP = 10; 
KI = 1.0; 
KD = 0.5;
TARGET_SPEED = 3.5; % Target speed
MAX_TIME = 30.0;    % Simulation duration

% --- 4. State Variables ---
prev_pos = [];
speed_integral = 0.0;
prev_error = 0.0;
time_history = [];
actual_speed_history = [];
target_history = [];

fprintf('UUV Controller Started. Waiting for sensors...\n');

% --- 5. Main Webots Loop ---
while wb_robot_step(TIME_STEP) ~= -1
    curr_time = wb_robot_get_time();
    
    if curr_time > MAX_TIME
        fprintf('30 seconds reached. Stopping UUV.\n');
        break; 
    end

    % Get Sensor Values
    pos = wb_gps_get_values(gps);
    rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    
    if isempty(pos) || any(isnan(pos)) || isempty(rpy)
        continue; 
    end

    % --- Signed Speed Calculation (GPS + IMU Fusion) ---
    if ~isempty(prev_pos)
        vx = (pos(1) - prev_pos(1)) / dt;
        vz = (pos(3) - prev_pos(3)) / dt;
        yaw = rpy(3); % Yaw angle
        
        % Vector projection for signed speed
        actual_speed = (vx * sin(yaw) + vz * cos(yaw)); 
    else
        actual_speed = 0.0;
    end
    prev_pos = pos;

    % --- PID Logic ---
    error = TARGET_SPEED - actual_speed;
    speed_integral = speed_integral + (error * dt);
    speed_integral = max(-10, min(10, speed_integral)); % Anti-windup
    
    derivative = (error - prev_error) / dt;
    prev_error = error;

    thrust = (KP * error) + (KI * speed_integral) + (KD * derivative);
    clamped_thrust = max(-20, min(20, thrust)); 

    % Apply to motors
    for i = 1:4
        wb_motor_set_velocity(motors(i), clamped_thrust);
    end

    % Log Data
    time_history(end+1) = curr_time;
    actual_speed_history(end+1) = actual_speed;
    target_history(end+1) = TARGET_SPEED;
end

% Stop motors
for i = 1:4, wb_motor_set_velocity(motors(i), 0); end

% --- 6. Plotting ---
figure('Name', 'UUV Speed Response', 'Color', 'w');
plot(time_history, actual_speed_history, 'b', 'LineWidth', 2); hold on;
plot(time_history, target_history, 'r--', 'LineWidth', 1.5);
title(sprintf('PID Response (P=%d, I=%.1f, D=%.1f)', KP, KI, KD));
xlabel('Time (s)'); ylabel('Speed (m/s)');
legend('Actual Speed', 'Target Speed');
grid on;