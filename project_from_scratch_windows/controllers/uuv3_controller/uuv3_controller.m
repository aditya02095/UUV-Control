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
uuv1_position = [];

DESIRED_OFFSET = 0.0;
KP_DISTANCE = 0.6;
KI_DISTANCE = 0.18;
KD_DISTANCE = 0.1;

distance_integral = 0.0;
previous_distance_error = 0.0;

previous_distance_error = 0.0;
derivative_initialized = false;

INTEGRAL_LIMIT = 20.0;

MIN_THRUST = -20.0;
MAX_THRUST = 20.0;

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


while wb_robot_step(TIME_STEP) ~= -1

    own_position = wb_gps_get_values(gps);
 
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
    
    own_position = reshape(own_position, 1, 3);
    uuv1_position = reshape(uuv1_position, 1, 3);
    dz = own_position(3) - uuv1_position(3);

    distance_error = dz - DESIRED_OFFSET;
    P = KP_DISTANCE * distance_error;

    distance_integral = distance_integral + distance_error * dt;
    distance_integral = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, distance_integral));
    I = KI_DISTANCE * distance_integral;

    if ~derivative_initialized
        distance_derivative = 0.0;
        previous_distance_error = distance_error;
        derivative_initialized = true;
    else
        distance_derivative = (distance_error - previous_distance_error) / dt;
        previous_distance_error = distance_error;
    end
    D = KD_DISTANCE * distance_derivative;
    
    thrust = -(P + I + D);
    thrust = max(MIN_THRUST, min(MAX_THRUST, thrust));

    wb_motor_set_velocity(motors(1), thrust);
    wb_motor_set_velocity(motors(2), thrust);
    wb_motor_set_velocity(motors(3), 0.0);
    wb_motor_set_velocity(motors(4), 0.0);

    fprintf('Distance 2: %.2f m | Error: %.2f | Thrust: %.2f\n', dz, distance_error, thrust);
end


for i = 1:4
    wb_motor_set_velocity(motors(i), 0.0);
end

wb_robot_cleanup();