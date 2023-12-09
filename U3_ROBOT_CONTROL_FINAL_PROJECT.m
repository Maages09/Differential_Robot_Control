% Constants from Code 1
goal_x = 5;  goal_y = 5;
robot_x = 0; robot_y = 0;
k_p_linear = 0.4; k_i_linear = 0.1;
k_p_angular = 0.5; k_i_angular = 0.6;
robot_theta = 0;
num_iterations = 100; dt = 0.1;
wheel_radius = 3.5; robot_width = 24.7;

% Initialize control errors for the PID controller
integral_linear_error = 0; integral_angular_error = 0;
errorA = 0; errorA1 = 0; errorA2 = 0;
errorB = 0; errorB1 = 0; errorB2 = 0;
OutputA = 0; OutputA1 = 0;
OutputB = 0; OutputB1 = 0;

% PID Constants from Code 2
KPA = 0.0228467; KIA = 0.90389; KDA = 0.0000685401; TiempoA = 0.9;
KPB = 0.0189727; KIB = 0.12745; KDB = 0.0000798181489; TiempoB = 0.9;

% Umbral para determinar si el robot ha llegado al objetivo
umbral_proximidad = 0.2;  % Ajusta este valor según sea necesario

% Create a figure for plotting
figure;
hold on;
plot(goal_x, goal_y, 'ro', 'MarkerSize', 10);  % Plot the goal position
axis([0 10 0 10]);  % Set axis limits for the plot
robot_handle = plot(robot_x, robot_y, 'bo', 'MarkerSize', 20);  % Initialize a graphical representation of the robot

% Create arrays to store the robot's path
path_x = zeros(1, num_iterations);
path_y = zeros(1, num_iterations);

% Simulation loop
for i = 1:num_iterations
    % Calculate the error in position and desired velocities
    error_x = goal_x - robot_x;
    error_y = goal_y - robot_y;
    v_desired = k_p_linear * error_x + k_i_linear * integral_linear_error;
    theta_desired = atan2(error_y, error_x);
    error_theta = theta_desired - robot_theta;
    omega_desired = k_p_angular * error_theta + k_i_angular * integral_angular_error;

    % Reducir la velocidad a medida que el robot se acerca al objetivo
    distancia_objetivo = norm([goal_x - robot_x, goal_y - robot_y]);
    if distancia_objetivo < umbral_proximidad
        factor_reduccion = distancia_objetivo / umbral_proximidad;
        v_desired = v_desired * factor_reduccion;
        omega_desired = omega_desired * factor_reduccion;
    end

    % Update the control errors for the PI controller
    integral_linear_error = integral_linear_error + error_x * dt;
    integral_angular_error = integral_angular_error + error_theta * dt;

    % Set the PID target speeds based on desired linear and angular velocities
    targetMA = (2 * v_desired - omega_desired * robot_width) / (2 * wheel_radius);
    targetMB = (2 * v_desired + omega_desired * robot_width) / (2 * wheel_radius);

    % PID Controller for Left Wheel Speed
    errorA = targetMA - OutputA1;  % Assuming OutputA1 represents current left wheel speed
    OutputA = OutputA1 + (KPA + (KDA/TiempoA)) * errorA + (-KPA + KIA * TiempoA -2 * (KDA/TiempoA)) * errorA1 + (KDA/TiempoA) * errorA2;
    OutputA1 = OutputA;
    errorA2 = errorA1;
    errorA1 = errorA;
    OutputA = max(min(OutputA, 250), 0);  % Apply limits

    % PID Controller for Right Wheel Speed
    errorB = targetMB - OutputB1;  % Assuming OutputB1 represents current right wheel speed
    OutputB = OutputB1 + (KPB + (KDB/TiempoB)) * errorB + (-KPB + KIB * TiempoB -2 * (KDB/TiempoB)) * errorB1 + (KDB/TiempoB) * errorB2;
    OutputB1 = OutputB;
    errorB2 = errorB1;
    errorB1 = errorB;
    OutputB = max(min(OutputB, 250), 0);  % Apply limits

    % Update the robot's position and orientation using PID outputs
    v_avg = (OutputA + OutputB) / 2;
    robot_x = robot_x + v_avg * cos(robot_theta) * dt;
    robot_y = robot_y + v_avg * sin(robot_theta) * dt;
    robot_theta = robot_theta + atan2((OutputB - OutputA) * wheel_radius, robot_width) * dt;

    % Update the graphical representation of the robot's position
    set(robot_handle, 'XData', robot_x, 'YData', robot_y);

    % Store the current robot position in the path arrays
    path_x(i) = robot_x;
    path_y(i) = robot_y;

    % Print the current robot state
    fprintf('Iteration %d: Robot Position (%.2f, %.2f), Robot Orientation %.2f\n', i, robot_x, robot_y, robot_theta);

    % Check if the robot has reached the goal
    if distancia_objetivo < umbral_proximidad
        fprintf('Robot reached the goal!\n');
        break;
    end

    % Pause for visualization (optional)
    pause(dt);
end

% Plot the robot's path
plot(path_x, path_y, 'b-');
xlabel('X Position');
ylabel('Y Position');
title('Robot Path');
grid on;
hold off;

