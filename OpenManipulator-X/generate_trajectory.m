%% generate_trajectory.m - Create smooth trajectory from recorded waypoints
%
% Loads waypoints from record_waypoints.m and generates:
%   q_desired(t)      - smooth position trajectory
%   q_desired_dot(t)  - velocity (analytical derivative of spline)
%   q_desired_ddot(t) - acceleration (analytical second derivative)
%
% Saves everything to 'desired_trajectory.mat' which run_robot.m can load.
clc; clear all; close all;

%% Settings
t_sample = 0.03;          % must match run_robot.m
time_per_segment = 5.0;   % seconds to travel between each waypoint
                          % increase for slower motion, decrease for faster

%% Load waypoints
data = load('recorded_waypoints.mat');
waypoints = data.waypoints;   % n_wp x 4
n_wp = size(waypoints, 1);

fprintf('Loaded %d waypoints\n', n_wp);
for i = 1:n_wp
    fprintf('  WP %d: [%.3f, %.3f, %.3f, %.3f] rad\n', i, waypoints(i,:));
end

%% Create time stamps for each waypoint
t_wp = (0:n_wp-1) * time_per_segment;   % equally spaced in time
tfin = t_wp(end);
t = 0:t_sample:tfin;
N = length(t);

fprintf('\nTrajectory duration: %.1f seconds\n', tfin);
fprintf('Time per segment: %.1f seconds\n', time_per_segment);
fprintf('Total samples: %d\n\n', N);

%% Generate smooth trajectory using cubic spline interpolation
q_desired = zeros(4, N);
q_desired_dot = zeros(4, N);
q_desired_ddot = zeros(4, N);

for joint = 1:4
    % fit cubic spline through waypoints for this joint
    pp = spline(t_wp, waypoints(:, joint)');
    
    % evaluate position
    q_desired(joint, :) = ppval(pp, t);
    
    % derivative of spline = velocity
    pp_dot = fnder(pp, 1);
    q_desired_dot(joint, :) = ppval(pp_dot, t);
    
    % second derivative = acceleration
    pp_ddot = fnder(pp, 2);
    q_desired_ddot(joint, :) = ppval(pp_ddot, t);
end

%% Save
save('desired_trajectory.mat', 'q_desired', 'q_desired_dot', 'q_desired_ddot', ...
     't_sample', 'waypoints', 't_wp', 'tfin');
fprintf('Saved trajectory to desired_trajectory.mat\n');

%% Plot the generated trajectory
figure;
for i = 1:4
    subplot(2,2,i);
    plot(t, q_desired(i,:), 'b', 'LineWidth', 1.5); hold on;
    plot(t_wp, waypoints(:,i), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    xlabel('Time [s]'); ylabel(sprintf('q%d [rad]', i));
    title(sprintf('Joint %d', i)); grid on;
    legend('Trajectory', 'Waypoints', 'Location', 'best');
end
sgtitle('Generated Trajectory - Positions');

figure;
for i = 1:4
    subplot(2,2,i);
    plot(t, q_desired_dot(i,:), 'b', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel(sprintf('dq%d [rad/s]', i));
    title(sprintf('Joint %d', i)); grid on;
end
sgtitle('Generated Trajectory - Velocities');

figure;
for i = 1:4
    subplot(2,2,i);
    plot(t, q_desired_ddot(i,:), 'b', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel(sprintf('ddq%d [rad/s^2]', i));
    title(sprintf('Joint %d', i)); grid on;
end
sgtitle('Generated Trajectory - Accelerations');

fprintf('\nTrajectory looks good? If yes, run run_robot.m with TASK_TYPE = ''trajectory''\n');
