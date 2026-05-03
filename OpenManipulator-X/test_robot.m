%% test_robot.m - Matching friend's working code structure exactly
% Key differences from our previous attempts:
%   1. NO homing, NO position mode. Go straight to current mode.
%   2. Manually position robot near home before running.
%   3. Simple test: move only Joint 4 to 1.0 rad (same as friend's test)
clc; clear all; close all;

addpath("Communication_Code");
addpath("generated_dynamics");

%% Conversions
factor_degre_to_rad = pi/180;
factor_mA_to_A = 1/1000;
factor_A_to_mA = 1000/1;

%% Load parameters
R = load('Identification/identification_result.mat');
p = [R.p(1:6); ...
     R.x_opt_vec(1); R.x_opt_vec(2); R.x_opt_vec(3); R.x_opt_vec(4); ...
     R.x_opt_vec(5); R.x_opt_vec(6); R.x_opt_vec(7); ...
     R.x_opt_vec(8); R.x_opt_vec(9); R.x_opt_vec(10); ...
     R.x_opt_vec(11); R.x_opt_vec(12); R.x_opt_vec(13); ...
     R.x_opt_vec(14); R.x_opt_vec(15); R.x_opt_vec(16); ...
     R.id_info.g];
pf = [R.x_opt_vec(17); R.x_opt_vec(18); R.x_opt_vec(19); R.x_opt_vec(20)];

%% Gains (same as friend: Kp=10, Kv=2)
Kp = 10*eye(4);
Kv = 2*eye(4);

%% Trajectory - same as friend's test: only joint 4 moves to 1.0
t_sample = 0.04;
tfin = 3;
t = 0:t_sample:tfin;
N = length(t);

q_desired = [0.0*ones(1,N); 0.0*ones(1,N); 0.0*ones(1,N); 1.0*ones(1,N)];
q_desired_dot = zeros(4, N);
q_desired_ddot = zeros(4, N);

%% Connect robot - go DIRECTLY to current mode (no homing)
fprintf('=== IMPORTANT: Manually position robot near home [0,0,0,0] before running ===\n');
fprintf('Press Enter when robot is positioned...\n');
pause;

robot = Robot();
cleanupObj = onCleanup(@() safe_stop(robot));

%% Set current control mode directly (like friend's code)
robot.writeMode('c');

%% Storage
q_real = zeros(4, N+1);
q_dot_real = zeros(4, N+1);
tau_k = zeros(4, N);
dt = zeros(1, N);

%% Read initial conditions (friend's way)
joint_readings = robot.getJointsReadings();
q_real(:, 1) = (joint_readings(1, :)*factor_degre_to_rad)';
q_dot_real(:, 1) = (joint_readings(2, :)*factor_degre_to_rad)';

fprintf('Initial q: [%.3f, %.3f, %.3f, %.3f] rad\n', q_real(:,1));
fprintf('Target q:  [%.3f, %.3f, %.3f, %.3f] rad\n\n', q_desired(:,1));

%% Control loop - EXACTLY matching friend's structure
for k = 1:N
    tic

    % compute using previous readings (friend's way)
    q_now = (joint_readings(1, :)*factor_degre_to_rad)';
    q_dot_now = (joint_readings(2, :)*factor_degre_to_rad)';

    % computed torque controller
    e = q_desired(:,k) - q_now;
    e_dot = q_desired_dot(:,k) - q_dot_now;

    M = M_fun(q_now, p);
    C = C_fun(q_now, q_dot_now, p);
    G = G_fun(q_now, p);

    aq = q_desired_ddot(:,k) + Kv * e_dot + Kp * e;
    tau_k(:, k) = M * aq + C * q_dot_now + G;

    % convert and send (friend's way)
    torques = [tau_k(1,k), tau_k(2,k), tau_k(3,k), tau_k(4,k)];
    current = torque_to_current(torques);
    current_mA = current * factor_A_to_mA;
    robot.writeCurrents(current_mA);

    % wait for sample period
    while toc < t_sample
    end
    dt(k) = toc;

    % read for next iteration (friend's way)
    joint_readings = robot.getJointsReadings();
    q_real(:, k+1) = (joint_readings(1, :)*factor_degre_to_rad)';
    q_dot_real(:, k+1) = (joint_readings(2, :)*factor_degre_to_rad)';

    % print every second
    if mod(k, round(1/t_sample)) == 0
        fprintf('t=%4.1fs | q=[%+.3f %+.3f %+.3f %+.3f] | e=%.3f\n', ...
            t(k), q_now(1), q_now(2), q_now(3), q_now(4), norm(e));
    end
end

%% Stop
robot.writeCurrents([0,0,0,0]);
fprintf('\nDone!\n');

%% Plot (friend's style)
q_real_k = q_real(:, 1:N);

figure(1)
for i = 1:4
    subplot(2,2,i)
    plot(t, q_desired(i,:), 'r.'); hold on;
    plot(t, q_real_k(i,:), 'b.');
    xlabel('Time [s]'); ylabel(sprintf('q%d [rad]', i));
    title(sprintf('Joint %d', i));
    legend('Desired', 'Current', 'Location', 'best');
    grid on;
end
sgtitle('Computed Torque - Joint Positions');

figure(2)
for i = 1:4
    subplot(2,2,i)
    plot(t, tau_k(i,:), 'm.');
    xlabel('Time [s]'); ylabel(sprintf('tau%d [Nm]', i));
    title(sprintf('Joint %d', i));
    grid on;
end
sgtitle('Computed Torque - Control Torques');

function safe_stop(robot)
    fprintf('\n!!! Zero current !!!\n');
    try, robot.writeCurrents([0,0,0,0]); catch, end
end