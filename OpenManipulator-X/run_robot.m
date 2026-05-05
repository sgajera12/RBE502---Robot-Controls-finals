%% run_robot.m - Real Robot Control for RBE 502 Final Project
% Matches working code structure: connect → current mode → control loop.
% No homing, no manual positioning. Robot starts wherever it is and
% the controller drives it to the desired position.
clc; clear all; close all;

CONTROLLER_TYPE = 'ct';% 'ct', 'robust', or 'adaptive'
TASK_TYPE = 'regulation';% 'regulation' or 'trajectory'

fprintf(' RBE 502 Final Project \n');
fprintf('Controller: %s | Task: %s\n\n', CONTROLLER_TYPE, TASK_TYPE);

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
p_geom = [p(1:6); p(23)];
% p(10) = 0.100;


%% Gains 
% Kp = diag([20 30 40 30]);
% Kv = diag([2 3 2 1 ]);
Kp = diag([30 20 50 15]);
Kv = diag([2 2 3 1 ]);
%% Lyapunov equation (for robust and adaptive)
n = 4;
A_err = [zeros(n) eye(n); -Kp -Kv];
Q_lyap = eye(2*n);
P_lyap = lyap(A_err', Q_lyap);

%% Robust parameters
rho = 5;
epsilon = 1;

%% Adaptive parameters
n_params = 16;
pi_hat = p(7:22);
R_gain = 1e5 * eye(n_params);
R_inv = inv(R_gain);

%% Generate desired trajectory
if strcmp(TASK_TYPE, 'regulation')
    %% Timing
    t_sample = 0.01;
    tfin = 5;
    t = 0:t_sample:tfin;
    N = length(t);
    q_desired = repmat([0.50; -0.35; 0.30; 0.15], 1, N);
    q_desired_dot = zeros(4, N);
    q_desired_ddot = zeros(4, N);
elseif strcmp(TASK_TYPE, 'trajectory')
    t_sample = 0.04;
    tfin = 15;
    t = 0:t_sample:tfin;
    N = length(t);

    % Very gentle sine wave - small amplitude, slow speed
    % Each joint: q_d(t) = A * sin(omega * t)
    % A = 0.3 rad (~17 degrees), omega = 0.4 rad/s (one cycle in ~16 seconds)
    A_amp   = [0.3;  0.2;  0.15;  0.2];     % smaller for joints 3,4
    omega   = [0.4;  0.4;  0.4;   0.4];     % same slow speed

    q_desired = zeros(4, N);
    q_desired_dot = zeros(4, N);
    q_desired_ddot = zeros(4, N);
    for k = 1:N
        q_desired(:,k)      = A_amp .* sin(omega * t(k));
        q_desired_dot(:,k)  = A_amp .* omega .* cos(omega * t(k));
        q_desired_ddot(:,k) = -A_amp .* (omega.^2) .* sin(omega * t(k));
    end
end
% elseif strcmp(TASK_TYPE, 'trajectory')
%     % load trajectory from recorded waypoints
%     % run record_waypoints.m then generate_trajectory.m first
%     traj = load('desired_trajectory.mat');
%     q_desired = traj.q_desired;
%     q_desired_dot = traj.q_desired_dot;
%     q_desired_ddot = traj.q_desired_ddot;
%     t_sample = traj.t_sample;
%     tfin = traj.tfin;
%     t = 0:t_sample:tfin;
%     N = length(t);
% 
%     % make sure trajectory and time vector match
%     if size(q_desired, 2) ~= N
%         N = min(N, size(q_desired, 2));
%         t = t(1:N);
%         q_desired = q_desired(:, 1:N);
%         q_desired_dot = q_desired_dot(:, 1:N);
%         q_desired_ddot = q_desired_ddot(:, 1:N);
%     end
% 
%     fprintf('Loaded trajectory: %d waypoints, %.1f seconds\n', ...
%         size(traj.waypoints, 1), tfin);
% end

%% Connect robot and go directly to current mode (like friend's code)
robot = Robot();
cleanupObj = onCleanup(@() safe_stop(robot));
robot.writeMode('c');

%% Storage
q_real = zeros(4, N+1);
q_dot_real = zeros(4, N+1);
tau_hist = zeros(4, N);
dt = zeros(1, N);
if strcmp(CONTROLLER_TYPE, 'adaptive')
    pi_hat_hist = zeros(n_params, N);
end

%% Read initial conditions
joint_readings = robot.getJointsReadings();
q_real(:, 1) = (joint_readings(1, :)*factor_degre_to_rad)';
q_dot_real(:, 1) = (joint_readings(2, :)*factor_degre_to_rad)';

fprintf('Start: [%.3f, %.3f, %.3f, %.3f] rad\n', q_real(:,1));
fprintf('Target: [%.3f, %.3f, %.3f, %.3f] rad\n\n', q_desired(:,1));

%% Control loop (matching friend's working structure exactly)
for k = 1:N
    tic

    % current state from previous readings (friend's way)
    q_now = (joint_readings(1, :)*factor_degre_to_rad)';
    q_dot_now = (joint_readings(2, :)*factor_degre_to_rad)';

    % desired at this time step
    qd = q_desired(:, k);
    qd_dot = q_desired_dot(:, k);
    qd_ddot = q_desired_ddot(:, k);

    % error
    e = qd - q_now;
    e_dot = qd_dot - q_dot_now;

    % compute torque based on controller type
    switch CONTROLLER_TYPE
        case 'ct'
            % computed torque (same as friend's tau_computed_torque)
            M = M_fun(q_now, p);
            C = C_fun(q_now, q_dot_now, p);
            G = G_fun(q_now, p);
            aq = qd_ddot + Kv * e_dot + Kp * e;
            tau_hist(:, k) = M * aq + C * q_dot_now + G;

        case 'robust'
            M = M_fun(q_now, p);
            C = C_fun(q_now, q_dot_now, p);
            G = G_fun(q_now, p);

            z = [e; e_dot];
            B_mat = [zeros(n); eye(n)];
            w = B_mat' * P_lyap * z;
            w_norm = norm(w);
            if w_norm > epsilon
                Delta = rho * (w / w_norm);
            else
                Delta = rho * (w / epsilon);
            end

            aq = qd_ddot + Kv * e_dot + Kp * e + Delta;
            tau_hist(:, k) = M * aq + C * q_dot_now + G;

        case 'adaptive'
            pi_hat(1:4) = max(pi_hat(1:4), 1e-4);
            pi_hat(5:16) = max(pi_hat(5:16), 1e-8);

            p_hat = [p_geom(1:6); pi_hat; p_geom(7)];
            M_hat = M_fun(q_now, p_hat);
            C_hat = C_fun(q_now, q_dot_now, p_hat);
            G_hat = G_fun(q_now, p_hat);

            aq = qd_ddot + Kv * e_dot + Kp * e;
            tau_hist(:, k) = M_hat * aq + C_hat * q_dot_now + G_hat;

            % adaptation law
            z = [e; e_dot];
            w = P_lyap(n+1:2*n, :) * z;
            Y = Y_fun(q_now, q_dot_now, aq, p_geom);
            pi_hat_dot = R_inv * (Y' * (M_hat' \ w));
            max_rate = max(abs(pi_hat), 1e-6);
            pi_hat_dot = max(min(pi_hat_dot, max_rate), -max_rate);
            pi_hat_dot(~isfinite(pi_hat_dot)) = 0;
            pi_hat = pi_hat + t_sample * pi_hat_dot;
            pi_hat_hist(:, k) = pi_hat;
    end

    % send torque to robot (same conversion as friend's code)
    % friction compensation (real robot only, per project instructions)
    % F = ViscousFriction_fun(q_dot_now, pf);
    % tau_hist(:, k) = tau_hist(:, k) + F;
    % 
    % torques = [tau_hist(1,k), tau_hist(2,k), tau_hist(3,k), tau_hist(4,k)];
    torques = [tau_hist(1,k), tau_hist(2,k), tau_hist(3,k), tau_hist(4,k)];
    current = torque_to_current(torques);
    current_mA = current * factor_A_to_mA;
    robot.writeCurrents(current_mA);

    % wait for sample period
    while toc < t_sample
    end
    dt(k) = toc;

    % read for next iteration
    joint_readings = robot.getJointsReadings();
    q_real(:, k+1) = (joint_readings(1, :)*factor_degre_to_rad)';
    q_dot_real(:, k+1) = (joint_readings(2, :)*factor_degre_to_rad)';

    if mod(k, round(1/t_sample)) == 0
        fprintf('t=%4.1fs | e_norm=%.4f\n', t(k), norm(e));
    end
end

%% Stop
robot.writeCurrents([0, 0, 0, 0]);
fprintf('\nDone! Max dt: %.4fs\n', max(dt));

%% Save
q_real_k = q_real(:, 1:N);
q_dot_real_k = q_dot_real(:, 1:N);
save_name = sprintf('real_%s_%s.mat', CONTROLLER_TYPE, TASK_TYPE);
if strcmp(CONTROLLER_TYPE, 'adaptive')
    save(save_name, 't', 'q_real_k', 'q_dot_real_k', 'tau_hist', ...
         'q_desired', 'q_desired_dot', 'dt', 'pi_hat_hist');
else
    save(save_name, 't', 'q_real_k', 'q_dot_real_k', 'tau_hist', ...
         'q_desired', 'q_desired_dot', 'dt');
end
fprintf('Saved: %s\n', save_name);

%% Plots
fig_prefix = sprintf('%s / %s', upper(CONTROLLER_TYPE), TASK_TYPE);

figure;
for i = 1:4
    subplot(2,2,i);
    plot(t, q_desired(i,1:N), 'r--', 'LineWidth', 1.5); hold on;
    plot(t, q_real_k(i,:), 'b', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel(sprintf('q%d [rad]', i));
    title(sprintf('Joint %d', i)); grid on;
    legend('Desired', 'Current', 'Location', 'best');
end
sgtitle([fig_prefix ' - Joint Positions']);

figure;
for i = 1:4
    subplot(2,2,i);
    plot(t, q_dot_real_k(i,:), 'b', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel(sprintf('dq%d [rad/s]', i));
    title(sprintf('Joint %d', i)); grid on;
end
sgtitle([fig_prefix ' - Joint Velocities']);

figure;
for i = 1:4
    subplot(2,2,i);
    plot(t, tau_hist(i,:), 'b', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel(sprintf('tau%d [Nm]', i));
    title(sprintf('Joint %d', i)); grid on;
end
sgtitle([fig_prefix ' - Control Torques']);

e_norm = vecnorm(q_desired(:,1:N) - q_real_k);
figure;
plot(t, e_norm, 'b', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('||e(t)||_2');
title([fig_prefix ' - Tracking Error Norm']); grid on;

if strcmp(CONTROLLER_TYPE, 'adaptive')
    param_names = {'m1','m2','m3','m4'};
    figure;
    for i = 1:4
        subplot(2,2,i);
        plot(t, pi_hat_hist(i,:), 'b', 'LineWidth', 1.2);
        hold on; yline(p(6+i), 'k:', 'LineWidth', 1);
        xlabel('Time [s]'); ylabel(param_names{i});
        title(param_names{i}); grid on;
    end
    sgtitle([fig_prefix ' - Parameter Evolution']);
end