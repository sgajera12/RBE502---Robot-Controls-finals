%% plot_real_results.m - Compare real robot results across controllers
% Run this AFTER running run_robot.m for each controller/task combination.
% It loads the saved .mat files and generates comparison plots.
%
% Expected files (from run_robot.m):
%   real_ct_regulation.mat
%   real_robust_regulation.mat
%   real_adaptive_regulation.mat
%   real_ct_trajectory.mat
%   real_robust_trajectory.mat
%   real_adaptive_trajectory.mat

clc; clear all; close all;

%% Choose which task to compare
TASK = 'regulation';   % change to 'trajectory' for trajectory comparison

%% Load data
ct_file = sprintf('real_ct_%s.mat', TASK);
rb_file = sprintf('real_robust_%s.mat', TASK);
ad_file = sprintf('real_adaptive_%s.mat', TASK);

has_ct = isfile(ct_file);
has_rb = isfile(rb_file);
has_ad = isfile(ad_file);

if has_ct
    ct = load(ct_file);
    t = ct.t;
    fprintf('Loaded: %s\n', ct_file);
end
if has_rb
    rb = load(rb_file);
    t = rb.t;
    fprintf('Loaded: %s\n', rb_file);
end
if has_ad
    ad = load(ad_file);
    t = ad.t;
    fprintf('Loaded: %s\n', ad_file);
end

N = length(t);
joint_labels = {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4'};
fig_title = sprintf('Real Robot - %s Comparison', TASK);

%% Joint Positions
figure('Name', 'Real: Positions');
for i = 1:4
    subplot(2,2,i);
    if has_ct
        plot(t, ct.q_d_all(i,1:N), 'r--', 'LineWidth', 1.5); hold on;
        plot(t, ct.q_real(i,:), 'b', 'LineWidth', 1.2);
    end
    if has_rb, plot(t, rb.q_real(i,:), 'g', 'LineWidth', 1.2); hold on; end
    if has_ad, plot(t, ad.q_real(i,:), 'm', 'LineWidth', 1.2); hold on; end
    xlabel('Time [s]'); ylabel('q_i [rad]');
    title(joint_labels{i}); grid on;
    if i == 1, legend('q_d', 'CT', 'Robust', 'Adaptive', 'Location', 'best'); end
end
sgtitle([fig_title ' - Joint Positions']);

%% Tracking Error Norm
figure('Name', 'Real: Error Norm');
if has_ct
    e_ct = vecnorm(ct.q_d_all(:,1:N) - ct.q_real);
    plot(t, e_ct, 'b', 'LineWidth', 1.5); hold on;
end
if has_rb
    e_rb = vecnorm(rb.q_d_all(:,1:N) - rb.q_real);
    plot(t, e_rb, 'g', 'LineWidth', 1.5); hold on;
end
if has_ad
    e_ad = vecnorm(ad.q_d_all(:,1:N) - ad.q_real);
    plot(t, e_ad, 'm', 'LineWidth', 1.5); hold on;
end
xlabel('Time [s]'); ylabel('||e(t)||_2');
title([fig_title ' - Tracking Error Norm']);
legend('CT', 'Robust', 'Adaptive', 'Location', 'best');
grid on;

%% Control Torques
figure('Name', 'Real: Torques');
for i = 1:4
    subplot(2,2,i);
    if has_ct, plot(t, ct.tau_hist(i,:), 'b', 'LineWidth', 1.2); hold on; end
    if has_rb, plot(t, rb.tau_hist(i,:), 'g', 'LineWidth', 1.2); hold on; end
    if has_ad, plot(t, ad.tau_hist(i,:), 'm', 'LineWidth', 1.2); hold on; end
    xlabel('Time [s]'); ylabel('\tau_i [Nm]');
    title(joint_labels{i}); grid on;
    if i == 1, legend('CT', 'Robust', 'Adaptive', 'Location', 'best'); end
end
sgtitle([fig_title ' - Control Torques']);

fprintf('Done! Comparison plots generated.\n');
