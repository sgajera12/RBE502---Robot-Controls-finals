%% record_waypoints.m - Record robot waypoints by manual movement
% 
% HOW TO USE:
%   1. Run this script
%   2. The robot motors will be disabled (you can move it freely by hand)
%   3. Move the robot to a position you want
%   4. Press ENTER to record that position as a waypoint
%   5. Repeat for as many waypoints as you want
%   6. Type 'done' and press ENTER to finish
%   7. The waypoints are saved to 'recorded_waypoints.mat'
%
% After recording, run generate_trajectory.m to create a smooth trajectory.
clc; clear all; close all;

addpath("Communication_Code");
addpath("generated_dynamics");

factor_degre_to_rad = pi/180;

%% Connect
fprintf('Connecting to robot...\n');
robot = Robot();

%% Disable motors so you can move the robot by hand
robot.writeMode('c');
robot.writeCurrents([0, 0, 0, 0]);   % zero current = free to move

fprintf('\n=== WAYPOINT RECORDER ===\n');
fprintf('Motors are OFF. Move the robot freely by hand.\n');
fprintf('Press ENTER to record current position as a waypoint.\n');
fprintf('Type "done" and press ENTER when finished.\n\n');

waypoints = [];
wp_count = 0;

while true
    user_input = input(sprintf('Waypoint %d - Move robot, then press ENTER (or type "done"): ', wp_count + 1), 's');
    
    if strcmpi(strtrim(user_input), 'done')
        break;
    end
    
    % read current position
    readings = robot.getJointsReadings();
    q_now = readings(1, :) * factor_degre_to_rad;
    
    wp_count = wp_count + 1;
    waypoints(wp_count, :) = q_now;
    
    fprintf('  Recorded: [%.3f, %.3f, %.3f, %.3f] rad\n\n', ...
        q_now(1), q_now(2), q_now(3), q_now(4));
end

%% Add the first waypoint at the end to make a loop (optional)
fprintf('\nMake the trajectory loop back to start? (y/n): ');
loop_input = input('', 's');
if strcmpi(strtrim(loop_input), 'y')
    waypoints(end+1, :) = waypoints(1, :);
    fprintf('Added first waypoint at the end for looping.\n');
end

%% Save
save('recorded_waypoints.mat', 'waypoints');
fprintf('\nSaved %d waypoints to recorded_waypoints.mat\n', size(waypoints, 1));

%% Display
fprintf('\nRecorded waypoints:\n');
for i = 1:size(waypoints, 1)
    fprintf('  WP %d: [%.3f, %.3f, %.3f, %.3f] rad\n', i, waypoints(i,:));
end

%% Release robot
robot.writeCurrents([0, 0, 0, 0]);
fprintf('\nDone! Now run generate_trajectory.m to create a smooth trajectory.\n');
