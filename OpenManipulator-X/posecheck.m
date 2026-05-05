%% read_joints.m - Live joint position display
clc; clear all; close all;
addpath("Communication_Code");

robot = Robot();
robot.writeMode('c');
robot.writeCurrents([0,0,0,0]);   % zero current = free to move

deg2rad = pi/180;

fprintf('Motors OFF - move robot freely\n');
fprintf('Press Ctrl+C to stop\n\n');

while true
    readings = robot.getJointsReadings();
    q = readings(1,:) * deg2rad;
    fprintf('q = [%+.3f, %+.3f, %+.3f, %+.3f] rad\r', q(1), q(2), q(3), q(4));
    pause(0.1);
end