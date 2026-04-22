%% Add to Path Script

% Absolute path to the libraries on your system
dynamixel_library_path = '/home/pinaka/rbe502_final/DynamixelSDK'
%
% Add necessary folders and subfolders from the Dynamixel Library
addpath(genpath(dynamixel_library_path + "/c/include"));
addpath(dynamixel_library_path + "/c/build/linux64");
addpath(genpath(dynamixel_library_path + "/matlab"));
