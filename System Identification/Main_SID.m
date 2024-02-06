clear all
close all
clc

%% Settings
test_name = 'MiniE_FlexibleStructure_np2_StabandPerf';
dev_mode = true; %Setting this to true enables developer mode which disables some features making it faster to run the code

%% File paths
addpath lib
plotsFolderPath = fullfile(pwd, 'Plots',test_name);
if ~exist(plotsFolderPath, 'dir')
    mkdir(plotsFolderPath);
end

%% UI Get files
if dev_mode
    fileName.roll = 'FlightLogs\Flight6_RollSIDMan.ulg';
    fileName.pitch = 'FlightLogs\Flight7_PitchSIDMan.ulg';
    fileName.yaw = 'FlightLogs\Flight8_YawSIDMan.ulg';
    fileName.validation = 'FlightLogs\Flight9_Altitude.ulg';
else
    [file,path] = uigetfile('*.ulg', 'Select a Ulg file with Roll SID manouvers');
    fileName.roll = [path file];
    [file,path] = uigetfile('*.ulg', 'Select a Ulg file with Pitch SID manouvers');
    fileName.pitch = [path file];
    [file,path] = uigetfile('*.ulg', 'Select a Ulg file with Yaw SID manouvers');
    fileName.yaw = [path file];
    [file,path] = uigetfile('*.ulg', 'Select a Ulg file with SID validation manouvers');
    fileName.validation = [path file];
end

%% SID Options
%Roll
id_man_options.roll.command_threshold = 0.3;
id_man_options.roll.response_threshold = 0.1;
id_man_options.roll.min_maneuver_duration = 1;
id_man_options.roll.min_response_time = 1;

%Pitch
id_man_options.pitch.command_threshold = 0.3;
id_man_options.pitch.response_threshold = 0.1;
id_man_options.pitch.min_maneuver_duration =  1;
id_man_options.pitch.min_response_time = 1;

%Yaw
id_man_options.yaw.command_threshold = 0.065;
id_man_options.yaw.response_threshold = 0.02;
id_man_options.yaw.min_maneuver_duration =  1;
id_man_options.yaw.min_response_time = 1;
         

%Data selection
inputVariable.Name = 'vehicle_rates_setpoint';
inputVariable.InstanceID = 0;
inputVariable.Field = {'roll','pitch','yaw'};

outputVariable.Name = 'vehicle_angular_velocity';
outputVariable.InstanceID = 0;
outputVariable.Field = 'xyz';

%Filtering
filter.flag = 0; %1 to filter data, 0 to disable filtering of data (currently using data already filtered by PX4) 
filter.order = 3;
filter.framelength = 31;


%% System Identification
axis='roll';
[SID_model_and_control.roll, SID_model.roll, tfestopt_results.roll, valid_flight.roll, params.roll] = PX4_SISO_SID_TF(axis,fileName,inputVariable,outputVariable,filter,id_man_options,plotsFolderPath);
axis='pitch';
[SID_model_and_control.pitch, SID_model.pitch, tfestopt_results.pitch, valid_flight.pitch, params.pitch] = PX4_SISO_SID_TF(axis,fileName,inputVariable,outputVariable,filter,id_man_options,plotsFolderPath);
axis='yaw';
[SID_model_and_control.yaw, SID_model.yaw, tfestopt_results.yaw, valid_flight.yaw, params.yaw] = PX4_SISO_SID_TF(axis,fileName,inputVariable,outputVariable,filter,id_man_options,plotsFolderPath);



%% Saving Results
resultFolderPath = fullfile(pwd, 'Result SID Models');
if ~exist(resultFolderPath, 'dir')
    mkdir(resultFolderPath);
end
save([resultFolderPath,'\',test_name],'SID_model_and_control', 'SID_model', 'tfestopt_results','params', 'valid_flight')

