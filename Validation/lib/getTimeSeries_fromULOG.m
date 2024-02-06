function [FlightData, params, timestep] = getTimeSeries_fromULOG(fileName)
%Get IDDATA from PX4 ULOG 
%   Load specified ulog file, parse 2 variables (input and output) then
%   manipulate data sampling to match, apply filter if required and then
%   return the iddata

time_edge_cut = 5;

%% Load log file
ulog = ulogreader(fileName);

%% Data extraction
%Angular rate setpoint
vehicle_rates_setpoint_struct = readTopicMsgs(ulog,'TopicNames',{'vehicle_rates_setpoint'}, 'InstanceID',{0});
raw.vehicle_rates_setpoint.timestamp = seconds(vehicle_rates_setpoint_struct.TopicMessages{1,1}.timestamp);
vehicle_rates_setpoint_timestep = mode(diff(raw.vehicle_rates_setpoint.timestamp));
raw.vehicle_rates_setpoint.roll = double(vehicle_rates_setpoint_struct.TopicMessages{1,1}.roll);
raw.vehicle_rates_setpoint.pitch = double(vehicle_rates_setpoint_struct.TopicMessages{1,1}.pitch);
raw.vehicle_rates_setpoint.yaw = double(vehicle_rates_setpoint_struct.TopicMessages{1,1}.yaw);

%Angular rate
vehicle_rates_struct = readTopicMsgs(ulog,'TopicNames',{'vehicle_angular_velocity'}, 'InstanceID',{0});
raw.vehicle_rates.timestamp = seconds(vehicle_rates_struct.TopicMessages{1,1}.timestamp);
vehicle_rates_timestep = mode(diff(raw.vehicle_rates.timestamp));
raw.vehicle_rates.roll = vehicle_rates_struct.TopicMessages{1,1}.xyz(:,1);
raw.vehicle_rates.pitch = vehicle_rates_struct.TopicMessages{1,1}.xyz(:,2);
raw.vehicle_rates.yaw = vehicle_rates_struct.TopicMessages{1,1}.xyz(:,3);

%Commands
actuator_controls_struct = readTopicMsgs(ulog,'TopicNames',{'actuator_controls_0'}, 'InstanceID',{0});
raw.actuator_controls.timestamp = seconds(actuator_controls_struct.TopicMessages{1,1}.timestamp_sample);
actuator_controls_timestep = mode(diff(raw.actuator_controls.timestamp));
raw.actuator_controls.roll = actuator_controls_struct.TopicMessages{1,1}.control(:,1);
raw.actuator_controls.pitch = actuator_controls_struct.TopicMessages{1,1}.control(:,2);
raw.actuator_controls.yaw = actuator_controls_struct.TopicMessages{1,1}.control(:,3);

%Extract ground effect signal to trim takoff and landing
IsInGroundEffect_struct = readTopicMsgs(ulog,'TopicNames',{'vehicle_land_detected'}, 'InstanceID',{0});
IsInGroundEffect_timestamp = seconds(IsInGroundEffect_struct.TopicMessages{1,1}.timestamp);
IsInGroundEffect_signal = IsInGroundEffect_struct.TopicMessages{1,1}.in_ground_effect;

%Extract gains 
params_table = readParameters(ulog);
for i=1:size(params_table,1)
    params.(params_table{i,1}) = params_table{i,2};
end

%% Data manipulation
%Universal time sampling
time_start = max([raw.vehicle_rates_setpoint.timestamp(1,1),raw.vehicle_rates.timestamp(1,1),raw.actuator_controls.timestamp(1,1),IsInGroundEffect_timestamp(find(~IsInGroundEffect_signal,1,"first"))]);
time_end = min([raw.vehicle_rates_setpoint.timestamp(end,1),raw.vehicle_rates.timestamp(end,1),raw.actuator_controls.timestamp(end,1),IsInGroundEffect_timestamp(find(~IsInGroundEffect_signal,1,"last"))]);
timestep = min([vehicle_rates_setpoint_timestep vehicle_rates_timestep actuator_controls_timestep]);  %Use smallest timestep
%Trimming out takeoff and landing based on time_edge_cut
timestamp = (time_start+time_edge_cut:timestep:time_end-time_edge_cut)';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%CHECK IF WORKING
%Interp (upsampling)
topic_fields = fields(raw);
for i=1:length(topic_fields)
    axis = fields(raw.(topic_fields{i}));
    %Delete NaN data points
    nan_indx = find(isnan(raw.(topic_fields{i}).roll));
    raw.(topic_fields{i}).timestamp(nan_indx)=[];
    for j=1:length(axis)
        if ~strcmp(axis{j},'timestamp')
            raw.(topic_fields{i}).(axis{j})(nan_indx)=[];
            interpData.(topic_fields{i}).(axis{j}) = interp1(raw.(topic_fields{i}).timestamp,raw.(topic_fields{i}).(axis{j}),timestamp);
        end
    end
    clear nan_indx
end


%TimeSerie
timestamp_zero = timestamp-timestamp(1);

topic_fields = fields(interpData);
for i=1:length(topic_fields)
    axis = fields(interpData.(topic_fields{i}));
    for j=1:length(axis)
        FlightData.(topic_fields{i}).(axis{j}) = timeseries(interpData.(topic_fields{i}).(axis{j}),timestamp_zero);
    end
end

end

