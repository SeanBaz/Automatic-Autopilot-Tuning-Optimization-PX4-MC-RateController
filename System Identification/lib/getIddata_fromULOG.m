function [iddata_out, params] = getIddata_fromULOG(axis,validation,fileName,inputVariable,outputVariable,filter,time_edge_cut)
%Get IDDATA from PX4 ULOG 
%   Load specified ulog file, parse 2 variables (input and output) then
%   manipulate data sampling to match, apply filter if required and then
%   return the iddata

%% Axis selection
switch axis
    case 'roll'
        axis_num = 1;
    case 'pitch'
        axis_num = 2;
    case 'yaw'
        axis_num = 3;
    otherwise
        axis_num = 0;
end

%% Load log file
if validation
    ulog = ulogreader(fileName.validation);
else
    ulog = ulogreader(fileName.(axis));
end


%% Data extraction
%Extract input data
inputData_struct = readTopicMsgs(ulog,'TopicNames',{inputVariable.Name}, 'InstanceID',{inputVariable.InstanceID});
inputData_timestamp = seconds(inputData_struct.TopicMessages{1,1}.timestamp);
inputData_allaxes = inputData_struct.TopicMessages{1,1}.(inputVariable.Field{axis_num});
inputData_raw = inputData_allaxes;

%Extract outputdata
outputData_struct = readTopicMsgs(ulog,'TopicNames',{outputVariable.Name}, 'InstanceID',{outputVariable.InstanceID});
outputData_timestamp = seconds(outputData_struct.TopicMessages{1,1}.timestamp);
outputData_allaxes = outputData_struct.TopicMessages{1,1}.(outputVariable.Field);
outputData_raw = outputData_allaxes(:,axis_num);

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
time_start = max([inputData_timestamp(1,1),outputData_timestamp(1,1),IsInGroundEffect_timestamp(find(~IsInGroundEffect_signal,1,"first"))]);
time_end = min([inputData_timestamp(end,1),outputData_timestamp(end,1),IsInGroundEffect_timestamp(find(~IsInGroundEffect_signal,1,"last"))]);
input_timestep = mean(diff(inputData_timestamp));
output_timestep = mean(diff(outputData_timestamp));
timestep = min([input_timestep output_timestep]);  %Use smallest timestep
%Trimming out takeoff and landing based on time_edge_cut
timestamp = (time_start+time_edge_cut:timestep:time_end-time_edge_cut)';

%Interp (upsampling)
inputData_raw = double(inputData_raw);
inputData = interp1(inputData_timestamp,inputData_raw,timestamp);
outputData = interp1(outputData_timestamp,outputData_raw,timestamp);

%Filtering 
if filter.flag==1
    inputData = sgolayfilt(inputData,filter.order,filter.framelength);
    outputData = sgolayfilt(outputData,filter.order,filter.framelength);
end

%IDDATA
iddata_out = iddata(outputData,inputData,timestep);

end

