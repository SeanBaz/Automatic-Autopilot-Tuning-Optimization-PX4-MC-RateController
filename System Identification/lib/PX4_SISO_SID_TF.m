function [SID_model_and_control, SID_model, best_models_table, valid_flight, params] = PX4_SISO_SID_TF(axis,fileName,inputVariable,outputVariable,filter,id_man_options,plotsFolderPath)
%PX4 Single-Input-Single-Output System Identificatio with TF
%   Parse the ULOG flight log extracting only the specified 2 variables
%   (input and output) and performs system identification finding the best
%   TF that fits the data

disp(['***********   SYSTEM IDENTIFICATION FOR ', upper(axis), ' AXIS   ***********'])

%% Pull iddata and params from ULOG
%SID data
[iddata_SID, params]  = getIddata_fromULOG(axis,0,fileName,inputVariable,outputVariable,filter,5);
iddata_SID.Name = 'Angular rate SID';
iddata_SID.InputName = 'Angular rate setpoint';
iddata_SID.InputUnit = 'rad/s';
iddata_SID.OutputName = 'Angular rate';
iddata_SID.OutputUnit = 'rad/s';
%Plot
plotname = ['SID iddata for ',axis,' axis'];
figure('Name',plotname)
plot(iddata_SID)
saveas(gcf, [plotsFolderPath '\' plotname '.png']);
saveas(gcf, [plotsFolderPath '\' plotname '.fig']);

%Validation data
[iddata_validation, ~]  = getIddata_fromULOG(axis,1,fileName,inputVariable,outputVariable,filter,5);
iddata_validation.Name = 'Angular rate Validation';
iddata_validation.InputName = 'Angular rate setpoint';
iddata_validation.InputUnit = 'rad/s';
iddata_validation.OutputName = 'Angular rate';
iddata_validation.OutputUnit = 'rad/s';
%Plot
plotname = ['Validation iddata for ',axis,' axis'];
figure('Name',plotname)
plot(iddata_validation)
saveas(gcf, [plotsFolderPath '\' plotname '.png']);
saveas(gcf, [plotsFolderPath '\' plotname '.fig']);

%% Identify manouvers timeframes
[timeframes] = identify_maneuvers(iddata_SID.SamplingInstants, iddata_SID.InputData, iddata_SID.OutputData, id_man_options.(axis));

%% Data range selection
%SID data
for i=1:length(timeframes)  
    indexStart = find(iddata_SID.SamplingInstants>=timeframes(i,1),1,'first');
    indexEnd = find(iddata_SID.SamplingInstants>=timeframes(i,2),1,'first');
    fragmented_iddata.(['frag',num2str(i)]) = iddata_SID(indexStart:indexEnd);
    %Merge iddata sections
    if i>1
        merged_iddata = merge(merged_iddata,fragmented_iddata.(['frag',num2str(i)]));
    elseif i==1
        merged_iddata = fragmented_iddata.frag1;
    end
end
%Plot
plotname = ['Identified manouvers in the SID iddata for ',axis,' axis'];
figure('Name',plotname)
plot(iddata_SID,':k')
hold on
plot(merged_iddata)
saveas(gcf, [plotsFolderPath '\' plotname '.png']);
saveas(gcf, [plotsFolderPath '\' plotname '.fig']);

%% System Identification of model and rate controller
np_min=1;   %Def 1
np_max=2;   %Def 3
nz_min=0;   %Def 0
n_models=7; %Number of best model to save

f = waitbar(0,'Estimating best Transfer Function for SID model');
maxFit=zeros(n_models,1);
table_varTypes = {'double','logical','uint8','uint8','uint8','double'};
table_varNames = {'Model Fitnes','Single manouvers','Manouver ID','Number of Poles','Number of Zeros','Est Delay'};
best_models_table = table('Size',[10 6],'VariableTypes',table_varTypes,'VariableNames',table_varNames);
for np=np_min:np_max 
    for nz= nz_min:np
        for j=1:2
            for i=0:1 %1-SID for every single experiment, 2-SID for all flight together
                switch i
                    case 0
                        experiments = 1:length(timeframes);
                    case 1
                        experiments = (1:length(timeframes))';
                end

                for k=1:size(experiments,1) 
                    temp_iddata = getexp(merged_iddata, experiments(k,:));
                    delay = [0 delayest(temp_iddata)*iddata_SID.Ts];
                    
                    G_temp = tfest(temp_iddata,np,nz,delay(j));
                    [~,fit_temp,~] = compare(iddata_validation,G_temp);

                    if any(fit_temp>maxFit)
                        model_index = find(fit_temp>maxFit,1,"first");
                        maxFit(model_index) = fit_temp;
                        G{model_index} = G_temp;
                        best_models_table(model_index,:) = {fit_temp, i, k, np, nz, delay(j)};
                    end
                end
            end 
        end                
    end
    waitbar(np-np_min/length(np_min:np_max),f,'Estimating best Transfer Function for SID model');
end 
close(f)

%Print resutl
disp(['Max fit on out-of-sample data is:' num2str(maxFit(1)) '%'])
best_models_table
%Plot
plotname = ['Fit of best ' num2str(n_models) ' models on validation data for ',axis,' axis'];
figure('Name',plotname)
compare(iddata_validation,G{1:end})
leg_string{1} = 'Real Flight Data';
for i=1:n_models
    leg_string{i+1} = ['Model ' num2str(i)];
end
legend(leg_string)
saveas(gcf, [plotsFolderPath '\' plotname '.png']);
saveas(gcf, [plotsFolderPath '\' plotname '.fig']);

%% TF calculation of SID model only BY REMOVING PID CONTROLLER
%Variables
s=tf('s');
Derivative=s/(1e10*s+1);

K=params.(['MC_' upper(axis) 'RATE_K']);
P=params.(['MC_' upper(axis) 'RATE_P']);
I=params.(['MC_' upper(axis) 'RATE_I']);
D=params.(['MC_' upper(axis) 'RATE_D']);
DGyro_lpf_cutoff=params.IMU_DGYRO_CUTOFF; 

if DGyro_lpf_cutoff>0
    LPF=1/(1+s/(2*pi*DGyro_lpf_cutoff));
else
    LPF=1;
end

%A{i}=-G{i}/(G{i}*(K*D*Derivative*LPF+K*P+K*I/s)-K*P-K*I/s);
for i=1:length(G)
    %Need to remove the delay and add it later otherwise will convert it to a SS (no issue with that but hard to compare)
    tf_temp = tf(G{i}.Numerator,G{i}.Denominator);
    A{i}=-tf_temp/(tf_temp*(K*D*Derivative*LPF+K*P+K*I/s)-K*P-K*I/s);
    A{i}.IODelay = G{i}.IODelay;
end


%% Packaging data for simulation
%Saving model 1 to be used for simulations and tuning
SID_model_and_control = G(1);
SID_model = A(1);
%Saving the validation data as timeseries for simulink
validSamplingInstant_0 = iddata_validation.SamplingInstants-iddata_validation.SamplingInstants(1);
valid_flight.ang_setpoint=timeseries(iddata_validation.u, validSamplingInstant_0);
valid_flight.ang_rate=timeseries(iddata_validation.y, validSamplingInstant_0);

end