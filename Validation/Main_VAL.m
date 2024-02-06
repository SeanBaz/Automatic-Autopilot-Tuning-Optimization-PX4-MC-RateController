clear all 
close all
clc

%% OPTIONS
ValidationName = 'MiniE_flexibleStructure_v4';
dev_mode = true;        %Setting this to true enables developer mode which disables some features making it faster to run the code
flag_noise = 1;         %0-to disable noise, 1-to enable noise
flag_NoiseLvl= 1;       %0-correct sensor noise levels for each axis, 1-highest level of sensor noise, 2-increase sensor noise x5 

%Noise measured as variance from sensor reading in trim conditions
if flag_NoiseLvl == 1 %Highest gyro noise
    noise_amplitude = 3e-5*ones(1,3);
elseif flag_NoiseLvl == 2 %Super noise
    noise_amplitude = [3e-5 0.7e-5 0.8e-5].*5;
else %Normal noise 
    noise_amplitude = [3e-5 0.7e-5 0.8e-5];
end

%Step value for performance metric calculations
step_value= [0.7  0.8 0.6];     % [Roll, Pitch, Yaw] respectively

%% File Paths
addpath lib
plotsFolderPath = fullfile(pwd, 'Plots',ValidationName);
if ~exist(plotsFolderPath, 'dir')
    mkdir(plotsFolderPath);
end

%% LOAD SID MODEL, OPTIMIZATION RESULTS AND FLIGHT DATA
if dev_mode
    SID_filePath = [fileparts(pwd) '\System Identification\Result SID Models\MiniE_FlexibleStructure_np2.mat'];
    SID = load(SID_filePath);
    OPT_filePath = [fileparts(pwd) '\Tuning Optimization\Results\MiniE_flexibleStructure_v14_12HzMaxDGYRO_deep.mat'];
    OPT = load(OPT_filePath);
    SID_Flight_fileName = [fileparts(pwd) '\System Identification\FlightLogs\Flight9_Altitude.ulg'];
    [SID_valid_FlightData, SID_FlightParams, SID_timestep] = getTimeSeries_fromULOG(SID_Flight_fileName);
    OPT_Flight_fileName = [fileparts(pwd) '\Validation\Flight Logs\MiniE5050A_Flight15_Outdoor_MC_TuningValidation_OptimizationV7_12HzFIlter.ulg'];
    [OPT_FlightData, OPT_FlightParams, OPT_timestep] = getTimeSeries_fromULOG(OPT_Flight_fileName);
else
    [SID_file,SID_path] = uigetfile('*.mat', 'Select a SID model mat file');
    SID = load([SID_path SID_file]);
    [OPT_file,OPT_path] = uigetfile('*.mat', 'Select a mat file with the optimized set of gains');
    OPT = load([OPT_path OPT_file]);
    SID_Flight_fileName = uigetfile('*.ulg', 'Select a SID flight for validation');
    [SID_valid_FlightData, SID_FlightParams, SID_timestep] = getTimeSeries_fromULOG(SID_Flight_fileName);
    OPT_Flight_fileName = uigetfile('*.ulg', 'Select a flight with optimized gains for validation');
    [OPT_FlightData, OPT_FlightParams, OPT_timestep] = getTimeSeries_fromULOG(OPT_Flight_fileName);
end

%% Variable initialization
axis = {'roll', 'pitch', 'yaw'};

%% Validation
for i=1:3 %Repeat Validation script for all axis
    disp(' ')
    disp(['***********   VALIDATION FOR ', upper(axis{i}), ' AXIS   ***********'])
    %% SID Validation
    %Set coefficients
    K=SID.params.(axis{i}).(['MC_' upper(axis{i}) 'RATE_K']);
    P=SID.params.(axis{i}).(['MC_' upper(axis{i}) 'RATE_P']);
    I=SID.params.(axis{i}).(['MC_' upper(axis{i}) 'RATE_I']);
    D=SID.params.(axis{i}).(['MC_' upper(axis{i}) 'RATE_D']);
    windup_limit = SID.params.(axis{i}).(['MC_' upper(axis{i}(1)) 'R_INT_LIM']); 

    %Feed forward (not used)
    FF=0;

    %Original filter values
    gyro_cutoff = SID.params.(axis{i}).IMU_GYRO_CUTOFF;
    dgyro_cutoff = SID.params.(axis{i}).IMU_DGYRO_CUTOFF;
    
    %Variable selection for axis
    G_temp=SID.SID_model_and_control.(axis{i}){1};
    A_temp=SID.SID_model.(axis{i}){1};
    
    %Set flight data
    RealFlightData.AngularRate_setpoints = SID_valid_FlightData.vehicle_rates_setpoint.(axis{i});
    RealFlightData.AngularRate_response = SID_valid_FlightData.vehicle_rates.(axis{i});
    RealFlightData.command = SID_valid_FlightData.actuator_controls.(axis{i});
    overwriteSensorFeedback = false;
    useRealFlightData = true;

    %Set stop time of the simulation
    timestep = SID_timestep;
    StartTime = 0;
    StopTime = SID_valid_FlightData.vehicle_rates_setpoint.(axis{i}).Time(end);
    
    %Simulation 
    sim('lib\PX4_angRate_controller_Validation.slx');
    
    %Fit calculation
    [~,R2_isolSIDvsSID,~] = fitnessCalc(Validation_Results.Data(:,4),Validation_Results.Data(:,2));
    [~,R2_SIDvsReal,~] = fitnessCalc(Validation_Results.Data(:,5),Validation_Results.Data(:,2));
    [~,R2_SIDnoisevsReal,~] = fitnessCalc(Validation_Results.Data(:,5),Validation_Results.Data(:,3));
    [~,R2_SIDCommandvsReal,~] = fitnessCalc(Validation_Results.Data(:,7),Validation_Results.Data(:,6));
    disp(['The goodness of fit of the SID isolated model compared to the model+controller is R2=' num2str(R2_isolSIDvsSID) '%'])
    disp(['The goodness of fit of the SID model compared to real flight data is R2=' num2str(R2_SIDvsReal) '%'])
    disp(['The goodness of fit of the SID model with noise compared to real flight data is R2=' num2str(R2_SIDnoisevsReal) '%'])
    disp(['The goodness of fit of the SID command signal compared to real flight command is R2=' num2str(R2_SIDCommandvsReal) '%'])
    
    
    %Plotting
    %1
    plotname = ['Comparison of SID model&controller vs isolated SID FDM and reconstructed controller for ',axis{i},'-rate'];
    figure('Name',plotname)
    plot(Validation_Results.Time,Validation_Results.Data(:,1),'Color','#D95319','LineWidth',1)
    hold on
    plot(Validation_Results.Time,Validation_Results.Data(:,4),'LineWidth',1.5,'Color','#EDB120')
    plot(Validation_Results.Time,Validation_Results.Data(:,2),'LineWidth',1.5,'Color','#0072BD')
    ylabel('Angular-rate (rad/s)');
    xlabel('Time (Seconds)');
    title(plotname);
    legend('Setpoint', 'SID model+controller response',['Isolated FDM model + reconstructed controller response, fit=' num2str(R2_isolSIDvsSID) '%'])
    grid on 
    grid minor
    saveas(gcf, [plotsFolderPath '\' plotname '.png']);
    saveas(gcf, [plotsFolderPath '\' plotname '.fig']);

    %2
    plotname = ['Comparison of SID model vs real flight data for ',axis{i},'-rate'];
    figure('Name',plotname)
    sub1 = subplot(2,1,1);
    plot(Validation_Results.Time,Validation_Results.Data(:,1),'Color','#D95319','LineWidth',1)
    hold on
    plot(Validation_Results.Time,Validation_Results.Data(:,5),'--k','LineWidth',1)
    plot(Validation_Results.Time,Validation_Results.Data(:,2),'LineWidth',1.5,'Color','#0072BD')
    plot(Validation_Results.Time,Validation_Results.Data(:,3),'LineWidth',1.5,'Color','#77AC30')
    ylabel('Angular-rate (rad/s)');
    xlabel('Time (Seconds)');
    title(plotname);
    legend('Setpoint','Real vehicle response', ['SID model response, fit=' num2str(R2_SIDvsReal) '%'],['SID model response with noise, fit=' num2str(R2_SIDnoisevsReal) '%'])
    grid on 
    grid minor

    sub2 = subplot(2,1,2);
    plot(Validation_Results.Time,Validation_Results.Data(:,7),'--k','LineWidth',1)
    hold on
    plot(Validation_Results.Time,Validation_Results.Data(:,6)-mean(Validation_Results.Data(:,6))+mean(Validation_Results.Data(:,7)),'Color','#EDB120','LineWidth',1)
    ylabel('Actuator command');
    xlabel('Time (Seconds)');
    legend('Real flight command signal','Simulated command signal')
    grid on 
    grid minor
    linkaxes([sub1 sub2],'x');
    saveas(gcf, [plotsFolderPath '\' plotname '.png']);
    saveas(gcf, [plotsFolderPath '\' plotname '.fig']);
    
    %Perfomance and stability metrics
    clear useRealFlightData StopTime Validation_Results
    useRealFlightData = false;
    StopTime = 5;
    sim('lib\PX4_angRate_controller_Validation.slx');
    Initial_performanceMetrics = evaluateControllerPerformance(Validation_Results);
    disp('The performance metrics for the initial tuning are:')
    disp(Initial_performanceMetrics)

    %% Control Matching 
    clear Validation_Results overwriteSensorFeedback StartTime StopTime useRealFlightData
    overwriteSensorFeedback = true;
    useRealFlightData = true;
    %If using a new flight section use the entire flight
    if ~dev_mode
        StartTime = 0;
        StopTime = SID_valid_FlightData.vehicle_rates_setpoint.(axis{i}).Time(end);
    else %If using the flight data provided in the paper we need to select a flight section without dropouts otherwise there will be an unrealistic integral reset in the command
        switch i
            case 1
                StartTime = 110;
                StopTime = 165;
            case 2
                StartTime = 104;
                StopTime = 165;
            case 3
                StartTime = 150;
                StopTime = 250;
        end
    end
    sim('lib\PX4_angRate_controller_Validation.slx');
    
    [~,R2_SimCommandvsReal,~] = fitnessCalc(Validation_Results.Data(:,7),Validation_Results.Data(:,6));
    disp(['The goodness of fit of the Simulated command signal compared to real flight command is R2=' num2str(R2_SimCommandvsReal) '%'])
    
    %Plotting
    plotname = ['Comparison of real flight actuator command vs simulated controller output for ',axis{i},' axis'];
    figure('Name',plotname)
    plot(Validation_Results.Time,Validation_Results.Data(:,7),'--k','LineWidth',1)
    hold on
    %Subtracting the difference of the mean to account for the integrator initialization, better way to do it would be to initialize the unitdelay with the integrator value
    plot(Validation_Results.Time,Validation_Results.Data(:,6)-mean(Validation_Results.Data(:,6))+mean(Validation_Results.Data(:,7)),'Color','#EDB120','LineWidth',1)
    ylabel('Actuator command');
    xlabel('Time (Seconds)');
    title(plotname);
    legend('Real flight command signal',['Simulated command signal, fit=' num2str(R2_SimCommandvsReal) '%'])
    grid on 
    grid minor
    saveas(gcf, [plotsFolderPath '\' plotname '.png']);
    saveas(gcf, [plotsFolderPath '\' plotname '.fig']);
    

    %% OPT VALIDATION
    clear K P I D windup_limit FF gyro_cutoff dgyro_cutoff RealFlightData Validation_Results timestep StartTime StopTime overwriteSensorFeedback
    %Set coefficients from optimized flight test (Use this)
    K=OPT_FlightParams.(['MC_' upper(axis{i}) 'RATE_K']);
    P=OPT_FlightParams.(['MC_' upper(axis{i}) 'RATE_P']);
    I=OPT_FlightParams.(['MC_' upper(axis{i}) 'RATE_I']);
    D=OPT_FlightParams.(['MC_' upper(axis{i}) 'RATE_D']);
    windup_limit = OPT_FlightParams.(['MC_' upper(axis{i}(1)) 'R_INT_LIM']); 
    
    %Set coefficients from optimization results (For testing)
    % P = OPT.opt_gains.(axis{i})(1);
    % I = OPT.opt_gains.(axis{i})(2);
    % D = OPT.opt_gains.(axis{i})(3);

    %Feed forward (not used)
    FF=0;

    %Original filter values
    gyro_cutoff = OPT_FlightParams.IMU_GYRO_CUTOFF;
    dgyro_cutoff = OPT_FlightParams.IMU_DGYRO_CUTOFF;
    
    %Set flight data
    RealFlightData.AngularRate_setpoints = OPT_FlightData.vehicle_rates_setpoint.(axis{i});
    RealFlightData.AngularRate_response = OPT_FlightData.vehicle_rates.(axis{i});
    RealFlightData.command = OPT_FlightData.actuator_controls.(axis{i});
    overwriteSensorFeedback = false;
    useRealFlightData = true;

    %Set stop time of the simulation
    timestep = OPT_timestep;
    StartTime = 0;
    StopTime = OPT_FlightData.vehicle_rates_setpoint.(axis{i}).Time(end);
    
    %Simulation 
    sim('lib\PX4_angRate_controller_Validation.slx');
    
    %Fit calculation
    [~,R2_OPTvsRealOPT,~] = fitnessCalc(Validation_Results.Data(:,5),Validation_Results.Data(:,2));
    [~,R2_OPTnoisevsRealOPT,~] = fitnessCalc(Validation_Results.Data(:,5),Validation_Results.Data(:,3));
    [~,R2_OptSimCommandvsReal,~] = fitnessCalc(Validation_Results.Data(:,7),Validation_Results.Data(:,6));
    disp(['The goodness of fit of the simulated optimized controller compared to real flight data with optimized parameters is R2=' num2str(R2_OPTvsRealOPT) '%'])
    disp(['The goodness of fit of the simulated optimized controller with noise compared to real flight data is R2=' num2str(R2_OPTnoisevsRealOPT) '%'])
    disp(['The goodness of fit of the simulated optimized command signal compared to real flight command is R2=' num2str(R2_OptSimCommandvsReal) '%'])
    
    %Plotting
    plotname = ['Comparison of simulated optimized response vs real flight data response with optimized parameters for ',axis{i},'-rate'];
    figure('Name',plotname)
    sub3 = subplot(2,1,1);
    plot(Validation_Results.Time,Validation_Results.Data(:,1),'Color','#D95319','LineWidth',1)
    hold on
    plot(Validation_Results.Time,Validation_Results.Data(:,5),':k','LineWidth',1)
    plot(Validation_Results.Time,Validation_Results.Data(:,4),'--','LineWidth',1,'Color','#4DBEEE')
    plot(Validation_Results.Time,Validation_Results.Data(:,2),'LineWidth',1.5,'Color','#0072BD')
    plot(Validation_Results.Time,Validation_Results.Data(:,3),'LineWidth',1.5,'Color','#77AC30')
    ylabel('Angular-rate (rad/s)');
    xlabel('Time (Seconds)');
    title(plotname);


    legend('Setpoint','Real vehicle response with optimized parameters','Response with initial parameters (unoptimized)', ['Simulated optimized model response, fit=' num2str(R2_OPTvsRealOPT) '%'],['Simulated optimized model response with noise, fit=' num2str(R2_OPTnoisevsRealOPT) '%'])
    grid on 
    grid minor

    sub4 = subplot(2,1,2);
    plot(Validation_Results.Time,Validation_Results.Data(:,7)-mean(Validation_Results.Data(:,7)),'--k','LineWidth',1)
    hold on
    plot(Validation_Results.Time,Validation_Results.Data(:,6),'Color','#EDB120','LineWidth',1)
    ylabel('Actuator command');
    xlabel('Time (Seconds)');
    legend('Real flight command signal',['Simulated command signal, fit=' num2str(R2_OptSimCommandvsReal) '%'])
    grid on 
    grid minor
    linkaxes([sub3 sub4],'x');
    saveas(gcf, [plotsFolderPath '\' plotname '.png']);
    saveas(gcf, [plotsFolderPath '\' plotname '.fig']);
    

    %Perfomance and stability metrics
    clear useRealFlightData StopTime Validation_Results
    useRealFlightData = false;
    StopTime = 5;
    sim('lib\PX4_angRate_controller_Validation.slx');
    Optimized_performanceMetrics = evaluateControllerPerformance(Validation_Results);
    disp('The performance metrics for the initial tuning are:')
    disp(Optimized_performanceMetrics)
    
    %Open Loop Stability
    % figure('Name',['Open loop stability margins for ' axis{i} ' axis'])
    % margin(A_temp)
    % saveas(gcf, [plotsFolderPath '\OpenLoopMargins.png']);
    % saveas(gcf, [plotsFolderPath '\OpenLoopMargins.fig']);

    %Initial controller Stability
    % figure('Name',['Initial controller closed loop stability margins ' axis{i} ' axis'])
    % margin(G_temp)
    % saveas(gcf, [plotsFolderPath '\ClosedLoopMargins.png']);
    % saveas(gcf, [plotsFolderPath '\ClosedLoopMargins.fig']);

    clear K P I D windup_limit FF gyro_cutoff dgyro_cutoff RealFlightData Validation_Results timestep StartTime StopTime overwriteSensorFeedback
end

