clear all 
close all
clc

%% OPTIONS
optimizationName = 'MiniE_flexibleStructure_v14_12HzMaxDGYRO_deep';
dev_mode = true;        %Setting this to true enables developer mode which disables some features making it faster to run the code
flag_optfilter = 0;     %1-Optimize DGYRO low pass filter, 0-No filter optimization of DGYRO
flag_noise = 1;         %0-to disable noise, 1-to enable noise
flag_NoiseLvl= 1;       %0-correct sensor noise levels for each axis, 1-highest level of sensor noise, 2-increase sensor noise x5 


%OPTIMIZATION SETTINGS
gaopt.PopulationSize = 200;                      %Size of the population.
gaopt.MaxGenerations = 4*gaopt.PopulationSize;  %Maximum number of iterations before the algorithm halts {100*population size}
gaopt.MaxTime = inf;                          %The algorithm stops after running for MaxTime seconds {inf}
gaopt.MaxStallTime = inf;                       %The algorithm stops if there is no improvement in the objective function for MaxStallTime seconds {inf}
gaopt.FunctionTolerance = 1e-6;                 %The algorithm stops if the average relative change in the best fitness function value over MaxStallGenerations generations is less than or equal to FunctionTolerance {1e-6}
gaopt.MaxStallGenerations = 50;                 %The algorithm stops if the average relative change in the best fitness function value over MaxStallGenerations generations is less than or equal to FunctionTolerance.  {50}


%% Simulation settings
IMU_GYRO_RATE = 400;    %Hz at which the inner loop is running
%Command input settings
step_value= [0.7  0.8 0.6];     % [Roll, Pitch, Yaw] respectively
StopTime_list = [2.5 2.5 5];    % [Roll, Pitch, Yaw] respectively

%Noise measured as variance from sensor reading in trim conditions
if flag_NoiseLvl == 1 %Highest gyro noise
    noise_amplitude = 3e-5*ones(1,3);
elseif flag_NoiseLvl == 2 %Super noise
    noise_amplitude = [3e-5 0.7e-5 0.8e-5].*5;
else %Normal noise 
    noise_amplitude = [3e-5 0.7e-5 0.8e-5];
end

%Set Hz for the DGYRO second-order LPF if not being optimized
dgyro_cutoff_hardcode = 12; %Setting it to 0 disable the filter

%Cost function weights for [Roll, Pitch, Yaw]
Q_list=[1.2 1.2 2]; 
R_list=[2 2 2];
C_list=[0.002 0.0016 0.002]; 


%% File Paths
addpath lib
plotsFolderPath = fullfile(pwd, 'Plots',optimizationName);
if ~exist(plotsFolderPath, 'dir')
    mkdir(plotsFolderPath);
end

%% LOAD SID MODEL DATA
if dev_mode
    SID_filePath = [fileparts(pwd) '\System Identification\Result SID Models\MiniE_FlexibleStructure_np2.mat'];
    load(SID_filePath)
else
    [SID_file,SID_path] = uigetfile('*.mat', 'Select a SID model mat file');
    load([SID_path SID_file]);
end

%% Variable initialization
axis = {'roll', 'pitch', 'yaw'};
fun = @GA_tuning_function;

%Parameter resolution [Kp ki kd]
gain_resolution = [1/0.01 1/0.01 1/0.0005];

%Parameters limit [Kp ki kd]
lb.roll = [0.01, 0, 0];
ub.roll = [2, 2, 1];

lb.pitch = [0.01, 0, 0];
ub.pitch = [5, 5, 2];

lb.yaw = [0, 0, 0];
ub.yaw = [5, 5, 2];

%If optimizing the LPF filter at the same time, add resolution and limits
if flag_optfilter
    gain_resolution = [gain_resolution 1];
    
    lb.roll = [lb.roll 12];
    ub.roll = [ub.roll 120];
    lb.pitch = [lb.pitch 12];
    ub.pitch = [ub.pitch 120];
    lb.yaw = [lb.yaw 12];
    ub.yaw = [ub.yaw 120];
end

%% Optimization
for i=1:3 %Repeat optimization for all axis
    disp(' ')
    disp(['***********   TUNING OPTIMIZATION FOR ', upper(axis{i}), ' AXIS   ***********'])

    %Set coefficients
    K_init=params.(axis{i}).(['MC_' upper(axis{i}) 'RATE_K']);
    P_init=params.(axis{i}).(['MC_' upper(axis{i}) 'RATE_P']);
    I_init=params.(axis{i}).(['MC_' upper(axis{i}) 'RATE_I']);
    D_init=params.(axis{i}).(['MC_' upper(axis{i}) 'RATE_D']);
    windup_limit = params.(axis{i}).(['MC_' upper(axis{i}(1)) 'R_INT_LIM']); 
    
    %PID scaler standard form (not used)
    K=K_init;
    
    %Feed forward (not used)
    FF=0;
    %Feed forward initial guess from dynamic inversion
    %[OL_step_response,~] = step(A_temp);
    %FF=1/max(OL_step_response);

    %Original filter values
    gyro_cutoff = params.(axis{i}).IMU_GYRO_CUTOFF;
    dgyro_cutoff_init = params.(axis{i}).IMU_DGYRO_CUTOFF;
    
    %Variable selection for axis
    G_temp=SID_model_and_control.(axis{i}){1};
    A_temp=SID_model.(axis{i}){1};
    Q = Q_list(i);
    R = R_list(i);
    C = C_list(i);
    StopTime = StopTime_list(i);

    %Set initial parameters for optimization 
    clear Initialparam
    Initialparam = [P_init I_init D_init];
    if flag_optfilter
        Initialparam(4) = dgyro_cutoff_init;
    else
        dgyro_cutoff = dgyro_cutoff_hardcode;%dgyro_cutoff_init; %Fix DGYRO value for optimization
    end
    Initialparam = Initialparam.*gain_resolution;

    %Get cost function value from the initial parameters (SID flight gains)
    InitialCostScore.(axis{i}) = GA_tuning_function([Initialparam]);
    disp(['The cost value with the initial tuning gains is: ' num2str(InitialCostScore.(axis{i}))])
    
    %Optimization
    lb_temp = lb.(axis{i}).*gain_resolution;
    ub_temp = ub.(axis{i}).*gain_resolution;
    Initialparam_const = max(min(ub_temp,Initialparam),lb_temp);
    IntCon = 1:length(Initialparam_const);
    ga_options = optimoptions('ga','PlotFcn',{@gaplotgenealogy, @gaplotbestf, @gaplotbestindiv},...
        'InitialPopulationMatrix',Initialparam_const,'PopulationSize',gaopt.PopulationSize,'MaxTime',gaopt.MaxTime,...
        'MaxStallTime',gaopt.MaxStallTime,'MaxStallGenerations',gaopt.MaxStallGenerations,'MaxGenerations',gaopt.MaxGenerations);

    [opt_gains.(axis{i}),fval.(axis{i}),exitflag.(axis{i}),output.(axis{i}),population.(axis{i}),scores.(axis{i})]  ...
        = ga(fun,length(Initialparam_const),[],[],[],[],lb_temp,ub_temp,[],IntCon,ga_options);
    
    saveas(gcf, [plotsFolderPath '\GeneticAlgorithm_' axis{i} '.png']);
    saveas(gcf, [plotsFolderPath '\GeneticAlgorithm_' axis{i} '.fig']);

    %Re-scale optimized gains with the resolution
    opt_gains.(axis{i}) = opt_gains.(axis{i})./gain_resolution;

    %Print result to console
    disp(['The optimized gains are: ' num2str(opt_gains.(axis{i}))])
    disp(['The cost value with the optimized tuning gains is: ' num2str(fval.(axis{i})) ' an improvement of ' num2str((InitialCostScore.(axis{i})-fval.(axis{i}))./fval.(axis{i})*100) '% compared to the intial tuning.'])
    
    %% Plots
    %RUN SIMULATION AND SAVE PLOTS OF IMPROVED RESULTS
    [~,StepResponse.(axis{i})] = GA_tuning_function(opt_gains.(axis{i}).*gain_resolution);

    plotname = ['Performance comprison of optimized vs initial gains for ',axis{i},'-rate'];
    figure('Name',plotname)
    plot(StepResponse.(axis{i}),'LineWidth',1.5)
    text(0.1,0.4,{'Optm Gains:',['P=' num2str(opt_gains.(axis{i})(1))],['I=' num2str(opt_gains.(axis{i})(2))],['D=' num2str(opt_gains.(axis{i})(3))]})
    ylabel('Angular-rate (rad/s)');
    title(plotname);
    legend('Setpoint', 'Optimized response','Opt response with noise','Initial response', 'Opt Command signal with noise')
    grid on 
    grid minor
    saveas(gcf, [plotsFolderPath '\' plotname '.png']);
    saveas(gcf, [plotsFolderPath '\' plotname '.fig']);

    %Run step simulation for last generation 
    plotname = ['Step response of last GA generation for ',axis{i},'-rate'];
    figure('Name',plotname)
    for j=1:size(population.(axis{i}),1)
        [~,reults_tmp] = GA_tuning_function(population.(axis{i})(j,:));
        plot(reults_tmp.Time,reults_tmp.Data(:,3))
        hold on
    end
    grid on; grid minor
    p1 = plot(StepResponse.(axis{i}).Time,StepResponse.(axis{i}).Data(:,1),'k','LineWidth',1.5);
    p2 = plot(StepResponse.(axis{i}).Time,StepResponse.(axis{i}).Data(:,3),'r','LineWidth',2);
    ylabel('Angular-rate (rad/s)')
    xlabel('Time (seconds)')
    title(plotname);
    legend([p1 p2],{'Setpoint','Selected Optimal Chromosome'})
    saveas(gcf, [plotsFolderPath '\' plotname '.png']);
    saveas(gcf, [plotsFolderPath '\' plotname '.fig']);
end


%% Save optimize parameter and results
resultFolderPath = fullfile(pwd, 'Results');
if ~exist(resultFolderPath, 'dir')
    mkdir(resultFolderPath);
end
save([resultFolderPath,'\',optimizationName],'InitialCostScore', 'StepResponse','opt_gains','fval','exitflag','output', 'population', 'scores')




