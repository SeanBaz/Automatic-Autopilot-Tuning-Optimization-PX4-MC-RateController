function [cost, StepResponse] = GA_tuning_function(k)
    
    %If incorrect parameter are passed terminate funciton
    if length(k)<3 || length(k)>4
        errordlg('Incorrect length of parameter for the GA_tuning_function','Error')
        cost = NaN;
        return
    end
    
    %Assign parameters to base workspace of simualtion
    assignin('base','P',k(1));
    assignin('base','I',k(2));
    assignin('base','D',k(3));
    
    if length(k)==4
        assignin('base','dgyro_cutoff',k(4));
    end
    
    
    %Run simulation
    sim('lib\PX4_angRate_controller_GAoptimization.slx');

    %Return cost
    cost = COST.Data(end);
end

