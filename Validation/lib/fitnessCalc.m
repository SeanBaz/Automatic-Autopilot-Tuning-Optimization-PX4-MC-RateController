function [SSE,R2,RMSE] = fitnessCalc(y_true,y_evaluated)
    
    %Remove the mean
    y_true = y_true-mean(y_true);
    y_evaluated = y_evaluated-mean(y_evaluated);
    
    %SSE
    SSE = sum((y_true-y_evaluated).^2);
    
    %R2
    y_average = mean(y_true);
    SST = sum((y_true-y_average).^2);
    R2 = (1-SSE/SST)*100;

    %RMSE
    MSE = mean((y_true-y_evaluated).^2);
    RMSE = sqrt(MSE);
end