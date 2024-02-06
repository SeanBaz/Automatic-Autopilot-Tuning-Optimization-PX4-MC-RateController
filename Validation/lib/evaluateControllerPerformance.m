function performanceMetrics = evaluateControllerPerformance(Validation_Results)



    % Extract simulation data
    time = Validation_Results.Time;
    output = Validation_Results.Data(:,2);
    setpoint = Validation_Results.Data(:,1); 

    % Calculate performance metrics
    steadyStateError = setpoint(end) - output(end);
    riseTime = calculateRiseTime(time, output, setpoint);
    settlingTime = calculateSettlingTime(time, output, setpoint);
    overshoot = calculateOvershoot(output, setpoint);

    % Create a structure to store the results
    performanceMetrics = struct(...
        'SteadyStateError', steadyStateError, ...
        'RiseTime', riseTime, ...
        'SettlingTime', settlingTime, ...
        'Overshoot', overshoot ...
    );

end

function tr = calculateRiseTime(time, output, setpoint)
    % Calculate rise time
    threshold_min = 0.1 * (setpoint(end) - output(1)) + output(1);
    threshold_max = 0.9 * setpoint(end);
    idxRiseStart = find(output > threshold_min, 1);
    idxRiseEnd = find(output > threshold_max, 1);
    tr = time(idxRiseEnd) - time(idxRiseStart);
end

function ts = calculateSettlingTime(time, output, setpoint)
    % Calculate settling time
    tolerance = 0.05 * (setpoint(end) - output(1));
    idxSettleStart = find(abs(output - output(end)) > tolerance, 1, "last");
    idxStep = find(abs(setpoint-setpoint(end))< tolerance, 1);
    ts = time(idxSettleStart) - time(idxStep);
end

function overshoot = calculateOvershoot(output, setpoint)
    % Calculate overshoot in percentage
    overshoot = max((output - setpoint(end)) / setpoint(end)) * 100;
end
