function [maneuver_timeseries] = identify_maneuvers(time, command_signal, response_signal, options)
    Ts = mode(diff(time));
    sample_length = length(time);

    options.min_response_time_samp = floor(options.min_response_time/Ts); %In number of samples

    % Find the time instants where the command signal exceeds the threshold
    maneuver_flag_instants = abs(command_signal) > options.command_threshold;

    % Loop through the detected maneuver start instants to find start and end times for each maneuver
    i=1;
    maneuver_timeseries = [];
    while i <= sample_length
        if maneuver_flag_instants(i)
            %If for 1 seconds the response signal does not go above the threshold discart the current segment
            if all(abs(response_signal(i:i+options.min_response_time_samp))<options.response_threshold)
                i = i+options.min_response_time_samp;
                continue
            end
    
            maneuver_start_time = time(i);
    
            % Find the end time of the maneuver in the response signal
            for j = i+options.min_response_time_samp : sample_length
                j_future = min(j+options.min_response_time_samp,sample_length);
                if all(abs(response_signal(j:j_future)) < options.response_threshold)
                    maneuver_end_time =  time(j);
                    break
                end
            end
            
            %If j ended the loop no end manouver so not completing the loop
            if j==length(response_signal) 
                break
            end
    
            % Check if the maneuver duration is at least options.min_maneuver_duration seconds
            if (maneuver_end_time - maneuver_start_time) >= options.min_maneuver_duration
                % Add the maneuver time series to the matrix and include time before and after
                maneuver_timeseries(end+1,:) = [max(maneuver_start_time-0.5,time(1)), min(maneuver_end_time+0.5,time(end))];
                i = j+floor(0.5/Ts);
            else
                i=i+1;
            end

            %%TODO: Eliminate section that are missing data (log dropouts)

        else
            i=i+1;
        end
    end
end