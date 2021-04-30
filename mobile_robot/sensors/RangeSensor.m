classdef RangeSensor
    properties
        % range sensor position
        position
        
        % maximum range
        range
        
        % observation noise covariance
        R
    end
    
    methods
        %------------------------------------------------------------------
        % purpose: create a range sensor
        %  input: position = range sensor position
        %            range = maximum range
        %                R = observation noise covariance
        % output:      obj = range sensor object
        %------------------------------------------------------------------
        function [obj] = RangeSensor(position, range, R)
           obj.position = position;
           obj.range = range;
           obj.R = R;
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: observe robot state
        %  input:         obj = range sensor object
        %               state = robot state [x y z]'
        % output: measurement = range measurement
        %                   H = observation matrix
        %------------------------------------------------------------------
        function [measurement, H] = observe_state(obj, state)
            measurement = norm(obj.position - state);
            if measurement > obj.range
                measurement = [];
                H = [];
            else
                H = [-(obj.position(1:2) - state(1:2))' / measurement, 0];
                measurement = normrnd(measurement, obj.R);
            end
        end
        %------------------------------------------------------------------
    end
end