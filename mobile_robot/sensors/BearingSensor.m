classdef BearingSensor
    properties
        % bearing sensor position
        position
        
        % observation noise covariance
        R
    end
    
    methods
        %------------------------------------------------------------------
        % purpose: create a bearing sensor
        %  input: position = bearing sensor position
        %                R = observation noise covariance
        % output:      obj = bearing sensor object
        %------------------------------------------------------------------
        function [obj] = BearingSensor(position, R)
            obj.position = position;
            obj.R = R;
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: observe robot state
        %  input:         obj = bearing sensor object
        %               state = robot state [x y theta]'
        % output: measurement = bearing measurement
        %                   H = observation matrix
        %------------------------------------------------------------------
        function [measurement, H] = observe_state(obj, state)
            delta = obj.position(1:2) - state(1:2);
            a = delta(1)^2 + delta(2)^2;
            measurement = atan2(delta(2), delta(1)) - state(3);
            measurement = normrnd(measurement, obj.R);
            H = [delta(2) / a, -delta(1) / a, -1];
        end
        %------------------------------------------------------------------
    end
end