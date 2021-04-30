classdef Scanner
    properties
        % scan angles
        angles
        
        % maximum scan range
        range
        
        % observation noise covariance
        R
    end
    
    methods
        %------------------------------------------------------------------
        % purpose: create a scanner
        %  input:     M = number of scan angles
        %         range = maximum scan range
        %             R = observation noise covariance matrix
        % output:   obj = scanner object
        %------------------------------------------------------------------
        function [obj] = Scanner(M, range, R)
            obj.angles = 2 * pi * (0:M-1)' / M;
            obj.range = range;
            obj.R = R;
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: observe robot state
        %  input:          obj = scanner object
        %                state = robot state [x y theta]'
        %                  map = occupancy map
        % output: measurements = range measurements
        %               angles = scan angles associated with range
        %                        measurements
        %------------------------------------------------------------------
        function [measurements, angles] = observe_state(obj, state, map)
            end_points = rayIntersection(map, state, obj.angles, obj.range);
            measurements = vecnorm(end_points - state(1:2)', 2, 2);
            measurements = normrnd(measurements, diag(obj.R));
            out_range = isnan(measurements);
            measurements = measurements(~out_range);
            angles = obj.angles(~out_range);
        end
        %------------------------------------------------------------------
    end
end