classdef MobileRobot
    properties (SetAccess = private)
        % true robot state
        state_true
        
        % estimated robot state
        state_est
        
        % state covariance matrix
        P
        
        % process noise covariance matrix
        Q
        
        % collision body
        col_body
        
        % robot radius
        radius
        
        % robot height
        height
        
        % mass of robot
        mass
        
        % planar inertia of robot about z-axis
        inertia
        
        % flag denoting if robot is omnidirectional
        is_omni
        
        % observation sensors
        sensors
        
        % scanner used for mapping
        scanner
    end
    
    methods
        %------------------------------------------------------------------
        % purpose: create a mobile robot
        %  input: initial_state = initial state of robot [x y theta]'
        %                     P = initial state covariance matrix
        %                     Q = process noise covariance matrix
        %                radius = radius of robot
        %                height = height of robot
        %                  mass = mass of robot
        %               is_omni = flag denoting if robot is omnidirectional
        %               sensors = collection of sensors used by robot
        %               scanner = scanner used for mapping
        % output:           obj = mobile robot object
        %------------------------------------------------------------------
        function [obj] = MobileRobot(initial_state, P, Q, radius, height, mass, is_omni, sensors, scanner)
            obj.state_true = initial_state;
            obj.state_est = normrnd(initial_state, diag(P));
            obj.P = P;
            obj.Q = Q;
            obj.radius = radius;
            obj.height = height;
            obj.mass = mass;
            obj.inertia = 0.5 * mass * radius ^ 2;
            obj.is_omni = is_omni;
            obj.sensors = sensors;
            obj.scanner = scanner;
            obj.col_body = collisionCylinder(radius, height);
            obj.col_body.Pose(1:3, 4) = [initial_state(1:2); height / 2];
            obj.col_body.Pose(1:3, 1:3) = rot([0;0;1], initial_state(3));
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: transition the true and estimated states of the robot
        %  input: obj = mobile robot object
        %           u = control input
        %          ts = time step
        % output: obj = mobile robot object
        %           F = state transition matrix
        %------------------------------------------------------------------
        function [obj, F] = transition_state(obj, u, ts)
            theta = obj.state_true(3);
            f = [cos(theta) 0; sin(theta) 0; 0 1];
            obj.state_true = normrnd(obj.state_true + f * u * ts, diag(obj.Q));
            obj.col_body.Pose(1:2, 4) = obj.state_true(1:2);
            obj.col_body.Pose(1:3, 1:3) = rot([0;0;1], obj.state_true(3));
            
            theta = obj.state_est(3);
            f = [cos(theta) 0; sin(theta) 0; 0 1];
            obj.state_est = normrnd(obj.state_est + f * u * ts, diag(obj.Q));
            F = [1 0 -sin(theta) * u(1) * ts; 0 1 cos(theta) * u(1) * ts; 0 0 1];
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: transition the true and estimated states of the
        %          omnidirectional robot
        %  input: obj = mobile robot object
        %           u = control input
        %          ts = time step
        % output: obj = mobile robot object
        %           F = state transition matrix
        %------------------------------------------------------------------
        function [obj, F] = transition_state_omni(obj, u, ts)
            obj.state_true = normrnd(obj.state_true + u * ts, diag(obj.Q));
            obj.col_body.Pose(1:2, 4) = obj.state_true(1:2);
            obj.col_body.Pose(1:3, 1:3) = rot([0;0;1], obj.state_true(3));
            
            obj.state_est = normrnd(obj.state_est + u * ts, diag(obj.Q));
            F = eye(numel(obj.state_true));
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: observe the true and estimated states of the robot
        %  input: obj = mobile robot object
        % output:   y = observation innovation
        %           H = observation matrix
        %           R = observation noise covariance matrix
        %------------------------------------------------------------------
        function [y, H, R] = observe_state(obj)
            num_sensors = numel(obj.sensors);
            y = nan(num_sensors, 1);
            H = nan(num_sensors, numel(obj.state_true));
            R = nan(num_sensors, 1);
            
            for ii=1:num_sensors
                if isa(obj.sensors{ii}, 'RangeSensor')
                    [meas_true, ~] = observe_state(obj.sensors{ii}, [obj.state_true(1:2); obj.height / 2]);
                    [meas_est, H_est] = observe_state(obj.sensors{ii}, [obj.state_est(1:2); obj.height / 2]);
                else
                    [meas_true, ~] = observe_state(obj.sensors{ii}, obj.state_true);
                    [meas_est, H_est] = observe_state(obj.sensors{ii}, obj.state_est);
                end
                
                if ~isempty(meas_true) && ~isempty(meas_est)
                    y(ii) = meas_true - meas_est;
                    H(ii,:) = H_est;
                    R(ii) = obj.sensors{ii}.R;
                end
            end
            
            used_sensors = ~isnan(y);
            y = y(used_sensors);
            H = H(used_sensors, :);
            R = diag(R(used_sensors));
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: update the robot states using extended kalman filter
        %  input: obj = mobile robot object
        %           u = control input
        %          ts = time step
        % output: obj = mobile robot object
        %------------------------------------------------------------------
        function [obj] = kalman_filter(obj, u, ts)
            if obj.is_omni
                [obj, F] = transition_state_omni(obj, u, ts);
            else
                [obj, F] = transition_state(obj, u, ts);
            end
            obj.P = F * obj.P * F' + obj.Q;
            [y, H, R] = observe_state(obj);
            S = H * obj.P * H' + R;
            K = obj.P * H' / S;
            obj.state_est = obj.state_est + K * y;
            obj.P = (eye(size(obj.P)) - K * H) * obj.P;
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: scan the robot's surroundings
        %  input:          obj = mobile robot object
        %                  map = occupancy map
        % output:   detections = detected points in environment
        %         measurements = range measurements
        %               angles = scan angles associated with range
        %                        measurements
        %------------------------------------------------------------------
        function [detections, measurements, angles] = scan(obj, map)
            [measurements, angles] = observe_state(obj.scanner, obj.state_true, map);
            detections = obj.state_est(1:2)' + measurements .* [cos(obj.state_est(3) + angles) sin(obj.state_est(3) + angles)];
        end
        %------------------------------------------------------------------
    end
end