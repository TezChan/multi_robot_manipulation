classdef RectangleGrasp
    properties (SetAccess = private)
        % length of rectangle in x direction when orientation is 0 degrees
        height
        
        % length of rectangle in y direction when orientation is 0 degrees
        width
        
        % mass of load
        mass
        
        % planar inertia of load about z-axis
        inertia
        
        % grasp type
        % 1 ==> rigid grasp
        % 2 ==> point contact without friction
        % 3 ==> point contact with friction
        grasp_type
        
        % friction cone angle for point contact with friction
        phi
        
        % offsets of contacts from center of load in load frame
        contact_offsets
        
        % agents comprising the grasp
        agents
        
        % number of agents comprising the grasp
        num_agents
    end
    
    methods
        %------------------------------------------------------------------
        % purpose: create a rectangle grasp
        %  input:          height = length of rectangle in x direction when
        %                           orientation is 0 degrees
        %                   width = length of rectangle in y direction when
        %                           orientation is 0 degrees
        %                    mass = mass of load
        %              grasp_type = grasp type
        %                     phi = friction angle for point contact grasp
        %                           with friction
        %         contact_offsets = offsets of contacts from center of
        %                           load in load frame
        %                  agents = list of mobile robot objects
        % output:             obj = rectangle grasp object
        %------------------------------------------------------------------
        function [obj] = RectangleGrasp(height, width, mass, grasp_type, phi, contact_offsets, agents)
            obj.height = height;
            obj.width = width;
            obj.mass = mass;
            obj.inertia = mass * (height ^ 2 + width ^ 2) / 12;
            obj.grasp_type = grasp_type;
            obj.phi = phi;
            obj.contact_offsets = contact_offsets;
            obj.agents = agents;
            obj.num_agents = numel(agents);
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: generate the free space for the grasp
        %  input:        obj = rectangle grasp object
        %             bounds = bounds of free space [xmin xmax ymin ymax]
        %                  N = number of points for each dimension
        %                map = 2D binary occupancy map
        % output: free_space = collection of valid states for the grasp
        %------------------------------------------------------------------
        function [free_space] = generate_free_space(obj, bounds, N, map)
            % x and y points to try
            x = linspace(bounds(1), bounds(2), N);
            y = linspace(bounds(3), bounds(4), N);
            
            % angles to try
            angles = 2 * pi * (0:N-1) / N;
            angles(angles > pi) = angles(angles > pi) - 2 * pi;
            
            % states to try
            [X, Y, Angles] = meshgrid(x, y, angles);
            states = [Angles(:), X(:), Y(:)];
            
            % default rectangle
            rect_x = linspace(-obj.height / 2, obj.height / 2, N);
            rect_y = linspace(-obj.width / 2, obj.width / 2, N);
            [rect_x, rect_y] = meshgrid(rect_x, rect_y);
            rect_def = [rect_x(:), rect_y(:)];
            
            % keep track of valid states
            valid_states = true(N^3, 1);
            
            for ii=1:N^3
                % current state
                angle = states(ii, 1);
                
                % rectangle at current state
                rect = rect_def * [cos(angle) -sin(angle); sin(angle) cos(angle)]' + states(ii, 2:3);
                
                % check for collisions
                collisions = checkOccupancy(map, rect);
                valid_states(ii) = ~any(collisions);
            end
            
            % only keep valid states
            free_space = states(valid_states, :);
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: plot the free space for the grasp
        %  input:        obj = rectangle grasp object
        %         free_space = collection of valid states for the grasp
        % output:
        %------------------------------------------------------------------
        function [] = plot_free_space(obj, free_space)
            % default rectangle
            rect_x = linspace(-obj.height / 2, obj.height / 2, 50);
            rect_y = linspace(-obj.width / 2, obj.width / 2, 50);
            [rect_x, rect_y] = meshgrid(rect_x, rect_y);
            rect_def = [rect_x(:), rect_y(:)];
            
            hold on;
            for ii=1:size(free_space, 1)
                angle = free_space(ii, 1);
                rect = rect_def * [cos(angle) -sin(angle); sin(angle) cos(angle)]' + free_space(ii, 2:3);
                plot(rect(:,1), rect(:,2), 'b.');
            end
            hold off;
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: generate a path for the grasp using probability road map
        %  input:           obj = rectangle grasp object
        %            free_space = collection of valid states for the grasp
        %                   map = 2D binary occupancy map
        %          source_state = initial state
        %            sink_state = final state
        %            num_sample = number of configurations to sample from
        %                         the free space
        %         num_neighbors = maximum number of neighbors a vertex can
        %                         have
        %                    sf = smoothing factor
        % output:          path = collection of discrete states along a
        %                         path for the grasp
        %           path_smooth = smoothed collection of discrete states
        %                         along a path for the grasp
        %------------------------------------------------------------------
        function [path, path_smooth] = plan_path(obj, free_space, map, source_state, sink_state, num_sample, num_neighbors, sf)
            % number of free space points
            num_free = size(free_space, 1);
            
            path_created = false;
            num_attempts = 1;
            
            % loop until path created or path is not created in desired number of
            % attempts
            while ~path_created && num_attempts <= 10
                % sample the free space
                sample_ind = randperm(num_free, num_sample);
                sample_space = free_space(sample_ind, :);
                
                % add source and sink positions to vertex list
                vertices = [source_state; sample_space; sink_state];
                num_vertices = num_sample + 2;
                
                % store graph edges
                edges = inf(num_vertices, num_vertices);
                
                % create graph edges
                for vertex_ind=1:num_vertices
                    % current vertex in graph
                    vertex = vertices(vertex_ind, :);
                    
                    % position delta between vertices
                    delta_vertex_pos = vertices(:,2:3) - vertex(2:3);
                    
                    % position error metric between vertices
                    dist_vertex_pos = vecnorm(delta_vertex_pos, 2, 2);
                    
                    % orientation error metric between vertices
                    dist_vertex_ori = zeros(size(dist_vertex_pos));
                    R_vertex = rot([0; 0; 1], vertex(1));
                    for ii=1:numel(dist_vertex_ori)
                        R_tmp = rot([0; 0; 1], vertices(ii, 1));
                        dist_vertex_ori(ii) = norm(R_vertex * R_tmp' - eye(3));
                    end
                    
                    % total error metric
                    dist_vertex = dist_vertex_pos + dist_vertex_ori;
                    [~, sort_ind] = sort(dist_vertex);
                    neighbors = sort_ind(1:num_neighbors+1);
                    
                    % try to create edge between vertex and neighbors
                    for ii=1:num_neighbors
                        neighbor_ind = neighbors(ii);
                        if neighbor_ind ~= vertex_ind
                            if isinf(edges(vertex_ind, neighbor_ind))
                                % edge does not already exist, see if vertex can be linked to
                                % neighbor with straight line
                                desired_angle =  mod(atan2(delta_vertex_pos(neighbor_ind,2), delta_vertex_pos(neighbor_ind,1)), 2*pi);
                                end_points = rayIntersection(map, [vertex(2:3) 0], desired_angle, 100);
                                obstructed = norm(end_points(1,:) - vertex(2:3)) <= dist_vertex_pos(neighbor_ind);
                                
                                if ~obstructed
                                    % add edge
                                    edges(vertex_ind, neighbor_ind) = dist_vertex(neighbor_ind);
                                    edges(neighbor_ind, vertex_ind) = dist_vertex(neighbor_ind);
                                end
                            end
                        end
                    end
                end
                
                % path successfully created
                path = dijkstra_undirected(vertices, edges);
                path_created = size(path,1) > 1;
                num_attempts = num_attempts + 1;
            end
            
            % smooth out path
            ind = 1:size(path, 1);
            ind_new = 1:1/sf:size(path, 1);
            path_smooth = zeros(numel(ind_new), 3);
            for ii=1:3
                path_smooth(:,ii) = interp1(ind, path(:,ii), ind_new);
            end
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: plot the path for the grasp
        %  input:  obj = rectangle grasp object
        %         path = collection of discrete states along a path for the
        %                grasp
        % output:
        %------------------------------------------------------------------
        function [] = plot_path(obj, path)
            % default rectangle
            rect_x = linspace(-obj.height / 2, obj.height / 2, 50);
            rect_y = linspace(-obj.width / 2, obj.width / 2, 50);
            [rect_x, rect_y] = meshgrid(rect_x, rect_y);
            rect_def = [rect_x(:), rect_y(:)];
            
            hold on;
            for ii=1:size(path, 1)
                angle = path(ii, 1);
                rect = rect_def * [cos(angle) -sin(angle); sin(angle) cos(angle)]' + path(ii, 2:3);
                plot(rect(:,1), rect(:,2), 'r.');
            end
            hold off;
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: generate a trajectory for the grasp from a collection of
        %          states along a path using a trapezoidal velocity profile
        %  input:        obj = rectangle grasp object
        %               path = collection of discrete states along a path for the grasp
        %              v_max = maximum velocity magnitude
        %              a_max = maximum acceleration magnitude
        %              d_max = maximum deceleration magnitude
        % output: trajectory = time indexed trajectory of grasp
        %                  t = time vector associated with trajectory
        %------------------------------------------------------------------
        function [trajectory, t] = plan_trajectory(obj, path, v_max, a_max, d_max)
            % lambda due to positional error
            lambda_pos = vecnorm(diff(path(:, 2:3)), 2, 2);
            
            % lambda due to orientation error
            lambda_ori = zeros(size(lambda_pos));
            for ii=2:size(path, 1)
                R_prev = rot([0; 0; 1], path(ii-1, 1));
                R_cur = rot([0; 0; 1], path(ii, 1));
                lambda_ori(ii-1) = norm(R_prev * R_cur' - eye(3));
            end
            
            % total discretized lambda
            lambda_dis = [0; cumsum(lambda_pos + lambda_ori)];
            
            % end time of acceleration period
            t_acc = v_max / a_max;
            
            % time difference between end of deceleration period and
            % coasting period
            t_dec_coast = v_max / d_max;
            
            % end time of deceleration period
            t_dec = (lambda_dis(end) - lambda_dis(1) + 0.5 * a_max * t_acc^2 + 0.5 * d_max * t_dec_coast^2) / (a_max * t_acc);
            
            % end time of coasting period
            t_coast = t_dec - t_dec_coast;
            
            % time vector
            t = 0:0.1:t_dec;
            
            % path lengths vs time
            lambda = zeros(size(t));
            
            % acceleration period
            t_ind = t < t_acc;
            t_delta = t_acc - t(t_ind);
            lambda_acc = lambda_dis(1) + 0.5 * a_max * t_acc ^ 2;
            lambda(t_ind) = lambda_acc + 0.5 * a_max * (t_delta .^ 2 - 2 * t_delta * t_acc);
            
            % coasting period
            t_ind = t >= t_acc & t < t_coast;
            t_delta = t_coast - t(t_ind);
            lambda_coast = lambda_acc + v_max * (t_coast - t_acc);
            lambda(t_ind) = lambda_coast - v_max * t_delta;
            
            % deceleration period
            t_ind = t >= t_coast;
            t_delta = t_dec - t(t_ind);
            lambda_dec = lambda_coast + v_max * (t_dec - t_coast) - 0.5 * d_max * (t_dec - t_coast) ^ 2;
            lambda(t_ind) = lambda_dec - v_max * t_delta - 0.5 * d_max * (t_delta .^ 2 + 2 * t_delta * (t_coast - t_dec));
            
            % grasp trajectory
            trajectory = zeros(numel(t), 3);
            for ii=1:3
                trajectory(:,ii) = interp1(lambda_dis, path(:,ii), lambda);
            end
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: plot the trajectory for the grasp
        %  input:        obj = rectangle grasp object
        %         trajectory = time indexed trajectory of grasp
        % output:
        %------------------------------------------------------------------
        function [] = plot_trajectory(obj, trajectory)
            % default rectangle
            rect_x = linspace(-obj.height / 2, obj.height / 2, 50);
            rect_y = linspace(-obj.width / 2, obj.width / 2, 50);
            [rect_x, rect_y] = meshgrid(rect_x, rect_y);
            rect_def = [rect_x(:), rect_y(:)];
            
            hold on;
            for ii=1:size(trajectory, 1)
                angle = trajectory(ii, 1);
                rect = rect_def * [cos(angle) -sin(angle); sin(angle) cos(angle)]' + trajectory(ii, 2:3);
                plot(rect(:,1), rect(:,2), 'g.');
                pause(0.05);
            end
            hold off;
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: compute contact velocities along trajectory
        %  input:        obj = rectangle grasp object
        %         trajectory = time indexed trajectory of grasp
        %                 ts = time step
        % output: v_contacts = contact velocities along trajectory
        %------------------------------------------------------------------
        function [v_contacts] = compute_contact_velocities(obj, trajectory, ts)
            % load velocity
            vel = [zeros(1, 3); diff(trajectory) / ts];
            
            % contact velocities
            v_contacts = zeros(size(trajectory, 1), 3 * obj.num_agents);
            
            for ii=1:size(trajectory, 1)
                R0C = rot([0;0;1], trajectory(ii, 1));
                for jj=1:obj.num_agents
                    p_delta = -R0C * hat(obj.contact_offsets(:,jj));
                    A = [1 0 0; p_delta(1:2, 3), eye(2)];                    
                    v_contacts(ii, 3*(jj-1)+1:3*jj) = vel(ii,:) * A';
                end
            end
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: compute agent velocities along trajectory
        %  input:        obj = rectangle grasp object
        %         v_contacts = contact velocities at current time
        %         load_state = state of load at current time
        % output:   v_agents = contact velocities at current time
        %------------------------------------------------------------------
        function [v_agents] = compute_agent_velocities(obj, v_contacts, load_state)
            % reformat inputs
            load_state = load_state(:);
            v_contacts = reshape(v_contacts, 3, []);
            
            % agent velocities
            v_agents = zeros(size(v_contacts));
            
            for ii=1:obj.num_agents
                R0C = rot([0;0;1], load_state(1));
                p_contact = [load_state(2:3); 0] + R0C * obj.contact_offsets(:,ii);
                p_delta = -hat(obj.agents(ii).state_est - p_contact);
                B = [1 0 0; p_delta(1:2,3) eye(2)];
                
                if obj.grasp_type > 1
                    % no angular velocity for point contact
                    B = [0 0 0; 0 1 0; 0 0 1] * B;
                end
                
                v_agents(:,ii) = B * v_contacts(:,ii);
            end
            
            % mobile robot agents use different state ordering
            v_agents = circshift(v_agents, -1, 1);
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: compute load forces along trajectory
        %  input:        obj = rectangle grasp object
        %         trajectory = time indexed trajectory of grasp
        %                 ts = sample time
        % output:     F_load = load forces along trajectory
        %------------------------------------------------------------------
        function [F_load] = compute_load_force(obj, trajectory, ts)
            % load velocities
            v_load = [zeros(1, 3); diff(trajectory) / ts];
            for ii=1:3
                v_load(:,ii) = smooth(v_load(:,ii));
            end
            
            % load accelerations
            a_load = [zeros(1, 3); diff(v_load) / ts];
            for ii=1:3
                a_load(:,ii) = smooth(a_load(:,ii));
            end
            
            % load forces
            F_load = a_load .* [obj.inertia obj.mass obj.mass];
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: compute contact forces along trajectory
        %  input:           obj = rectangle grasp object
        %            trajectory = time indexed trajectory of grasp
        %                F_load = load forces along trajectory
        % output:    F_contacts = contact forces along trajectory
        %------------------------------------------------------------------
        function [F_contacts] = compute_contact_forces(obj, trajectory, F_load)
            % contact forces
            F_contacts = zeros(size(F_load, 1), 3 * obj.num_agents);
            
            % keep track of which forces contacts can have
            used_forces = true(3 * obj.num_agents, 1);
            
            if obj.grasp_type > 1
               % agents cannot have a torque
               used_forces(1:3:end) = false;
            end
            
            if obj.grasp_type == 3
               % determine friction cones for contacts
               Ra = rot([0;0;1], obj.phi);
               Rb = rot([0;0;1], -obj.phi);
               fa = Ra * obj.contact_offsets;
               fb = Rb * obj.contact_offsets;
               fa = fa / norm(fa);
               fb = fb / norm(fb);
               C = zeros(2, 2, obj.num_agents);
               
               % propagation matrix from contacts to load
               G = zeros(3, 2 * obj.num_agents);
               for ii=1:obj.num_agents
                   Ci = [fa(:,ii), fb(:,ii)];
                   px = hat(obj.contact_offsets(:,ii));
                   G(:, 2*(ii-1)+1:2*ii) = [[0 0 -1] * px * Ci; Ci(1:2,:)];
                   C(:,:,ii) = Ci(1:2,:);
               end
               
               % force closure
               [~, b, ~, ~] = vert2lcon(G');
               
               if min(b)>0
                   disp('Force closure satisfied');
               else
                   disp('Force closure violated');
               end
               
               % optimization options
               options = optimoptions('fmincon', 'StepTolerance', 1e-10,...
                   'Algorithm', 'sqp', 'Display', 'off');
            end
            
            for ii=1:size(F_load, 1)
                if obj.grasp_type ~= 3
                    % rigid grasp and point contact without friction use
                    % pseudoinverse to determine contact forces
                    A = [];

                    R0C = rot([0;0;1], trajectory(ii, 1));
                    for jj=1:obj.num_agents
                        p = -R0C * hat(obj.contact_offsets(:,jj));          
                        Ai = [1 0 0; p(1:2, 3), eye(2)];

                        if obj.grasp_type > 1
                           % need to drop torque for point contact
                           Ai = [0 1 0; 0 0 1] * Ai;
                        end
                        A = [A; Ai];
                    end
                    
                    F_contacts(ii, used_forces) = pinv(A)' * F_load(ii,:)';
                else
                    F_contacts(ii, used_forces) = fmincon(@(x) friction_objective_fun(obj, x, C),...
                        zeros(2 * obj.num_agents, 1), [], [], G, F_load(ii,:)',...
                        zeros(2 * obj.num_agents, 1), [], [], options);
                end 
            end
            
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: update agent states
        %  input: obj = rectangle grasp object
        %           u = control input
        %          ts = time step
        % output: obj = rectangle grasp object
        %------------------------------------------------------------------
        function [obj] = update_agent_states(obj, u, ts)
            for ii=1:obj.num_agents
                obj.agents(ii) = kalman_filter(obj.agents(ii), u(:,ii), ts);
            end
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: return agent states
        %  input:          obj = rectangle grasp object
        % output: agent_states = agent states
        %------------------------------------------------------------------
        function [agent_states] = get_agent_states(obj)
            agent_states = zeros(3, obj.num_agents);
            for ii=1:obj.num_agents
                agent_states(:,ii) = obj.agents(ii).state_est;
            end
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: objective function to minimize for contact force
        %          computation with a point contact grasp with friction
        %  input: obj = rectangle grasp object
        %         eta = potential contact forces
        %           C = collection of friction cone boundaries for each
        %               contact
        % output: val = objective value
        %------------------------------------------------------------------
        function [val] = friction_objective_fun(obj, eta, C)
            val = 0;
            for ii=1:size(C, 3)
               val = val + norm(C(:,:,ii) * eta(2*(ii-1)+1:2*ii))^2;
            end
        end
        %------------------------------------------------------------------
    end
end