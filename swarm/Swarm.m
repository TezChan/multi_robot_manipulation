classdef Swarm
    properties (SetAccess = private)
        % position of swarm
        % center of swarm circle for distributed control, position of agent
        % 1 for follow-the-leader control
        position
        
        % radius of swarm circle
        radius
        
        % orientation of swarm
        angle
        
        % graph of agent network
        G
        
        % incidence matrix of agent network
        D
        
        % agents comprising the swarm
        agents
        
        % number of agents comprising the swarm
        num_agents
        
        % number of features each agent has
        num_features
        
        % number of links in the agent network
        num_links
        
        % feedback gain for relative differences between agents
        K_rel
        
        % feedback gain for swarm positioning
        K_pos
        
        % feedback gain for agent-agent collision avoidance
        K_agent_col
        
        % feedback gain for agent-environment collision avoidance
        K_env_col
        
        % separation distance between agents when in a platoon
        platoon_sep
        
        % agent control scheme relative to other agents
        % 1 ==> pose control
        % 2 ==> distance control
        % 3 ==> platoon control
        rel_scheme
        
        % swarm positioning scheme
        % 1 ==> distributed (centroid) control
        % 2 ==> follow-the-leader
        pos_scheme
        
        % control phase of the swarm
        % 1 ==> swarm formation
        % 2 ==> path following
        phase
        
        % gain used for control inputs most crucial to current control
        % phase
        gain_used = 0.2;
        
        % denotes whether swarm has converged to a position
        converged = false;
    end
    
    methods
        %------------------------------------------------------------------
        % purpose: create a swarm
        %  input:    position = position of swarm
        %              radius = radius of swarm circle
        %               angle = orientation of swarm
        %              agents = list of mobile robot objects
        %                   G = graph of agent network
        %          rel_scheme = agent control scheme relative to other
        %                       agents
        %          pos_scheme = swarm positioning scheme
        %         platoon_sep = separation distance between agents when in
        %                       a platoon
        % output:         obj = swarm object
        %------------------------------------------------------------------
        function [obj] = Swarm(position, radius, angle, agents, G, rel_scheme, pos_scheme, platoon_sep)
            obj.position = position;
            obj.radius = radius;
            obj.angle = angle;
            obj.G = G;
            obj.D = full(incidence(G));
            obj.num_links = size(obj.D, 2);
            obj.agents = agents;
            obj.num_agents = numel(agents);
            obj.num_features = numel(agents(1).state_true);
            obj.rel_scheme = rel_scheme;
            obj.pos_scheme = pos_scheme;
            obj.platoon_sep = platoon_sep;
            obj = set_phase(obj, 1);
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: update the position of the swarm circle
        %  input:      obj = swarm object
        %         position = center of swarm circle
        % output:      obj = swarm object
        %------------------------------------------------------------------
        function [obj] = set_position(obj, position)
            obj.position = position;
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: update the radius of the swarm circle
        %  input:    obj = swarm object
        %         radius = radius of swarm circle
        % output:    obj = swarm object
        %------------------------------------------------------------------
        function [obj] = set_radius(obj, radius)
            obj.radius = radius;
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: update the orientation of the swarm
        %  input:   obj = swarm object
        %         angle = orientation of swarm
        % output:   obj = swarm object
        %------------------------------------------------------------------
        function [obj] = set_angle(obj, angle)
            obj.angle = angle;
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: update the agent network
        %  input: obj = swarm object
        %           G = graph of agent network
        % output: obj = swarm object
        %------------------------------------------------------------------
        function [obj] = set_network(obj, G)
            obj.G = G;
            obj.D = full(incidence(G));
            obj.num_links = size(obj.D, 2);
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: update with new collection of agents
        %  input:    obj = swarm object
        %              G = graph of agent network
        %         agents = list of mobile robot objects
        % output:    obj = swarm object
        %------------------------------------------------------------------
        function [obj] = set_agents(obj, G, agents)
            obj = set_network(obj, G);
            obj.agents = agents;
            obj.num_agents = numel(agents);
            obj.num_features = numel(agents(1).state_true);
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: update swarm control phase and associated gains
        %  input:   obj = swarm object
        %         phase = new control phase
        % output:   obj = swarm object
        %------------------------------------------------------------------
        function [obj] = set_phase(obj, phase)
            obj.phase = phase;
            if phase == 1
                obj.K_rel = obj.gain_used;
                obj.K_agent_col = 2 * obj.gain_used;
                obj.K_env_col = 2 * obj.gain_used;
                obj.K_pos = 0;
            else
                obj.K_rel = obj.gain_used;
                obj.K_pos = obj.gain_used; 
                obj.K_agent_col = 2 * obj.gain_used;
                obj.K_env_col = 2 * obj.gain_used;
            end
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: update swarm control scheme
        %  input:        obj = swarm object
        %         rel_scheme = agent control scheme relative to other
        %                      agents
        %         pos_scheme = swarm positioning scheme
        % output:        obj = swarm object
        %------------------------------------------------------------------
        function [obj] = set_scheme(obj, rel_scheme, pos_scheme)
            obj.rel_scheme = rel_scheme;
            obj.pos_scheme = pos_scheme;
            obj = set_phase(obj, 1);
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: determine the desired difference variable
        %  input:          obj = swarm object
        % output: diff_desired = desired difference variable
        %------------------------------------------------------------------
        function [diff_desired] = get_desired_difference(obj)
            switch obj.rel_scheme
                case {1, 2}
                    % pose or distance control
                    agent_angle = obj.angle + (0:obj.num_agents-1) * 2 * pi / obj.num_agents;
                    agent_pos = obj.radius * [cos(agent_angle); sin(agent_angle)];
                    agent_states = [agent_pos; agent_angle];
                    diff_desired = kron(obj.D', eye(obj.num_features)) * agent_states(:);
                case 3
                    % platoon control
                    diff_desired = kron(ones(obj.num_links, 1), [-obj.platoon_sep; 0; 0]);
            end
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: determine the true difference variable
        %  input:       obj = swarm object
        % output: diff_true = true difference variable
        %------------------------------------------------------------------
        function [diff_true] = get_true_difference(obj)
            agent_states = get_agent_states(obj);
            diff_true = kron(obj.D', eye(obj.num_features)) * agent_states(:);
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: return agent states
        %  input:          obj = swarm object
        % output: agent_states = agent states
        %------------------------------------------------------------------
        function [agent_states] = get_agent_states(obj)
            agent_states = zeros(obj.num_features, obj.num_agents);
            for ii=1:obj.num_agents
                agent_states(:,ii) = obj.agents(ii).state_est;
            end
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: update agent states
        %  input: obj = swarm object  
        %           u = collection of control inputs for each agent
        %          ts = time step
        % output: obj = swarm object
        %------------------------------------------------------------------
        function [obj] = update_agent_states(obj, u, ts)
            for ii=1:obj.num_agents
                obj.agents(ii) = kalman_filter(obj.agents(ii), u(:,ii), ts);
            end
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: return centroid of agent positions
        %  input:      obj = swarm object
        % output: centroid = centroid of agent positions
        %------------------------------------------------------------------
        function [centroid] = get_agent_centroid(obj)
            agent_states = get_agent_states(obj);
            centroid = [sum(agent_states(1,:)); sum(agent_states(2,:))] / obj.num_agents;
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: stich individual agent scans into a larger swarm scan
        %  input:          obj = swarm object
        %                  map = occupancy map
        % output: measurements = range measurements
        %               angles = scan angles associated with range
        %                        measurements
        % notes: scan is only to be used in phase 3 - path following
        %------------------------------------------------------------------
        function [measurements, angles] = scan(obj, map)            
            measurements = [];
            angles = [];
            
            % centroid of swarm
            centroid = get_agent_centroid(obj);
            
            for ii=1:obj.num_agents
                % individual agent detections
                [detections, ~, ~] = scan(obj.agents(ii), map);
                
                % difference between detection points and swarm centroid
                deltas = detections - centroid';
                
                % associated range measurements and angles
                new_meas = vecnorm(deltas, 2, 2);
                new_ang = atan2(deltas(:,2), deltas(:,1));
                
                % add new measurements
                measurements = [measurements; new_meas];
                angles = [angles; new_ang];
            end
            
            % sort angles and measurements
            [angles, sort_ind] = sort(angles);
            measurements = measurements(sort_ind);
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: get control input due to relative differences between
        %          agents
        %  input: obj = swarm object
        % output:   u = control input
        %------------------------------------------------------------------
        function [u] = get_rel_control(obj)
            diff_desired = get_desired_difference(obj);
            diff_true = get_true_difference(obj);
            switch obj.rel_scheme
                case {1, 3}
                    % pose or platoon control
                    err = diff_desired - diff_true;
                    u = obj.K_rel * kron(obj.D, eye(obj.num_features)) * err;
                case 2
                    % distance control
                    diff_desired = reshape(diff_desired, obj.num_features, []);
                    diff_desired = diff_desired(1:2,:);
                    dist_desired = vecnorm(diff_desired);
                    diff_true = reshape(diff_true, obj.num_features, []);
                    diff_true = diff_true(1:2,:);
                    dist_true = vecnorm(diff_true);
                    err = [(1 ./ dist_true - 1 ./ dist_desired) .* diff_true ./ dist_true; zeros(obj.num_features-2, obj.num_agents)];
                    u = obj.K_rel * kron(obj.D, eye(obj.num_features)) * err(:);
            end
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: get control input due to swarm positioning
        %  input: obj = swarm object
        % output:   u = control input
        %------------------------------------------------------------------
        function [u] = get_pos_control(obj)
            switch obj.pos_scheme
                case 1
                    % distributed (centroid) position control
                    agent_centroid = get_agent_centroid(obj);
                    delta = obj.position - agent_centroid;
                    err = repmat([delta; zeros(obj.num_features-2, 1)], obj.num_agents, 1);
                case 2
                    % follow-the-leader (agent 1) position control
                    leader_pos = obj.agents(1).state_est(1:2);
                    delta = obj.position - leader_pos;
                    err = zeros(obj.num_agents * obj.num_features, 1);
                    err(1:2) = delta;
            end
            u = obj.K_pos * err;
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: get control input due to agent-agent collision avoidance
        %  input:        obj = swarm object
        %         col_buffer = distance used for collision avoidance
        % output:          u = control input
        %------------------------------------------------------------------
        function [u] = get_agent_col_control(obj, col_buffer)
            agent_states = get_agent_states(obj);
            G_full = ones(obj.num_agents);
            G_full = G_full - eye(obj.num_agents);
            D_full = full(incidence(graph(G_full)));
            deltas = reshape(kron(D_full', eye(obj.num_features)) * agent_states(:), obj.num_features, []);
            deltas(3:end,:) = [];
            distances = vecnorm(deltas);
            err = [(1./distances - 1/col_buffer) .* deltas ./ distances .* (distances < col_buffer); zeros(obj.num_features-2, size(D_full, 2))];
            u = obj.K_agent_col * kron(D_full, eye(obj.num_features)) * err(:);
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: get control input due to agent-environment collision
        %          avoidance
        %  input:        obj = swarm object
        %             colobj = cell array of collision objects representing
        %                      environment obstacles
        %         col_buffer = distance used for collision avoidance
        % output:          u = control input
        %------------------------------------------------------------------
        function [u] = get_env_col_control(obj, colobj, col_buffer)
            err = zeros(obj.num_features, obj.num_agents);
            for ii=1:obj.num_agents
                for jj=1:numel(colobj)
                    [~, distance, wit_points] = checkCollision(obj.agents(ii).col_body, colobj{jj});
                    delta_env = wit_points(1:2,1) - wit_points(1:2,2);
                    if distance < col_buffer
                        err(:,ii) = err(:,ii) + [(1/distance - 1/col_buffer) * delta_env / distance; 0];
                    end
                end
            end
            u = obj.K_env_col * err(:);
        end
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        % purpose: control swarm
        %  input:        obj = swarm object
        %             colobj = cell array of collision objects representing
        %                      environment obstacles
        %         col_buffer = distance used for collision avoidance
        %              u_max = control input saturation levels
        %                 ts = time step
        %            epsilon = convergence value
        % output:        obj = swarm object
        %------------------------------------------------------------------
        function [obj] = control(obj, colobj, col_buffer, u_max, ts, epsilon)
            % individual control inputs
            u_rel = get_rel_control(obj);
            u_pos = get_pos_control(obj);
            u_agent_col = get_agent_col_control(obj, col_buffer);
            u_env_col = get_env_col_control(obj, colobj, col_buffer);

            % total control input
            u = u_rel + u_pos + u_agent_col + u_env_col;
            u = reshape(u, obj.num_features, []);
            
            % saturate control input
            u = sign(u) .* min(abs(u), u_max);
            
            % update states of the agents
            obj = update_agent_states(obj, u, ts);
            
            % update swarm control phase
            if obj.phase == 1 && norm(u_rel) < epsilon
                % move from formation control to path following
                obj = set_phase(obj, 2);
                disp('move from formation control to path following');
            end
            
            % denote whether or not the swarm has converged to a position
            obj.converged = obj.phase == 2 && norm(u_pos + u_rel) < epsilon;
        end
        %------------------------------------------------------------------
    end
end