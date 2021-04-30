%--------------------------------------------------------------------------
% purpose: use rapidy exploring random tree for path planning
%  input:    free_space = collection of valid 2D positions for the robot
%               occ_map = 2D binary occupancy map
%            source_pos = starting position of the robot
%              sink_pos = ending position of the robot
%            num_sample = number of configurations to sample from the free
%                         space
% output:      vertices = graph vertices
%                 edges = graph edges
%                  path = path from source vertex to sink vertex
%--------------------------------------------------------------------------
function [vertices, edges, path] = rapidly_exploring_random_tree(free_space, occ_map, source_pos, sink_pos, num_sample)
% number of free space points
num_free = size(free_space, 1);

% number of vertices in the graph
num_vertices = num_sample + 2;

path_created = false;
num_attempts = 1;

% loop until path created or path is not created in desired number of
% attempts
while ~path_created && num_attempts <= 10
    % pre-allocate the graph
    vertices = nan(num_vertices, 2);
    edges = inf(num_vertices, num_vertices);
    
    % graph initially only consists of source node and sink node
    vertices(1,:) = source_pos;
    
    % sample the free space
    sample_ind = randperm(num_free, num_sample);
    sample_space = free_space(sample_ind, :);
    
    for sample_ind=1:num_sample
        % sample new vertex
        vertex = sample_space(sample_ind, :);
        
        % try to connect new vertex with closest vertex in the graph
        delta_vertex = vertices - vertex;
        dist_vertex = vecnorm(delta_vertex, 2, 2);
        [~, neighbor_ind] = min(dist_vertex);
        
        % add new vertex to graph
        vertex_ind = sample_ind + 1;
        vertices(vertex_ind, :) = vertex;
        
        % try to create edge between vertex and neighbor
        desired_angle =  mod(atan2(delta_vertex(neighbor_ind,2), delta_vertex(neighbor_ind,1)), 2*pi);
        end_points = rayIntersection(occ_map, [vertex 0], desired_angle, 100);
        obstructed = norm(end_points(1,:) - vertex) <= dist_vertex(neighbor_ind);
        
        if ~obstructed
            % add edge
            edges(vertex_ind, neighbor_ind) = dist_vertex(neighbor_ind);
            edges(neighbor_ind, vertex_ind) = dist_vertex(neighbor_ind);            
        end
    end
    
    % add sink node
    vertex = sink_pos;
    
    % try to connect new vertex with closest vertex in the graph
    delta_vertex = vertices - vertex;
    dist_vertex = vecnorm(delta_vertex, 2, 2);
    [~, neighbor_ind] = min(dist_vertex);
    
    % add sink node to graph
    vertices(end, :) = vertex;
    
    % try to create edge between vertex and neighbor
    desired_angle =  mod(atan2(delta_vertex(neighbor_ind,2), delta_vertex(neighbor_ind,1)), 2*pi);
    end_points = rayIntersection(occ_map, [vertex 0], desired_angle, 100);
    obstructed = norm(end_points(1,:) - vertex) <= dist_vertex(neighbor_ind);
    
    if ~obstructed
        % add edge
        edges(end, neighbor_ind) = dist_vertex(neighbor_ind);
        edges(neighbor_ind, end) = dist_vertex(neighbor_ind);
    end
    
    % path successfully created
    path = dijkstra_undirected(vertices, edges);
    path_created = size(path,1) > 1;
    num_attempts = num_attempts + 1;
end
end
%--------------------------------------------------------------------------