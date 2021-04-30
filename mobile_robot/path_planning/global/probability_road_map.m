%--------------------------------------------------------------------------
% purpose: use probability road map for path planning
%  input:    free_space = collection of valid 2D positions for the robot
%               occ_map = 2D binary occupancy map
%            source_pos = starting position of the robot
%              sink_pos = ending position of the robot
%            num_sample = number of configurations to sample from the free
%                         space
%         num_neighbors = maximum number of neighbors a vertex can have
% output:      vertices = graph vertices
%                 edges = graph edges
%                  path = path from source vertex to sink vertex
%--------------------------------------------------------------------------
function [vertices, edges, path] = probability_road_map(free_space, occ_map, source_pos, sink_pos, num_sample, num_neighbors)
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
    vertices = [source_pos; sample_space; sink_pos];
    num_vertices = num_sample + 2;
    
    % store graph edges
    edges = inf(num_vertices, num_vertices);
    
    % create graph edges
    for vertex_ind=1:num_vertices
        % current vertex in graph
        vertex = vertices(vertex_ind, :);
        
        % determine closest neighbors to vertex
        delta_vertex = vertices - vertex;
        dist_vertex = vecnorm(delta_vertex, 2, 2);
        [~, sort_ind] = sort(dist_vertex);
        neighbors = sort_ind(1:num_neighbors+1);
        
        % try to create edge between vertex and neighbors
        for ii=1:num_neighbors
            neighbor_ind = neighbors(ii);
            if neighbor_ind ~= vertex_ind
                if isinf(edges(vertex_ind, neighbor_ind))
                    % edge does not already exist, see if vertex can be linked to
                    % neighbor with straight line
                    desired_angle =  mod(atan2(delta_vertex(neighbor_ind,2), delta_vertex(neighbor_ind,1)), 2*pi);
                    end_points = rayIntersection(occ_map, [vertex 0], desired_angle, 100);
                    obstructed = norm(end_points(1,:) - vertex) <= dist_vertex(neighbor_ind);
                    
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
end
%--------------------------------------------------------------------------