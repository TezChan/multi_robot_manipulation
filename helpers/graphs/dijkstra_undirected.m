%--------------------------------------------------------------------------
% purpose: use dijkstra's algorithm to find the shortest path between two
%          vertices
%  input: vertices = vertices (x, y) in graph where the first vertex is the
%                    source and the last vertex is the sink
%            edges = symmetric matrix containing the distance between two
%                    vertices if they are connected and inf if they are not
% output:     path = shortest path between source and sink vertices
%--------------------------------------------------------------------------
function [path] = dijkstra_undirected(vertices, edges)
% number of vertices in graph
num_vertices = size(vertices, 1);

% queue used to traverse graph
vertex_queue = nan(num_vertices, 1);

% queue starts with source vertex
vertex_queue(1) = 1;
queue_size = 1;

% keep track of visited vertices
vertex_visited = false(1, num_vertices);
vertex_visited(1) = true;

% set initial vertex distances from source vertex
vertex_distances = inf(num_vertices, 1);
vertex_distances(1) = 0;

% keep track of each vertex's predecessor in path
vertex_prev = nan(num_vertices, 1);

% use dijkstra's algorithm to find shortest path between start and end
% vertex
for ii=1:num_vertices
    vertex_ind = vertex_queue(ii);
    
    if ~isnan(vertex_ind)
        % vertex's distance from start vertex
        vertex_distance = vertex_distances(vertex_ind);
        
        % find vertex's neighbors
        neighbors = ~isinf(edges(vertex_ind, :));
        neighbors_unvisited = neighbors & ~vertex_visited;
        neighbors = find(neighbors);
        neighbors_unvisited = find(neighbors_unvisited);
        num_neighbors = numel(neighbors);
        num_neighbors_unvisited = numel(neighbors_unvisited);
        
        % add unvisited neighbors to queue
        vertex_queue(queue_size+1:queue_size+num_neighbors_unvisited) = neighbors_unvisited;
        queue_size = queue_size + num_neighbors_unvisited;
        vertex_visited(neighbors_unvisited) = true;
        
        % update distances for neighbor vertex
        for jj=1:num_neighbors
            neighbor_ind = neighbors(jj);
            
            % neighbor's distance from start vertex
            neighbor_distance = vertex_distances(neighbor_ind);
            
            % distance between vertex and neighbor
            edge_distance = edges(vertex_ind, neighbor_ind);
            
            % update neighbor's distance from start vertex if shorter path has
            % been found
            if vertex_distance + edge_distance < neighbor_distance
                vertex_distances(neighbor_ind) = vertex_distance + edge_distance;
                vertex_prev(neighbor_ind) = vertex_ind;
            end
        end
    end
end

% link up the vertices in the shortest path
path = vertices(end, :);
prev = vertex_prev(end);

while ~isnan(prev)
    path = [path; vertices(prev, :)];
    prev = vertex_prev(prev);
end

% need to reverse the path
path = flip(path);
end
%--------------------------------------------------------------------------