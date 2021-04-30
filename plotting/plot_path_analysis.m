%--------------------------------------------------------------------------
% purpose: path plot analysis
%  input:    true_hist = history of true states
%            est_hist = history of estimated states
%           pos_final = desired position
%          free_space = collection of valid 2D positions for the robot
%             occ_map = 2D binary occupancy map
%            vertices = global path planning graph vertices
%               edges = global path planning graph edges
%                path = global path planning desired path
%           algorithm = string name of algorithm used
%                 fig = figure
% output:         fig = figure
%--------------------------------------------------------------------------
function [fig] = plot_path_analysis(true_hist, est_hist, pos_final,...
    free_space, occ_map, vertices, edges, path, algorithm, fig)
figure(fig);
clf(fig);
show(occ_map);
hold on;

plot_ind = 1;
plots(plot_ind) = plot(free_space(:,1), free_space(:,2), 'y.');
plot_labels{plot_ind} = 'free space';

if ~isempty(edges)
    plot_ind = plot_ind + 1;
    for ii=1:size(edges, 1)
        for jj=ii:size(edges, 1)
            if ~isinf(edges(ii,jj))
                plots(plot_ind) = plot([vertices(ii,1) vertices(jj,1)], [vertices(ii,2) vertices(jj,2)], 'c');
                plot_labels{plot_ind} = 'graph edges';
            end
        end
    end
end

plot_ind = plot_ind + 1;
plots(plot_ind) = plot(true_hist(1,:), true_hist(2,:), 'g', 'LineWidth', 2);
plot_labels{plot_ind} = 'true path';

plot_ind = plot_ind + 1;
plots(plot_ind) = plot(est_hist(1,:), est_hist(2,:), 'r', 'LineWidth', 2);
plot_labels{plot_ind} = 'estimated path';

if size(path, 2) > 1
    plot_ind = plot_ind + 1;
    plots(plot_ind) = plot(path(1,:), path(2,:), 'b', 'LineWidth', 2);
    plot_labels{plot_ind} = 'desired path';
    
    desired_length = sum(vecnorm(diff(path, 1, 2)));
    desired_length_text = sprintf('desired path length = %0.3fm', desired_length);
    text(-1, 10, desired_length_text);
end

plot_ind = plot_ind + 1;
plots(plot_ind) = plot(true_hist(1,1), true_hist(2,1), 'm*', 'MarkerSize', 10, 'LineWidth', 2);
plot_labels{plot_ind} = 'source position';

plot_ind = plot_ind + 1;
plots(plot_ind) = plot(pos_final(1), pos_final(2), 'mp', 'MarkerSize', 10, 'LineWidth', 2);
plot_labels{plot_ind} = 'sink position';

true_length = sum(vecnorm(diff(true_hist(1:2,:), 1, 2)));
true_length_text = sprintf('true path length = %0.03fm', true_length);
text(-2, 10, true_length_text);

pos_error = vecnorm(true_hist(1:2, end) - pos_final');
pos_error_text = sprintf('final position error = %0.03fm', pos_error);
text(-3, 10, pos_error_text);

xlabel('X');
ylabel('Y');
title([algorithm, ' path results']);
legend(plots, plot_labels, 'Location', 'east');
view(-90,90);
axis([-5 11 -8 11]);
hold off;
end
%--------------------------------------------------------------------------