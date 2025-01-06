% This program implements the Informed + Quick Rapidly-exploring Random Tree Star (I-QRRT*) algorithm with obstacle and no-go zone avoidance
clc; clear all; close all;
    % Parameters
    start = [50, 50];
    goal = [950, 950];
    max_iter = 2000;  % Maximum number of iterations for the algorithm
    step_size = 50; % Step size for moving from one point to another
    goal_threshold = 30;  % Threshold distance for considering the goal as reached
    search_radius = 60; % Radius for searching nearby nodes during the rewiring process
    goal_bias = 0.6;  % Probability of sampling directly towards the goal

    % Obstacles and no-go zones
    obstacles = [ % Array defining the position and size of rectangular obstacles
        200, 200, 50, 50;
        400, 400, 100, 100;
        600, 400, 80, 80;
        300, 700, 60, 60;
        700, 850, 120, 120;
        500, 200, 90, 90;
        700, 500, 70, 70;
        200, 800, 60, 60;
    ];
    no_go_zone_radius = 5; % Radius around obstacles that the algorithm should avoid

    % Initialize tree
    nodes = start;
    parents = -1;  % Root node has no parent (indicated by -1)

    % Plot obstacles and start/goal
    figure(1);
    hold on;
    grid on;
    axis([0 1000 0 1000]); % Set axis limits for the plot
    plotObstacles(obstacles, no_go_zone_radius); % Function to plot obstacles
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

    % Calculate ellipsoid parameters for informed sampling
    c_min = norm(start - goal);  % Minimum cost (straight-line distance) from start to goal
    center = (start + goal) / 2;
    rotation = atan2(goal(2) - start(2), goal(1) - start(1)); % Rotation angle of the ellipsoid
    a = c_min / 2;  % Semi-major axis
    b = a / 4;  % Semi-minor axis

    % Plot the ellipsoid
    theta = linspace(0, 2*pi, 100);  % Angle for ellipse parameterization
    ellipse_x = center(1) + a * cos(theta) * cos(rotation) - b * sin(theta) * sin(rotation);
    ellipse_y = center(2) + a * cos(theta) * sin(rotation) + b * sin(theta) * cos(rotation);
    plot(ellipse_x, ellipse_y, 'k--');

    % Measure the start time
    tic;

    % Main loop
    for iter = 1:max_iter
        % Bias sampling towards the goal or within ellipsoid
        q_rand = sample_with_goal_bias(goal, goal_bias, obstacles, no_go_zone_radius, center, a, b, rotation);

        % Check if the sampled point is valid (i.e., not inside an obstacle or no-go zone)
        if ~isPointValid(q_rand, obstacles, no_go_zone_radius)
            continue;
        end
        
        % Find the nearest node in the tree to the sampled point
        [q_nearest, q_nearest_idx] = findNearestNode(nodes, q_rand);
        
        % Debug: Check size and validity of q_nearest and q_nearest_idx
        if isempty(q_nearest) || q_nearest_idx <= 0 || q_nearest_idx > size(nodes, 1)
            error('Invalid nearest node or index.');
        end
        
        q_new = steer(q_nearest, q_rand, step_size); % Steer from the nearest node towards the sampled point

        if ~collisionFree(q_nearest, q_new, obstacles, no_go_zone_radius) % Check if the path from the nearest node to the new node is collision-free
            continue;
        end

        % Append the new node to the list of nodes
        nodes = [nodes; q_new];
        new_node_idx = size(nodes, 1); % Index of the newly added node

        % Ensure parents array is updated correctly
        parents = [parents; q_nearest_idx];

        % Find the best parent (node that minimizes the cost)
        q_min = q_nearest;
        q_min_idx = q_nearest_idx;
        for i = 1:size(nodes, 1)-1
            if norm(nodes(i, :) - q_new) < search_radius && ...
                    collisionFree(nodes(i, :), q_new, obstacles, no_go_zone_radius)
                if cost(nodes, parents, i) + norm(nodes(i, :) - q_new) < ...
                        cost(nodes, parents, q_min_idx) + norm(q_min - q_new)
                    q_min = nodes(i, :);
                    q_min_idx = i;
                end
            end
        end

        % Debug: Check if parent index is valid
        if q_min_idx <= 0 || q_min_idx > size(parents, 1)
            error('Invalid parent index.');
        end

        % Rewiring: Update the parent of nearby nodes if a shorter path is found
        for i = 1:size(nodes, 1)
            if i ~= new_node_idx && norm(nodes(i, :) - q_new) < search_radius && ...
                    collisionFree(q_new, nodes(i, :), obstacles, no_go_zone_radius)
                if cost(nodes, parents, new_node_idx) + norm(q_new - nodes(i, :)) < ...
                        cost(nodes, parents, i)
                    parents(i) = new_node_idx;
                end
            end
        end

        % Prune nodes that are too far from the goal to reduce oversampling
        if mod(iter, 100) == 0
            [nodes, parents] = pruneNodes(nodes, parents, goal, search_radius);
        end

        % Debug: Check size of nodes and parents after pruning
        if isempty(nodes) || isempty(parents)
            error('Nodes or parents array is empty after pruning.');
        end

        plot([q_min(1), q_new(1)], [q_min(2), q_new(2)], 'r'); % Plot the newly added edge
        drawnow;

        if norm(q_new - goal) < goal_threshold  % Check if the goal has been reached
            break;
        end
    end

    % Measure the end time
    elapsed_time = toc;

    % Check if the goal was reached and plot the final path if so
    [~, goal_idx] = findNearestNode(nodes, goal);
    if norm(nodes(goal_idx, :) - goal) < goal_threshold
        plotFinalPath(nodes, parents, goal_idx);
    else
        disp('Goal not reached');
    end

    % Display metrics
    disp(['Number of nodes: ', num2str(size(nodes, 1))]);
    if goal_idx > 0
        disp(['Path length: ', num2str(cost(nodes, parents, goal_idx))]);
    else
        disp('Path length: N/A');
    end
    disp(['Time required to reach the goal: ', num2str(elapsed_time), ' seconds']);
%end

function plotObstacles(obstacles, no_go_zone_radius) % Function to plot obstacles and their no-go zones
    if isempty(obstacles)
        error('Obstacles array is empty.');
    end
    for i = 1:size(obstacles, 1)
        rectangle('Position', obstacles(i, :), 'FaceColor', [0 0 0]);% Plot the obstacle as a filled rectangle
        rectangle('Position', [obstacles(i, 1) - no_go_zone_radius, obstacles(i, 2) - no_go_zone_radius, ...
            obstacles(i, 3) + 2*no_go_zone_radius, obstacles(i, 4) + 2*no_go_zone_radius], 'EdgeColor', 'r', 'LineStyle', '--'); % Plot the no-go zone as a dashed rectangle around the obstacle
    end
end

function [q_nearest, q_nearest_idx] = findNearestNode(nodes, q_rand) % Function to find the nearest node in the tree to a given point
    if isempty(nodes)
        error('Nodes array is empty.');
    end
    distances = sqrt(sum((nodes - q_rand).^2, 2));  % Compute Euclidean distances
    if isempty(distances)
        error('Distances array is empty.');
    end
    [~, q_nearest_idx] = min(distances);  % Find the index of the nearest node
    if q_nearest_idx <= 0 || q_nearest_idx > size(nodes, 1)
        error('Nearest node index is out of bounds.');
    end
    q_nearest = nodes(q_nearest_idx, :); % Return the nearest node
end

function q_new = steer(q_nearest, q_rand, step_size) % Function to steer from a point towards another with a given step size
    direction = (q_rand - q_nearest) / norm(q_rand - q_nearest); % Compute the direction
    q_new = q_nearest + step_size * direction; % Calculate the new point
end

function free = collisionFree(q1, q2, obstacles, no_go_zone_radius) % Function to check if a point is valid (not inside an obstacle or no-go zone)
    if isempty(obstacles)
        error('Obstacles array is empty.');
    end
    num_steps = ceil(norm(q2 - q1) / 5);
    if num_steps <= 0
        num_steps = 1; % Ensure at least one step
    end
    step_vector = (q2 - q1) / num_steps;
    
    for step = 0:num_steps
        point = q1 + step * step_vector;
        if ~isPointValid(point, obstacles, no_go_zone_radius)
            free = false;
            return;
        end
    end
    
    free = true;
end

function valid = isPointValid(point, obstacles, no_go_zone_radius)
    if isempty(obstacles)
        error('Obstacles array is empty.');
    end
    if isempty(point) || length(point) ~= 2
        error('Point is invalid.');
    end
    valid = true;
    for i = 1:size(obstacles, 1)
        if point(1) >= obstacles(i, 1) - no_go_zone_radius && ...
                point(1) <= obstacles(i, 1) + obstacles(i, 3) + no_go_zone_radius && ...
                point(2) >= obstacles(i, 2) - no_go_zone_radius && ...
                point(2) <= obstacles(i, 2) + obstacles(i, 4) + no_go_zone_radius
            valid = false;
            return;
        end
    end
end

function c = cost(nodes, parents, idx) % Function to calculate the cost (distance) of the path from start to a node
    if idx <= 0 || idx > length(parents)
        error('Index is out of bounds for parents array.');
    end
    c = 0;
    while idx > 0 && parents(idx) ~= -1
        c = c + norm(nodes(idx, :) - nodes(parents(idx), :));
        idx = parents(idx);
    end
end

function plotFinalPath(nodes, parents, goal_idx)
    if isempty(nodes) || isempty(parents)
        error('Nodes or parents array is empty.');
    end
    path = goal_idx;
    while path(1) > 0 && parents(path(1)) ~= -1
        path = [parents(path(1)), path];
    end
    if length(path) > 1
        for i = 2:length(path)
            plot([nodes(path(i-1), 1), nodes(path(i), 1)], [nodes(path(i-1), 2), nodes(path(i), 2)], 'g', 'LineWidth', 2);
        end
        plot(nodes(path, 1), nodes(path, 2), 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
    end
end

function sample = sample_with_goal_bias(goal, goalBias, obstacles, no_go_zone_radius, center, a, b, rotation)
    if rand() < goalBias
        sample = goal;
    else
        while true
            theta = 2 * pi * rand();
            r = sqrt(rand());
            sample = center + r * [a * cos(theta) * cos(rotation) - b * sin(theta) * sin(rotation), ...
                                   a * cos(theta) * sin(rotation) + b * sin(theta) * cos(rotation)];
            if isPointValid(sample, obstacles, no_go_zone_radius)
                break;
            end
        end
    end
end

function [nodes, parents] = pruneNodes(nodes, parents, goal, search_radius)
    % Check for empty input arrays
    if isempty(nodes) || isempty(parents)
        error('Nodes or parents array is empty.');
    end

    % Determine which nodes to remove based on distance from the goal
    to_remove = false(size(nodes, 1), 1);
    for i = 1:size(nodes, 1)
        if norm(nodes(i, :) - goal) > 2 * search_radius
            to_remove(i) = true;
        end
    end

    % Check if there are nodes to be removed
    if any(to_remove)
        % Ensure there are nodes left after pruning
        keep_indices = ~to_remove;
        if sum(keep_indices) <= 1
            % Ensure at least one node is kept
            warning('Pruning would result in too few nodes. No nodes will be removed.');
            return;
        end
        
        % Create new nodes array excluding the nodes to be removed
        nodes = nodes(keep_indices, :);
        
        % Create a mapping from old indices to new indices
        old_to_new_index = zeros(size(nodes, 1), 1);
        old_to_new_index(keep_indices) = 1:sum(keep_indices);
        
        % Update the parents array
        new_parents = parents;
        
        % Adjust parent indices
        for i = 1:length(parents)
            if parents(i) > 0
                % Map old parent index to new parent index
                if parents(i) <= length(old_to_new_index) && keep_indices(parents(i))
                    new_parents(i) = old_to_new_index(parents(i));
                else
                    new_parents(i) = -1; % Parent was removed or was invalid
                end
            end
        end
        
        % Ensure root node index stays -1
        new_parents(parents == -1) = -1;
        
        % Update the parents array
        parents = new_parents;
    end

    % Debugging output
    disp(['Number of nodes after pruning: ', num2str(size(nodes, 1))]);
    disp(['Number of parents after pruning: ', num2str(length(parents))]);
end
