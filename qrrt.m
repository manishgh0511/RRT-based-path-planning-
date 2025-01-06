%function qrrtimproved()
    % Parameters
    clc;
    clear all;
    close all;
    start = [50, 50];
    goal = [950, 950];
    max_iter = 2000;
    step_size = 50;
    goal_threshold = 30;
    search_radius = 200;
    goal_bias = 0.4; % Probability of sampling the goal directly

    % Obstacles and no-go zones
    obstacles = [
        200, 200, 50, 50;
        400, 400, 100, 100;
        600, 400, 80, 80;
        300, 700, 60, 60;
        700, 850, 120, 120;
        500, 200, 90, 90;
        700, 500, 70, 70;
        200, 800, 60, 60;
    ];
    no_go_zone_radius = 5;

    % Initialize tree
    nodes = start;
    parents = -1; % Root has no parent

    figure(1);
    hold on;
    grid on;
    axis([0 1000 0 1000]);
    plotObstacles(obstacles, no_go_zone_radius);
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

    % Measure the start time
    tic;

    % Main loop
    for iter = 1:max_iter
        % Bias sampling towards the goal
        if rand < goal_bias
            q_rand = goal;
        else
            q_rand = [rand*1000, rand*1000];
        end

        if ~isPointInFreeSpace(q_rand, obstacles, no_go_zone_radius)
            continue;
        end

        [q_nearest, q_nearest_idx] = findNearestNode(nodes, q_rand);
        q_new = steer(q_nearest, q_rand, step_size);

        if ~collisionFree(q_nearest, q_new, obstacles, no_go_zone_radius)
            continue;
        end

        nodes = [nodes; q_new];
        new_node_idx = size(nodes, 1);

        % Find parent
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
        parents = [parents; q_min_idx];

        % Rewiring
        for i = 1:size(nodes, 1)
            if i ~= new_node_idx && norm(nodes(i, :) - q_new) < search_radius && ...
                    collisionFree(q_new, nodes(i, :), obstacles, no_go_zone_radius)
                if cost(nodes, parents, new_node_idx) + norm(q_new - nodes(i, :)) < ...
                        cost(nodes, parents, i)
                    parents(i) = new_node_idx;
                end
            end
        end

        plot([q_min(1), q_new(1)], [q_min(2), q_new(2)], 'b');
        drawnow;

        if norm(q_new - goal) < goal_threshold
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
    disp(['Path length: ', num2str(cost(nodes, parents, goal_idx))]);
    disp(['Time required to reach the goal: ', num2str(elapsed_time), ' seconds']);
%end

function plotObstacles(obstacles, no_go_zone_radius)
    for i = 1:size(obstacles, 1)
        rectangle('Position', obstacles(i, :), 'FaceColor', [0 0 0]);
        rectangle('Position', [obstacles(i, 1) - no_go_zone_radius, obstacles(i, 2) - no_go_zone_radius, ...
            obstacles(i, 3) + 2*no_go_zone_radius, obstacles(i, 4) + 2*no_go_zone_radius], 'EdgeColor', 'r', 'LineStyle', '--');
    end
end

function [q_nearest, q_nearest_idx] = findNearestNode(nodes, q_rand)
    distances = sqrt(sum((nodes - q_rand).^2, 2));
    [~, q_nearest_idx] = min(distances);
    q_nearest = nodes(q_nearest_idx, :);
end

function q_new = steer(q_nearest, q_rand, step_size)
    direction = (q_rand - q_nearest) / norm(q_rand - q_nearest);
    q_new = q_nearest + step_size * direction;
end

function free = collisionFree(q1, q2, obstacles, no_go_zone_radius)
    % Number of steps to divide the path for collision checking
    num_steps = ceil(norm(q2 - q1) / 5); % Adjust step size for more precision
    step_vector = (q2 - q1) / num_steps;
    
    for step = 0:num_steps
        point = q1 + step * step_vector;
        if ~isPointInFreeSpace(point, obstacles, no_go_zone_radius)
            free = false;
            return;
        end
    end
    
    free = true;
end

function free = isPointInFreeSpace(point, obstacles, no_go_zone_radius)
    for i = 1:size(obstacles, 1)
        if point(1) >= obstacles(i, 1) - no_go_zone_radius && ...
                point(1) <= obstacles(i, 1) + obstacles(i, 3) + no_go_zone_radius && ...
                point(2) >= obstacles(i, 2) - no_go_zone_radius && ...
                point(2) <= obstacles(i, 2) + obstacles(i, 4) + no_go_zone_radius
            free = false;
            return;
        end
    end
    free = true;
end

function c = cost(nodes, parents, idx)
    c = 0;
    while idx > 0 && parents(idx) ~= -1
        c = c + norm(nodes(idx, :) - nodes(parents(idx), :));
        idx = parents(idx);
    end
end

function plotFinalPath(nodes, parents, goal_idx)
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
