%function qrrtnonholo()
    clc; close all;
    % Parameters
    start = [50, 50];
    goal = [950, 950];
    max_iter = 2000;
    step_size = 50;
    goal_threshold = 10;
    search_radius = 200;
    goal_bias = 0.4; % Probability of sampling the goal directly
    min_turn_radius = 5; % Minimum turn radius for the rover
    
    % Obstacles and no-go zones
    obstacles = [
        200, 200, 50, 50;
        400, 400, 100, 100;
        700, 850, 120, 120;
        500, 200, 90, 90;
        700, 500, 70, 70;
        200, 800, 60, 60;
    ];
    no_go_zone_radius = 5;

    % Initialize tree
    nodes = start;
    parents = -1; % Root has no parent
    orientations = 0; % Initial orientation (0 degrees)
    
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
            q_rand = [rand * 1000, rand * 1000];
        end

        if ~isPointInFreeSpace(q_rand, obstacles, no_go_zone_radius)
            continue;
        end

        [q_nearest, q_nearest_idx] = findNearestNode(nodes, q_rand);
        [q_new, orientation_new, is_turn] = steer(q_nearest, q_rand, step_size, orientations(q_nearest_idx), min_turn_radius);

        if ~collisionFree(q_nearest, q_new, obstacles, no_go_zone_radius)
            continue;
        end

        nodes = [nodes; q_new];
        orientations = [orientations; orientation_new];
        new_node_idx = size(nodes, 1);

        % Find parent
        q_min = q_nearest;
        q_min_idx = q_nearest_idx;
        for i = 1:size(nodes, 1) - 1
            if norm(nodes(i, :) - q_new) < search_radius && collisionFree(nodes(i, :), q_new, obstacles, no_go_zone_radius)
                if cost(nodes, parents, i) + norm(nodes(i, :) - q_new) < cost(nodes, parents, q_min_idx) + norm(q_min - q_new)
                    q_min = nodes(i, :);
                    q_min_idx = i;
                end
            end
        end
        parents = [parents; q_min_idx];

        % Rewiring
        for i = 1:size(nodes, 1)
            if i ~= new_node_idx && norm(nodes(i, :) - q_new) < search_radius && collisionFree(q_new, nodes(i, :), obstacles, no_go_zone_radius)
                if cost(nodes, parents, new_node_idx) + norm(q_new - nodes(i, :)) < cost(nodes, parents, i)
                    parents(i) = new_node_idx;
                end
            end
        end

        plot([q_min(1), q_new(1)], [q_min(2), q_new(2)], 'r');
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
        % Extract final path
        path = extractFinalPath(nodes, parents, goal_idx);
        % Calculate turn points and angles on final path
        [turn_points, turn_angles] = calculateTurns(path, nodes, orientations, min_turn_radius);
        % Plot final path and turn points with smoothing
        plotFinalPath(nodes, parents, goal_idx, orientations, turn_points, turn_angles, obstacles, no_go_zone_radius);
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
            obstacles(i, 3) + 2 * no_go_zone_radius, obstacles(i, 4) + 2 * no_go_zone_radius], 'EdgeColor', 'r', 'LineStyle', '--');
    end
end

function [q_nearest, q_nearest_idx] = findNearestNode(nodes, q_rand)
    distances = sqrt(sum((nodes - q_rand) .^ 2, 2));
    [~, q_nearest_idx] = min(distances);
    q_nearest = nodes(q_nearest_idx, :);
end

function [q_new, orientation_new, is_turn] = steer(q_nearest, q_rand, step_size, orientation_nearest, min_turn_radius)
    direction = (q_rand - q_nearest) / norm(q_rand - q_nearest);
    distance = norm(q_rand - q_nearest);
    if distance > step_size
        q_new = q_nearest + step_size * direction;
    else
        q_new = q_rand;
    end
    % Compute orientation
    orientation_new = atan2(q_new(2) - q_nearest(2), q_new(1) - q_nearest(1));
    % Check if a turn is required
    angle_diff = abs(orientation_new - orientation_nearest);
    if angle_diff > pi / 4  % Assuming a significant turn
        is_turn = true;
        % Adjust orientation to respect the minimum turn radius
        if distance > min_turn_radius
            % For simplicity, you might want to adjust the turn point to simulate turning
            q_new = q_nearest + min_turn_radius * direction;
        end
    else
        is_turn = false;
    end
end

function free = collisionFree(q1, q2, obstacles, no_go_zone_radius)
    % Number of steps to divide the path for collision checking
    num_steps = ceil(norm(q2 - q1) / 2); % Increased precision by reducing step size
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
    free = true;
    for i = 1:size(obstacles, 1)
        if point(1) >= obstacles(i, 1) - no_go_zone_radius && ...
                point(1) <= obstacles(i, 1) + obstacles(i, 3) + no_go_zone_radius && ...
                point(2) >= obstacles(i, 2) - no_go_zone_radius && ...
                point(2) <= obstacles(i, 2) + obstacles(i, 4) + no_go_zone_radius
            free = false;
            return;
        end
    end
end

function c = cost(nodes, parents, idx)
    c = 0;
    while idx > 0 && parents(idx) ~= -1
        c = c + norm(nodes(idx, :) - nodes(parents(idx), :));
        idx = parents(idx);
    end
end

function path = extractFinalPath(nodes, parents, goal_idx)
    path = goal_idx;
    while path(1) > 0 && parents(path(1)) ~= -1
        path = [parents(path(1)), path];
    end
end

function [turn_points, turn_angles] = calculateTurns(path, nodes, orientations, min_turn_radius)
    turn_points = [];
    turn_angles = [];
    for i = 2:length(path) - 1
        orientation_prev = orientations(path(i - 1));
        orientation_curr = orientations(path(i));
        angle_diff = abs(orientation_curr - orientation_prev);
        if angle_diff > pi / 4 % Assuming a significant turn
            turn_points = [turn_points; nodes(path(i), :)];
            turn_angles = [turn_angles; angle_diff];
        end
    end
end

function plotFinalPath(nodes, parents, goal_idx, orientations, turn_points, turn_angles, obstacles, no_go_zone_radius)
    % Extract the path
    path = extractFinalPath(nodes, parents, goal_idx);
    
    % Original path plotting
    figure;
    hold on;
    grid on;
    axis([0 1000 0 1000]);
    plotObstacles(obstacles, no_go_zone_radius);
    plot(nodes(:,1), nodes(:,2), 'k.', 'MarkerSize', 10); % Plot all nodes
    plot(nodes(path, 1), nodes(path, 2), 'r-', 'LineWidth', 2); % Plot the original path

    % Extracting the x and y coordinates of the path
    x = nodes(path, 1);
    y = nodes(path, 2);
    
    % Create a parameter t for the path length
    t = [0; cumsum(sqrt(diff(x) .^ 2 + diff(y) .^ 2))];
    
    % Fit cubic splines to x and y coordinates
    t_interp = linspace(0, t(end), 10 * length(t));
    x_spline = spline(t, x, t_interp);
    y_spline = spline(t, y, t_interp);

    % Plot smoothed path
    plot(x_spline, y_spline, 'b-', 'LineWidth', 2);
    
    % Plot turn points
    for i = 1:size(turn_points, 1)
        plot(turn_points(i, 1), turn_points(i, 2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    end
    
    % Mark the start and goal positions
    plot(nodes(path(1), 1), nodes(path(1), 2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(nodes(path(end), 1), nodes(path(end), 2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    title('Smoothed Path Using Cubic Splines');
    xlabel('X');
    ylabel('Y');
    hold off;
end
