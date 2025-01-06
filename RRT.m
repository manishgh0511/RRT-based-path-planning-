% This program implements the Rapidly-exploring Random Tree (RRT) algorithm with obstacle and no-go zone avoidance
    clc;          
    clear all;    
    close all;   
    
    % Define the size of the grid
    grid_size = 1000;
    
    % Define obstacles as an array where each row is [x_center, y_center, width, height]
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
    
    % Define the radius for no-go zones around each obstacle
    no_go_zone_radius = 10;

    % Define the start and goal positions on the grid
    start = [50, 50];
    goal = [950, 950];
    
    % Define the maximum number of iterations for the RRT algorithm
    max_iter = 10000;
    
    % RRT parameters
    delta_q = 10;     % Step size for each move in the RRT
    max_dist = 20;    % Maximum allowable distance between nodes in the tree
    
    % Plot the grid and obstacles
    figure;
    hold on;
    axis([0 grid_size 0 grid_size]);  % Set the axis limits
    
    % Draw obstacles and no-go zones
    for i = 1:size(obstacles, 1)
        % Calculate and draw the no-go zone (light gray)
        x_no_go = obstacles(i, 1) - obstacles(i, 3) / 2 - no_go_zone_radius;
        y_no_go = obstacles(i, 2) - obstacles(i, 4) / 2 - no_go_zone_radius;
        w_no_go = obstacles(i, 3) + 2 * no_go_zone_radius;
        h_no_go = obstacles(i, 4) + 2 * no_go_zone_radius;
        rectangle('Position', [x_no_go, y_no_go, w_no_go, h_no_go], 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'k'); 
        
        % Calculate and draw the obstacle (gray)
        x_obs = obstacles(i, 1) - obstacles(i, 3) / 2;
        y_obs = obstacles(i, 2) - obstacles(i, 4) / 2;
        rectangle('Position', [x_obs, y_obs, obstacles(i, 3), obstacles(i, 4)], 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'k'); 
    end
    
    % Highlight the start and goal positions
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');  % Start point in green
    plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');    % Goal point in red
    
    % Initialize the RRT algorithm by starting with the root node at the start position
    tree.vertices = start;
    tree.edges = [];
    start_time = cputime;  % Record the start time for performance measurement
    
    for iter = 1:max_iter
        % Randomly sample a point within the grid
        q_rand = [randi(grid_size), randi(grid_size)];
        
        % Find the nearest vertex in the current tree to the sampled point
        [q_near, idx_near] = findNearest(tree.vertices, q_rand);
        
        % Steer towards the random point from the nearest vertex by a defined step size
        q_new = steer(q_near, q_rand, delta_q);
        
        % Check if the new point is in collision with any obstacle or no-go zone
        if ~inCollision(q_new, obstacles, no_go_zone_radius)
            % If no collision, add the new point to the tree
            tree.vertices = [tree.vertices; q_new];
            tree.edges = [tree.edges; idx_near, size(tree.vertices, 1)];
            
            % Plot the new edge in the tree
            plot([q_near(1), q_new(1)], [q_near(2), q_new(2)], 'b');
            drawnow;
            
            % Check if the goal is reached
            if calculateDistance(q_new, goal) < max_dist
                disp('Goal reached!');
                break;
            end
        end
    end
    end_time = cputime;  % Record the end time for performance measurement
    
    % Plot the final path from the start to the goal
    path = [goal];
    idx = size(tree.vertices, 1);
    while idx > 1
        path = [tree.vertices(idx, :); path];
        idx = tree.edges(tree.edges(:, 2) == idx, 1);
    end
    path = [start; path];
    plot(path(:, 1), path(:, 2), 'm', 'LineWidth', 2);  % Final path in magenta
    
    % Calculate the total length of the path
    path_length = 0;
    for i = 1:size(path, 1) - 1
        path_length = path_length + calculateDistance(path(i, :), path(i + 1, :));
    end
    
    % Calculate the total time taken by the algorithm
    time_taken = end_time - start_time;
    
    % Display the results
    fprintf('Time taken: %.2f seconds\n', time_taken);
    fprintf('Number of nodes: %d\n', size(tree.vertices, 1));
    fprintf('Length of the path: %.2f units\n', path_length);
    
    hold off;
%end

% Helper function to find the nearest vertex to a random point
function [nearest, idx_near] = findNearest(vertices, q_rand)
    min_dist = inf;
    idx_near = 0;
    for i = 1:size(vertices, 1)
        dist = calculateDistance(vertices(i, :), q_rand);
        if dist < min_dist
            min_dist = dist;
            idx_near = i;
        end
    end
    nearest = vertices(idx_near, :);
end

% Helper function to steer from one point towards another by a fixed step size
function q_new = steer(q_near, q_rand, delta_q)
    direction = (q_rand - q_near) / calculateDistance(q_near, q_rand);
    q_new = q_near + delta_q * direction;
end

% Helper function to check if a point is in collision with any obstacle or no-go zone
function collision = inCollision(point, obstacles, radius)
    collision = false;
    for i = 1:size(obstacles, 1)
        x = obstacles(i, 1);
        y = obstacles(i, 2);
        w = obstacles(i, 3) / 2 + radius;
        h = obstacles(i, 4) / 2 + radius;
        if point(1) >= x - w && point(1) <= x + w && point(2) >= y - h && point(2) <= y + h
            collision = true;
            return;
        end
    end
end

% Helper function to calculate the Euclidean distance between two points
function dist = calculateDistance(point1, point2)
    dist = sqrt((point1(1) - point2(1))^2 + (point1(2) - point2(2))^2);
end
