% This program implements the Rapidly-exploring Random Tree (RRT) algorithm with obstacle and no-go zone avoidance
function RRT_star
    % Map and obstacle settings
    map_size = [1000, 1000];
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
    no_go_zone = 5;
    
    % Starting and goal positions
    start = [50, 50];
    goal = [950, 950];
    max_iter = 1000;
    step_size = 20;
    radius = 40;
    
    % Initialize the tree
    nodes(1).coord = start;
    nodes(1).cost = 0;
    nodes(1).parent = 0;

    figure;
    hold on;
    axis([0 map_size(1) 0 map_size(2)]);
    
    % Plot obstacles
    for i = 1:size(obstacles, 1)
        rectangle('Position', obstacles(i,:), 'FaceColor', [0.5 0.5 0.5]);
    end
    
    % Plot no-go zones
    for i = 1:size(obstacles, 1)
        rect = obstacles(i,:) + [-no_go_zone, -no_go_zone, 2*no_go_zone, 2*no_go_zone];
        rectangle('Position', rect, 'EdgeColor', 'r', 'LineStyle', '--');
    end
    
    % Plot start and goal points
    plot(start(1), start(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

    % Main RRT* algorithm
    tic;  % Start timer
    for i = 1:max_iter
        % Generate a random sample
        if rand < 0.1
            sample = goal;  % Bias towards the goal
        else
            sample = [rand * map_size(1), rand * map_size(2)];
        end

        % Find the nearest node in the tree
        [nearest_node, nearest_idx] = findNearestNode(nodes, sample);
        
        % Create a new node in the direction of the sample
        new_node.coord = steer(nearest_node.coord, sample, step_size);
        if ~collisionFree(nearest_node.coord, new_node.coord, obstacles, no_go_zone)
            continue;
        end
        
        new_node.cost = nearest_node.cost + euclideanDistance(new_node.coord, nearest_node.coord);
        new_node.parent = nearest_idx;
        
        % Find neighbors within a radius
        neighbors = findNeighbors(nodes, new_node.coord, radius);
        
        % Choose the best parent
        for j = 1:length(neighbors)
            neighbor = nodes(neighbors(j));
            if collisionFree(neighbor.coord, new_node.coord, obstacles, no_go_zone)
                new_cost = neighbor.cost + euclideanDistance(new_node.coord, neighbor.coord);
                if new_cost < new_node.cost
                    new_node.cost = new_cost;
                    new_node.parent = neighbors(j);
                end
            end
        end
        
        % Add the new node to the tree
        nodes = [nodes, new_node];
        plot([nodes(new_node.parent).coord(1), new_node.coord(1)], ...
             [nodes(new_node.parent).coord(2), new_node.coord(2)], 'g');

        % Rewire the tree
        for j = 1:length(neighbors)
            neighbor = nodes(neighbors(j));
            if neighbor.parent ~= 0 && collisionFree(new_node.coord, neighbor.coord, obstacles, no_go_zone)
                new_cost = new_node.cost + euclideanDistance(new_node.coord, neighbor.coord);
                if new_cost < neighbor.cost
                    plot([nodes(neighbor.parent).coord(1), neighbor.coord(1)], ...
                         [nodes(neighbor.parent).coord(2), neighbor.coord(2)], 'w');
                    neighbor.cost = new_cost;
                    neighbor.parent = length(nodes);
                    nodes(neighbors(j)) = neighbor;
                    plot([new_node.coord(1), neighbor.coord(1)], ...
                         [new_node.coord(2), neighbor.coord(2)], 'g');
                end
            end
        end
        
        % Check if we have reached the goal
        if euclideanDistance(new_node.coord, goal) < step_size
            disp('Goal reached');
            break;
        end

        drawnow;
    end
    time_taken = toc;  % End timer
    num_nodes = length(nodes);  % Number of nodes

    % Extract the path
    path = [goal];
    node = nodes(end);
    while node.parent ~= 0
        node = nodes(node.parent);
        path = [node.coord; path];
    end
    path = [start; path];
    
    % Plot the final path
    plot(path(:,1), path(:,2), 'r', 'LineWidth', 2);
    hold off;
    
    % Calculate the path length
    path_length = 0;
    for i = 1:size(path, 1) - 1
        path_length = path_length + euclideanDistance(path(i, :), path(i + 1, :));
    end
    
    % Display results
    fprintf('Time taken: %.2f seconds\n', time_taken);
    fprintf('Number of nodes: %d\n', num_nodes);
    fprintf('Path length: %.2f units\n', path_length);
end

function [nearest_node, nearest_idx] = findNearestNode(nodes, sample)
    min_dist = inf;
    nearest_idx = 0;
    for i = 1:length(nodes)
        dist = euclideanDistance(nodes(i).coord, sample);
        if dist < min_dist
            min_dist = dist;
            nearest_idx = i;
        end
    end
    nearest_node = nodes(nearest_idx);
end

function new_coord = steer(from, to, step_size)
    direction = to - from;
    distance = euclideanDistance(from, to);
    new_coord = from + min(step_size, distance) * (direction / distance);
end

function free = collisionFree(from, to, obstacles, no_go_zone)
    free = true;
    for i = 1:size(obstacles, 1)
        obs = obstacles(i,:) + [-no_go_zone, -no_go_zone, 2*no_go_zone, 2*no_go_zone];
        if pointInRect(to, obs) || pointInRect(from, obs) || lineIntersectsRect(from, to, obs)
            free = false;
            return;
        end
    end
end

function intersects = lineIntersectsRect(p1, p2, rect)
    rect_points = [rect(1), rect(2);
                   rect(1)+rect(3), rect(2);
                   rect(1)+rect(3), rect(2)+rect(4);
                   rect(1), rect(2)+rect(4)];
    for i = 1:4
        p3 = rect_points(i, :);
        p4 = rect_points(mod(i, 4) + 1, :);
        if lineIntersect(p1, p2, p3, p4)
            intersects = true;
            return;
        end
    end
    intersects = false;
end

function intersect = lineIntersect(p1, p2, p3, p4)
    a1 = p2(2) - p1(2);
    b1 = p1(1) - p2(1);
    c1 = a1 * p1(1) + b1 * p1(2);
    a2 = p4(2) - p3(2);
    b2 = p3(1) - p4(1);
    c2 = a2 * p3(1) + b2 * p3(2);
    det = a1 * b2 - a2 * b1;
    if det == 0
        intersect = false;
    else
        x = (b2 * c1 - b1 * c2) / det;
        y = (a1 * c2 - a2 * c1) / det;
        if min(p1(1), p2(1)) <= x && x <= max(p1(1), p2(1)) && ...
           min(p1(2), p2(2)) <= y && y <= max(p1(2), p2(2)) && ...
           min(p3(1), p4(1)) <= x && x <= max(p3(1), p4(1)) && ...
           min(p3(2), p4(2)) <= y && y <= max(p3(2), p4(2))
            intersect = true;
        else
            intersect = false;
        end
    end
end

function inside = pointInRect(point, rect)
    if point(1) >= rect(1) && point(1) <= rect(1) + rect(3) && ...
       point(2) >= rect(2) && point(2) <= rect(2) + rect(4)
        inside = true;
    else
        inside = false;
    end
end

function neighbors = findNeighbors(nodes, coord, radius)
    neighbors = [];
    for i = 1:length(nodes)
        if euclideanDistance(nodes(i).coord, coord) < radius
            neighbors = [neighbors, i];
        end
    end
end

function dist = euclideanDistance(p1, p2)
    sum_squares = 0;
    for i = 1:length(p1)
        sum_squares = sum_squares + (p1(i) - p2(i))^2;
    end
    dist = sqrt(sum_squares);
end
