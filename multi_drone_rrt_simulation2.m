function multi_drone_rrt_simulation
    % Parameters
    start_positions = [-4, -4, 0; -4, 4, 0; 4, -4, 0]; % Starting positions for three drones
    goal_positions = [4, 4, 10; -4, -4, 10; 4, -4, 10]; % Goal positions for three drones
    step_size = 0.5; % Step size for RRT*
    max_iter = 500; % Maximum iterations for RRT*
    goal_tolerance = 0.5; % Goal tolerance
    obstacle_speed = 0.2; % Speed of moving obstacles
    num_obstacles = 15; % Number of obstacles
    filename = 'multi_drone_path.gif'; % Output GIF file name

    % Create figure
    figure('Color', 'w', 'Renderer', 'opengl');
    axis equal
    hold on
    grid on
    view(3)
    xlim([-5 5])
    ylim([-5 5])
    zlim([0 10])

    % Plot start and goal positions
    scatter3(start_positions(:,1), start_positions(:,2), start_positions(:,3), 'g', 'filled')
    scatter3(goal_positions(:,1), goal_positions(:,2), goal_positions(:,3), 'r', 'filled')

    % Initialize obstacles
    obstacles = initialize_obstacles(num_obstacles, [-5, 5], [-5, 5], [0, 10]);
    obstacle_plots = plot_obstacles(obstacles);

    % Perform RRT* path planning for each drone
    paths = cell(3, 1);
    for k = 1:3
        paths{k} = rrt_star(start_positions(k, :), goal_positions(k, :), step_size, max_iter, goal_tolerance, obstacles, obstacle_speed);
    end

    % Plot the paths
    colors = {'k--', 'b--', 'm--'};
    for k = 1:3
        if ~isempty(paths{k})
            plot3(paths{k}(:,1), paths{k}(:,2), paths{k}(:,3), colors{k}, 'LineWidth', 1);
        end
    end

    % Create drone models
    drones = create_drone_model();
    drone_plots = gobjects(3, 1);
    for k = 1:3
        drone_plots(k) = plot3(drones(:, 1) + start_positions(k, 1), ...
                               drones(:, 2) + start_positions(k, 2), ...
                               drones(:, 3) + start_positions(k, 3), 'k', 'LineWidth', 2);
    end

    % Move the drones along their paths and save to GIF
    max_path_length = max(cellfun(@(p) size(p, 1), paths));
    for i = 1:max_path_length
        % Update obstacles
        obstacles = update_obstacles(obstacles, [-5, 5], [-5, 5], [0, 10], obstacle_speed);

        % Update obstacle plots
        for j = 1:length(obstacles)
            switch obstacles(j).shape
                case 'sphere'
                    [x, y, z] = sphere;
                    x = x * obstacles(j).size + obstacles(j).pos(1);
                    y = y * obstacles(j).size + obstacles(j).pos(2);
                    z = z * obstacles(j).size + obstacles(j).pos(3);
                    set(obstacle_plots(j), 'XData', x, 'YData', y, 'ZData', z);
                case 'cube'
                    [x, y, z] = cube(obstacles(j).size);
                    vertices = [x(:), y(:), z(:)] + obstacles(j).pos;
                    set(obstacle_plots(j), 'Vertices', vertices);
                case 'cylinder'
                    [x, y, z] = cylinder(obstacles(j).size / 2);
                    x = x + obstacles(j).pos(1);
                    y = y + obstacles(j).pos(2);
                    z = z * obstacles(j).size + obstacles(j).pos(3);
                    set(obstacle_plots(j), 'XData', x, 'YData', y, 'ZData', z);
            end
        end

        % Update drone positions
        for k = 1:3
            if i <= size(paths{k}, 1)
                set(drone_plots(k), 'XData', drones(:, 1) + paths{k}(i, 1), ...
                                    'YData', drones(:, 2) + paths{k}(i, 2), ...
                                    'ZData', drones(:, 3) + paths{k}(i, 3));
            end
        end

        drawnow;

        % Capture the plot as an image
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);

        % Write to the GIF file
        if i == 1
            imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
        else
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
        end
    end
end

function obstacles = initialize_obstacles(num_obstacles, xlim, ylim, zlim)
    obstacles = struct('pos', [], 'vel', [], 'size', [], 'shape', [], 'pattern', []);
    for i = 1:num_obstacles
        obstacles(i).pos = [rand * (xlim(2) - xlim(1)) + xlim(1), ...
                            rand * (ylim(2) - ylim(1)) + ylim(1), ...
                            rand * (zlim(2) - zlim(1)) + zlim(1)];
        obstacles(i).vel = rand(1, 3) * 0.2 - 0.1; % Random velocity
        obstacles(i).size = rand * 1 + 0.5; % Increase size
        if mod(i, 3) == 0
            obstacles(i).shape = 'sphere';
        elseif mod(i, 3) == 1
            obstacles(i).shape = 'cube';
        else
            obstacles(i).shape = 'cylinder';
        end
        % Define a movement pattern
        if mod(i, 2) == 0
            obstacles(i).pattern = 'circular';
        else
            obstacles(i).pattern = 'linear';
        end
    end
end

function obstacles = update_obstacles(obstacles, xlim, ylim, zlim, obstacle_speed)
    t = now * 100000; % Use current time as a variable to create smooth movement patterns
    for i = 1:length(obstacles)
        if strcmp(obstacles(i).pattern, 'circular')
            theta = obstacle_speed * t;
            obstacles(i).pos(1) = obstacles(i).pos(1) + cos(theta) * 0.1;
            obstacles(i).pos(2) = obstacles(i).pos(2) + sin(theta) * 0.1;
        else
            obstacles(i).pos = obstacles(i).pos + obstacles(i).vel * obstacle_speed;
            if obstacles(i).pos(1) < xlim(1) || obstacles(i).pos(1) > xlim(2)
                obstacles(i).vel(1) = -obstacles(i).vel(1);
            end
            if obstacles(i).pos(2) < ylim(1) || obstacles(i).pos(2) > ylim(2)
                obstacles(i).vel(2) = -obstacles(i).vel(2);
            end
            if obstacles(i).pos(3) < zlim(1) || obstacles(i).pos(3) > zlim(2)
                obstacles(i).vel(3) = -obstacles(i).vel(3);
            end
        end
    end
end

function path = rrt_star(start_pos, goal_pos, step_size, max_iter, goal_tolerance, obstacles, obstacle_speed)
    % RRT* Algorithm
    nodes = start_pos;
    parent = 0;
    goal_reached = false;

    for i = 1:max_iter
        % Update obstacles' positions
        obstacles = update_obstacles(obstacles, [-5, 5], [-5, 5], [0, 10], obstacle_speed);

        % Generate a random sample
        if rand < 0.1
            sample = goal_pos; % Bias towards the goal
        else
            sample = [rand*10-5, rand*10-5, rand*10]; % Random sample in space
        end

        % Find the nearest node
        distances = sqrt(sum((nodes - sample).^2, 2));
        [~, nearest_idx] = min(distances);
        nearest_node = nodes(nearest_idx, :);

        % Create a new node in the direction of the sample
        direction = (sample - nearest_node) / norm(sample - nearest_node);
        new_node = nearest_node + step_size * direction;

        % Check for collisions with obstacles
        if ~check_collision(new_node, obstacles)
            continue;
        end

        % Check if the new node is within the goal tolerance
        if norm(new_node - goal_pos) < goal_tolerance
            goal_reached = true;
            parent = [parent; nearest_idx];
            nodes = [nodes; goal_pos];
            break;
        end

        % Add the new node to the tree
        parent = [parent; nearest_idx];
        nodes = [nodes; new_node];
    end

    if goal_reached
        % Extract the path
        path = [goal_pos];
        node_idx = size(nodes, 1);
        while node_idx ~= 1
            node_idx = parent(node_idx);
            path = [nodes(node_idx, :); path];
        end
    else
        path = [];
    end
end

function collision = check_collision(point, obstacles)
    collision = true;
    for i = 1:length(obstacles)
        if norm(point - obstacles(i).pos) < (obstacles(i).size / 2)
            collision = false;
            return;
        end
    end
end

function drones = create_drone_model()
    % Create a more realistic quadcopter model
    arm_length = 0.5;
    body_width = 0.1;
    drones = [
        -arm_length, 0, 0;
         arm_length, 0, 0;
         NaN, NaN, NaN; % NaN to break the line
         0, -arm_length, 0;
         0, arm_length, 0;
         NaN, NaN, NaN; % NaN to break the line
         -body_width, body_width, 0;
         body_width, body_width, 0;
         body_width, -body_width, 0;
         -body_width, -body_width, 0;
         -body_width, body_width, 0;
    ];
end

function obstacle_plots = plot_obstacles(obstacles)
    obstacle_plots = gobjects(1, length(obstacles));
    for i = 1:length(obstacles)
        switch obstacles(i).shape
            case 'sphere'
                [x, y, z] = sphere;
                x = x * obstacles(i).size + obstacles(i).pos(1);
                y = y * obstacles(i).size + obstacles(i).pos(2);
                z = z * obstacles(i).size + obstacles(i).pos(3);
                obstacle_plots(i) = surf(x, y, z, 'FaceColor', 'r', 'EdgeColor', 'none');
            case 'cube'
                [x, y, z] = cube(obstacles(i).size);
                vertices = [x(:), y(:), z(:)] + obstacles(i).pos;
                obstacle_plots(i) = patch('Vertices', vertices, ...
                    'Faces', [1,2,6,5; 2,3,7,6; 3,4,8,7; 1,4,8,5; 1,2,3,4; 5,6,7,8], ...
                    'FaceColor', 'r', 'EdgeColor', 'none');
            case 'cylinder'
                [x, y, z] = cylinder(obstacles(i).size / 2);
                x = x + obstacles(i).pos(1);
                y = y + obstacles(i).pos(2);
                z = z * obstacles(i).size + obstacles(i).pos(3);
                obstacle_plots(i) = surf(x, y, z, 'FaceColor', 'r', 'EdgeColor', 'none');
        end
    end
end

function [x, y, z] = cube(size)
    x = [-1, 1, 1, -1, -1, 1, 1, -1] * size / 2;
    y = [-1, -1, 1, 1, -1, -1, 1, 1] * size / 2;
    z = [-1, -1, -1, -1, 1, 1, 1, 1] * size / 2;
end