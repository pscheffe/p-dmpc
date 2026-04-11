function loop_result = mcts_expand_loop_matlab( ...
        n_expansions_max, ...
        Hp, ...
        root_pose, ...
        reference_trajectory_points, ...
        random_numbers, ...
        all_successor_trims, ...
        maneuvers, ...
        trims, ...
        parents, ...
        children, ...
        shapes_tmp, ...
        n_nodes, ...
        constraint_checker ...
    )
    % MCTS_EXPAND_LOOP_MATLAB  MATLAB fallback for the MCTS expansion loop.

    n_expansions = 0;
    n_traversals = 0;
    is_finished = false;

    best_node_id = -1;
    best_cost = inf;

    while (n_expansions < n_expansions_max) && ~is_finished
        % Expand randomly for Hp steps.
        node_id = 1;
        solution_cost = 0;
        node_pose = root_pose;
        is_valid = false;

        for i_step = 1:Hp
            is_valid = false;
            n_traversals = n_traversals + 1;

            % Select node to expand randomly.
            trim_positions = find(children(:, node_id));
            n_trims = numel(trim_positions);

            if n_trims ~= 0
                child_position = trim_positions(ceil(random_numbers(n_traversals) * n_trims));
            else

                if node_id ~= 1
                    % Remove edge to node without children.
                    parent_id = parents(node_id);
                    children(children(:, parent_id) == node_id, parent_id) = 0;
                    break
                else
                    is_finished = true;
                    break
                end

            end

            parent_trim = trims(node_id);
            successor_trims = all_successor_trims{parent_trim, i_step};
            goal_trim = successor_trims(child_position);

            maneuver = maneuvers{parent_trim, goal_trim};

            c = cos(node_pose(3));
            s = sin(node_pose(3));

            transform = [c, -s, 0;
                         s, c, 0;
                         0, 0, 1];

            start_pose = node_pose;
            node_pose = node_pose + transform * maneuver.dpose;

            % Cost-to-come term for distance from reference trajectory.
            solution_cost = solution_cost + norm(node_pose(1:2) - reference_trajectory_points(:, i_step))^2;

            is_expanded = children(child_position, node_id) ~= 1;

            if is_expanded
                node_id = children(child_position, node_id);
                continue
            end

            n_expansions = n_expansions + 1;
            node_parent = node_id;

            shapes_without_offset = {transform(1:2, 1:2) * maneuver.area_without_offset + start_pose(1:2)};
            shapes = {transform(1:2, 1:2) * maneuver.area + start_pose(1:2)};

            if i_step ~= Hp
                shapes_for_boundary_check = shapes_without_offset;
                child_successor_trims = all_successor_trims{goal_trim, i_step + 1};
            else
                shapes_for_boundary_check = {transform(1:2, 1:2) * maneuver.area_large_offset + start_pose(1:2)};
                child_successor_trims = [];
            end

            is_valid = constraint_checker(shapes, shapes_for_boundary_check, i_step);

            if ~is_valid
                % Remove invalid edge.
                children(child_position, node_parent) = 0;
                break
            end

            % Add node.
            n_nodes = n_nodes + 1;
            parents(1, n_nodes) = node_parent;
            trims(:, n_nodes) = goal_trim;
            children(1:size(child_successor_trims, 2), n_nodes) = 1;
            children(child_position, node_parent) = n_nodes;
            shapes_tmp(:, n_nodes) = shapes;
            node_id = n_nodes;
        end

        if is_valid

            if solution_cost < best_cost
                best_cost = solution_cost;
                best_node_id = double(node_id);
            end

            % Avoid double exploration.
            children(child_position, node_parent) = 0;
        end

    end

    loop_result = struct();
    loop_result.trims = trims;
    loop_result.parents = parents;
    loop_result.children = children;
    loop_result.shapes_tmp = shapes_tmp;
    loop_result.n_nodes = n_nodes;
    loop_result.n_expansions = n_expansions;
    loop_result.best_node_id = best_node_id;
    loop_result.best_cost = best_cost;

end
