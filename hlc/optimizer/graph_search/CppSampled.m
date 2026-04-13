classdef CppSampled < OptimizerInterface

    properties
        set_up_constraints (1, 1) function_handle = @()[]; % required by factory, unused
        are_constraints_satisfied (1, 1) function_handle = @()[]; % required by factory, unused
        random_numbers (1, :) double = [];
        rand_stream (1, 1) RandStream = RandStream('mt19937ar', Seed = 42);
        n_expansions_max (1, 1) double = 250;
    end

    methods

        function obj = CppSampled()
            obj = obj@OptimizerInterface();
            obj.rand_stream = RandStream('mt19937ar', Seed = 42);
            % Fail hard if MEX not available: CppSampled requires native C++ loop
            assert(exist('mcts_expand_loop_mex', 'file') == 3, ...
            'CppSampled requires mcts_expand_loop_mex MEX binary. Build with build_mcts_expand_loop_mex().');
        end

        function info_v = run_optimizer(obj, vehicle_index, iter, mpa, ~, time_step)
            obj.rand_stream = RandStream('mt19937ar', Seed = time_step + vehicle_index);
            info_v = obj.do_graph_search(iter, mpa);
        end

    end

    methods (Access = private)

        function info = do_graph_search(obj, iter, mpa)
            assert(iter.amount == 1);
            Hp = size(iter.v_ref, 2);
            obj.random_numbers = rand(obj.rand_stream, 1, Hp * obj.n_expansions_max);
            n_successor_trims_max = mpa.maximum_branching_factor();
            info = ControlResultsInfo(iter.amount, Hp);

            all_successor_trims = mpa.successor_trims;

            trim = iter.trim_indices;
            root_successor_trims = all_successor_trims{trim, 1};
            max_nodes = obj.n_expansions_max + 1;
            trims = zeros(1, max_nodes, 'uint8');
            parents = zeros(1, max_nodes, 'uint32');
            children = zeros(n_successor_trims_max, max_nodes, 'uint32');

            root_pose = iter.x0(1:3)';
            trims(1) = trim;
            parents(1) = 0;
            children(1:size(root_successor_trims, 2), 1) = 1;
            n_nodes = 1;

            shapes_tmp = cell(iter.amount, max_nodes);

            % CppSampled uses native InterX path only: no MATLAB callbacks
            [vehicle_obstacles, hdv_obstacles, lanelet_boundary] = obj.set_up_constraints(iter, Hp);

            iVeh = 1;
            reference_trajectory_points = squeeze(iter.reference_trajectory_points(iVeh, :, 1:2))';
            maneuvers = mpa.maneuvers;

            % Set up InterX payload for native C++ constraint checking
            constraint_payload = struct( ...
                'vehicle_obstacles', {vehicle_obstacles}, ...
                'hdv_obstacles', {hdv_obstacles}, ...
                'lanelet_boundary', lanelet_boundary ...
            );

            % Call MEX directly: CppSampled always uses native InterX path
            loop_result = mcts_expand_loop_mex( ...
                obj.n_expansions_max, ...
                Hp, ...
                root_pose, ...
                reference_trajectory_points, ...
                obj.random_numbers, ...
                all_successor_trims, ...
                maneuvers, ...
                trims, ...
                parents, ...
                children, ...
                shapes_tmp, ...
                n_nodes, ...
                constraint_payload ...
            );

            trims = loop_result.trims;
            parents = loop_result.parents;
            n_nodes = loop_result.n_nodes;
            n_expansions = loop_result.n_expansions;
            best_node_id = loop_result.best_node_id;
            cost = loop_result.best_cost;

            info.n_expanded = n_expansions;

            if (best_node_id == -1)
                info.is_exhausted = true;
                return
            end

            tree = Tree();
            tree.trim = trims(1, 1:n_nodes);
            tree.parent = parents(1, 1:n_nodes);
            tree.k = -1 * ones(1, n_nodes);
            tree.g = -1 * ones(1, n_nodes);
            tree.h = -1 * ones(1, n_nodes);

            final_nodes = fliplr(tree.path_to_root(best_node_id));
            pose = zeros(3, length(final_nodes));
            pose(:, 1) = root_pose;

            for i = 2:length(final_nodes)
                start_trim = tree.trim(1, final_nodes(i - 1));
                goal_trim = tree.trim(1, final_nodes(i));

                maneuver = maneuvers{start_trim, goal_trim};

                c = cos(pose(3, i - 1));
                s = sin(pose(3, i - 1));

                transform = [c, -s, 0;
                             s, c, 0;
                             0, 0, 1];
                pose(:, i) = pose(:, i - 1) + transform * maneuver.dpose;
            end

            tree.x(1, final_nodes) = pose(1, :);
            tree.y(1, final_nodes) = pose(2, :);
            tree.yaw(1, final_nodes) = pose(3, :);
            tree.k(1, final_nodes) = 1:length(final_nodes);

            tree.g(1, best_node_id) = cost;

            info.y_predicted = return_path_to(best_node_id, tree);
            info.shapes = return_path_area(loop_result.shapes_tmp, tree, best_node_id);
            info.tree_path = final_nodes;
            info.predicted_trims = squeeze(double([tree.trim(1, info.tree_path(2:end))]));
            info.is_exhausted = false;
            info.needs_fallback = false;
            info.tree = tree;

        end

    end

end
