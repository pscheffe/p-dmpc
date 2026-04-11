function stats = benchmark_mcts_expand_loop(iterations, use_cpp_loop)
    % BENCHMARK_MCTS_EXPAND_LOOP  Measure runtime for the extracted MCTS core.

    if nargin < 1
        iterations = 25;
    end

    if nargin < 2
        use_cpp_loop = false;
    end

    [root_pose, reference_trajectory_points, random_numbers, all_successor_trims, maneuvers, trims, parents, children, shapes_tmp, n_nodes, constraint_checker] = createSyntheticLoopInputs();
    mex_available = exist('mcts_expand_loop_mex', 'file') == 3;

    elapsed = zeros(iterations, 1);

    for k = 1:iterations
        tic;
        mcts_expand_loop_gateway( ...
            use_cpp_loop, ...
            10, ...
            2, ...
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
        );
        elapsed(k) = toc;
    end

    stats = struct();
    stats.iterations = iterations;
    stats.mean_seconds = mean(elapsed);
    stats.median_seconds = median(elapsed);
    stats.min_seconds = min(elapsed);
    stats.max_seconds = max(elapsed);
    stats.use_cpp_loop = use_cpp_loop;
    stats.mex_available = mex_available;

    backend_name = 'MATLAB';

    if use_cpp_loop && mex_available
        backend_name = 'MEX';
    end

    fprintf('MCTS loop benchmark (%s backend) over %d iterations: mean %.6f s, median %.6f s\n', ...
        backend_name, stats.iterations, stats.mean_seconds, stats.median_seconds);
end

function [root_pose, reference_trajectory_points, random_numbers, all_successor_trims, maneuvers, trims, parents, children, shapes_tmp, n_nodes, constraint_checker] = createSyntheticLoopInputs()
    root_pose = [0; 0; 0];
    reference_trajectory_points = [1, 2; 0, 0];
    random_numbers = ones(1, 20);

    all_successor_trims = cell(2, 2);
    all_successor_trims{1, 1} = uint8([1 2]);
    all_successor_trims{1, 2} = uint8([1 2]);
    all_successor_trims{2, 1} = uint8([1 2]);
    all_successor_trims{2, 2} = uint8([1 2]);

    maneuvers = cell(2, 2);

    for start_trim = 1:2

        for goal_trim = 1:2
            maneuver.dpose = [1; 0; 0];
            maneuver.area_without_offset = [0 1 1 0; 0 0 1 1];
            maneuver.area = [0 1 1 0; 0 0 1 1];
            maneuver.area_large_offset = [0 1 1 0; 0 0 1 1];
            maneuvers{start_trim, goal_trim} = maneuver;
        end

    end

    trims = zeros(1, 11, 'uint8');
    trims(1) = 1;
    parents = zeros(1, 11, 'uint32');
    children = zeros(2, 11, 'uint32');
    children(:, 1) = uint32([1; 1]);
    shapes_tmp = cell(1, 11);
    n_nodes = 1;

    constraint_checker = @(~, ~, ~) true;
end
