function tests = mcts_expand_loop_unittest
    % MCTS_EXPAND_LOOP_UNITTEST  Regression tests for the extracted MCTS loop.

    tests = functiontests(localfunctions);
end

function testGatewayFallsBackToMatlabAndIsDeterministic(testcase)
    result_a = runLoop(false);
    result_b = runLoop(false);

    verifyTrue(testcase, isequal(result_a.best_node_id, result_b.best_node_id));
    verifyTrue(testcase, isequal(result_a.best_cost, result_b.best_cost));
    verifyTrue(testcase, isequal(result_a.n_expansions, result_b.n_expansions));
    verifyTrue(testcase, isequal(result_a.trims, result_b.trims));
    verifyTrue(testcase, isequal(result_a.parents, result_b.parents));
end

function testGatewayMatchesMexWhenCompiled(testcase)
    assumeTrue(testcase, exist('mcts_expand_loop_mex', 'file') == 3);

    matlab_result = runLoop(false);
    mex_result = runLoop(true);

    verifyTrue(testcase, isequal(matlab_result.best_node_id, mex_result.best_node_id));
    verifyTrue(testcase, isequal(matlab_result.best_cost, mex_result.best_cost));
    verifyTrue(testcase, isequal(matlab_result.n_expansions, mex_result.n_expansions));
    verifyTrue(testcase, isequal(matlab_result.n_nodes, mex_result.n_nodes));
    verifyTrue(testcase, isequal(matlab_result.trims, mex_result.trims));
    verifyTrue(testcase, isequal(matlab_result.parents, mex_result.parents));
    verifyTrue(testcase, isequal(matlab_result.children, mex_result.children));
    verifyTrue(testcase, isequaln(matlab_result.shapes_tmp, mex_result.shapes_tmp));
end

function result = runLoop(use_cpp_loop)
    [root_pose, reference_trajectory_points, random_numbers, all_successor_trims, maneuvers, trims, parents, children, shapes_tmp, n_nodes, constraint_checker] = createSyntheticLoopInputs();

    result = mcts_expand_loop_gateway( ...
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
