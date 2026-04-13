classdef mcts_cpp_systemtests < matlab.unittest.TestCase

    methods (Test)

        function test_mcts_cpp_matches_matlab_trajectories(testCase)
            ensure_mcts_mex_is_available(testCase);

            options = Config.load_from_file('tests/systemtests/Config_systemtests_prioritized.json');
            options.scenario_type = ScenarioType.commonroad;
            options.is_prioritized = true;
            options.computation_mode = ComputationMode.sequential;
            options.amount = 3;
            options.path_ids = [18, 19, 20];
            options.validate();

            matlab_result = run_experiment_with_optimizer(options, OptimizerType.MatlabSampled);
            cpp_result = run_experiment_with_optimizer(options, OptimizerType.CppSampled);

            verify_experiment_results_equal(testCase, matlab_result, cpp_result);
        end

        function test_mcts_cpp_runs_without_fallback_warning(testCase)
            ensure_mcts_mex_is_available(testCase);

            options = Config.load_from_file('tests/systemtests/Config_systemtests_prioritized.json');
            options.scenario_type = ScenarioType.commonroad;
            options.is_prioritized = true;
            options.computation_mode = ComputationMode.sequential;
            options.amount = 3;
            options.path_ids = [18, 19, 20];
            options.validate();

            warning_state = warning('query', 'MonteCarloTreeSearch:MexFallback');
            cleanup_warning = onCleanup(@() warning(warning_state));
            warning('error', 'MonteCarloTreeSearch:MexFallback');

            cpp_result = run_experiment_with_optimizer(options, OptimizerType.CppSampled);
            testCase.verifyNotEmpty(cpp_result);
        end

    end

end

function ensure_mcts_mex_is_available(testCase)

    if exist('mcts_expand_loop_mex', 'file') ~= 3
        build_mcts_expand_loop_mex();
    end

    testCase.verifyEqual(exist('mcts_expand_loop_mex', 'file'), 3);
end

function experiment_result = run_experiment_with_optimizer(options, optimizer_type)
    options.optimizer_type = optimizer_type;
    experiment_result = main(options);
end

function verify_experiment_results_equal(testCase, matlab_result, cpp_result)
    testCase.verifyEqual(matlab_result.n_steps, cpp_result.n_steps);
    testCase.verifyEqual(matlab_result.options.amount, cpp_result.options.amount);
    testCase.verifyEqual(matlab_result.options.Hp, cpp_result.options.Hp);

    for k = 1:matlab_result.n_steps
        testCase.verifyEqual(matlab_result.get_y_predicted(k), cpp_result.get_y_predicted(k));

        for i_vehicle = 1:matlab_result.options.amount
            matlab_info = matlab_result.control_results_info(i_vehicle, k);
            cpp_info = cpp_result.control_results_info(i_vehicle, k);

            testCase.verifyEqual(matlab_info.y_predicted, cpp_info.y_predicted);
            testCase.verifyEqual(matlab_info.predicted_trims, cpp_info.predicted_trims);
            testCase.verifyEqual(matlab_info.n_expanded, cpp_info.n_expanded, ...
                sprintf('n_expanded mismatch at step %d, vehicle %d', k, i_vehicle));
            testCase.verifyEqual(matlab_info.is_exhausted, cpp_info.is_exhausted);
            testCase.verifyEqual(matlab_info.needs_fallback, cpp_info.needs_fallback);
        end

    end

    for k = 1:matlab_result.n_steps
        testCase.verifyTrue(matlab_result.iteration_data(k).isequal(cpp_result.iteration_data(k)));
    end

end
