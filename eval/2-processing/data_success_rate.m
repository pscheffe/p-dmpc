function [success_rate, is_deadlocked] = data_success_rate(experiment_results, optional)

    arguments (Input)
        % ExperimentResult in order (n_vehicles x n_approaches x n_seeds)
        % First approach is assumed to be constant approach
        experiment_results (:, :, :) ExperimentResult
        optional.function_name (1, 1) function_handle = @is_experiment_deadlocked
    end

    arguments (Output)
        % (n_vehicles x n_approaches)
        success_rate (:, :) double
        % (n_vehicles x n_approaches x n_seeds)
        is_deadlocked (:, :, :) logical
    end

    is_deadlocked = zeros(size(experiment_results));

    for i = 1:numel(experiment_results)
        is_deadlocked(i) = optional.function_name(experiment_results(i));
    end

    success_rate = mean(1 - is_deadlocked, 3);
end
