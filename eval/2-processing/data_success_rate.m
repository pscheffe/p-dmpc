function [success_rate, is_deadlocked] = data_success_rate(experiment_results, optional)

    arguments (Input)
        % ExperimentResult in order (n_vehicles x n_approaches x n_seeds)
        % First approach is assumed to be constant approach
        experiment_results (:, :, :) ExperimentResult
        optional.function_name (1, 1) function_handle = @is_experiment_deadlocked
        optional.cache_folder (1, 1) string = FileNameConstructor.all_results()
    end

    arguments (Output)
        % (n_vehicles x n_approaches)
        success_rate (:, :) double
        % (n_vehicles x n_approaches x n_seeds)
        is_deadlocked (:, :, :) logical
    end

    is_deadlocked = zeros(size(experiment_results));

    filepath = fullfile( ...
        optional.cache_folder, ...
        sprintf( ...
            '6-prioritization_is_deadlocked_%s_%s.mat', ...
            experiment_results(1).options.scenario_type, ...
            experiment_results(1).options.optimizer_type ...
        ) ...
    );

    try
        cache = load(filepath);
        if size(cache.is_deadlocked) == size(is_deadlocked)
            is_deadlocked = cache.is_deadlocked;
        else
            error("Size mismatch of cached deadlock computation")
        end
    catch
        disp("Analyzing results for deadlocks ...");
        for i = 1:numel(experiment_results)
            is_deadlocked(i) = optional.function_name(experiment_results(i));
        end
        save(filepath,"is_deadlocked")
    end

    success_rate = mean(1 - is_deadlocked, 3);
end
