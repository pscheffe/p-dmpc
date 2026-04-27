cache_filename = fullfile(FileNameConstructor.all_results(), "tmp", "experiment_results.mat");

if ~isfile(cache_filename)
    computation_mode = "sequential";
    scenario = "commonroad";
    optimizer = "CppSampled";
    priority_strategies = [
                           PriorityStrategies.constant_priority
                           PriorityStrategies.random_priority
                           PriorityStrategies.coloring_priority
                           PriorityStrategies.FCA_priority
                           PriorityStrategies.explorative_priority
                           PriorityStrategies.optimal_priority
                           ];
    seeds = 1:50;
    Hp = 6;
    experiment_results = eval_experiments( ...
        computation_mode = computation_mode, ...
        scenario_type = scenario, ...
        optimizer = optimizer, ...
        priority_strategies = priority_strategies, ...
        Hp = Hp, ...
        seeds = seeds ...
    );

    save(cache_filename, "experiment_results");
else
    experiment_results = load(cache_filename).experiment_results;
end

%% time per expansion
durations = zeros(0, 35);
n_expansions = zeros(0, 35);

for experiment_result = experiment_results(:)'

    if experiment_result.options.priority == PriorityStrategies.explorative_priority || experiment_result.options.priority == PriorityStrategies.optimal_priority
        % In these prioritizations, n_expanded does not fit optimize0
        % possibly
        continue
    end

    timings = vertcat(experiment_result.timing.optimize0);
    durations = vertcat(durations, timings(2:2:end, :));

    n_raw = vertcat(experiment_result.control_results_info.n_expanded);
    n_expansions = vertcat(n_expansions, reshape(n_raw, size(experiment_result.control_results_info)));
end

t_per_expansion = durations ./ n_expansions;

t_mean_per_expansion = mean(t_per_expansion, "all");
t_median_per_expansion = median(t_per_expansion, "all");
t_max_per_expansion = max(t_per_expansion, [], "all");

%% agents per class

agents_per_class_ratios = zeros(0, 1);
n_agents_per_subgraph = zeros(0, 1);
n_classes_per_subgraph = zeros(0, 1);

for experiment_result = experiment_results(:)'

    for i_step = 1:experiment_result.n_steps
        directed_coupling_sequential = logical(experiment_result.iteration_data(i_step).directed_coupling_sequential);

        belonging_vector = conncomp(digraph(directed_coupling_sequential), Type = "weak");
        n_subgraphs = max(belonging_vector);

        for i_subgraph = 1:n_subgraphs
            is_in_subgraph = belonging_vector == i_subgraph;
            n_agents = nnz(is_in_subgraph);

            if n_agents < 3
                continue
            end

            subgraph_directed_coupling = directed_coupling_sequential(is_in_subgraph, is_in_subgraph);
            n_agent_classes = max(kahn(subgraph_directed_coupling));

            agents_per_class_ratios(end + 1, 1) = n_agents / n_agent_classes;
            n_agents_per_subgraph(end + 1, 1) = n_agents;
            n_classes_per_subgraph(end + 1, 1) = n_agent_classes;
        end

    end

end

agents_per_class_ratio_mean = mean(agents_per_class_ratios, "all");
agents_per_class_ratio_median = median(agents_per_class_ratios, "all");
agents_per_class_ratio_max = max(agents_per_class_ratios, [], "all");
