function compile_mcts_expand_loop_mex()
    % BUILD_MCTS_EXPAND_LOOP_MEX  Compile the MCTS loop MEX.
    %
    % This compiles the MEX used by CppSampled optimizer.

    source_file = fullfile(fileparts(mfilename('fullpath')), 'mcts_expand_loop_mex.cpp');

    if ~isfile(source_file)
        error('build_mcts_expand_loop_mex:MissingSource', ...
            'Missing source file: %s', source_file);
    end

    current_folder = pwd;
    cleanup = onCleanup(@() cd(current_folder));
    cd(fileparts(source_file));

    mex('-R2018a', 'CXXFLAGS=$CXXFLAGS -std=c++17', 'mcts_expand_loop_mex.cpp');

    clear cleanup;
end
