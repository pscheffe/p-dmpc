function are_satisfied = are_constraints_satisfied_interx_fast( ...
        shape, ...
        shape_for_boundary_check, ...
        i_step, ...
        vehicle_obstacles, ...
        lanelet_boundary, ...
        hdv_obstacles ...
    )
    % ARE_CONSTRAINTS_SATISFIED_INTERX_FAST  Collision check for single-vehicle MCTS loop.

    are_satisfied = true;

    if InterX(shape, vehicle_obstacles{i_step})
        are_satisfied = false;
        return
    end

    is_hdv_obstacle = ~all(all(isnan(hdv_obstacles{i_step})));

    if is_hdv_obstacle && InterX(shape, hdv_obstacles{i_step})
        are_satisfied = false;
        return
    end

    if InterX(shape_for_boundary_check, lanelet_boundary)
        are_satisfied = false;
        return
    end

end
