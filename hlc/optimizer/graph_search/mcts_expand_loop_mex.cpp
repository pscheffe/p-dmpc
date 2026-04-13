#include "mex.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr mwIndex kRootNodeIndex = 0;

struct Segment {
    double x1;
    double y1;
    double x2;
    double y2;
};

enum class ConstraintMode {
    Callback,
    InterX
};

struct InterXPayload {
    const mxArray* vehicle_obstacles;
    const mxArray* hdv_obstacles;
    const mxArray* lanelet_boundary;
};

mwSize getScalarSizeT(const mxArray* array, const char* name) {
    if (!mxIsDouble(array) || mxIsComplex(array) || mxGetNumberOfElements(array) != 1) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "%s must be a real scalar double.", name);
    }

    const double value = mxGetScalar(array);

    if (value < 0.0) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "%s must be non-negative.", name);
    }

    return static_cast<mwSize>(value);
}

double getScalarDouble(const mxArray* array, const char* name) {
    if (!mxIsDouble(array) || mxIsComplex(array) || mxGetNumberOfElements(array) != 1) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "%s must be a real scalar double.", name);
    }

    return mxGetScalar(array);
}

std::string getString(const mxArray* array, const char* name) {
    if (!mxIsChar(array)) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "%s must be a character array.", name);
    }

    char* raw = mxArrayToString(array);

    if (raw == nullptr) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "Failed to parse %s.", name);
    }

    std::string result(raw);
    mxFree(raw);
    return result;
}

unsigned char* getUint8Data(mxArray* array, const char* name) {
    if (!mxIsUint8(array) || mxIsComplex(array)) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "%s must be a real uint8 array.", name);
    }

    return static_cast<unsigned char*>(mxGetData(array));
}

std::uint32_t* getUint32Data(mxArray* array, const char* name) {
    if (!mxIsUint32(array) || mxIsComplex(array)) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "%s must be a real uint32 array.", name);
    }

    return static_cast<std::uint32_t*>(mxGetData(array));
}

double* getDoubleData(mxArray* array, const char* name) {
    if (!mxIsDouble(array) || mxIsComplex(array)) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "%s must be a real double array.", name);
    }

    return mxGetPr(array);
}

mxArray* duplicateCellArray(const mxArray* array, const char* name) {
    if (!mxIsCell(array)) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "%s must be a cell array.", name);
    }

    return mxDuplicateArray(array);
}

const mxArray* getCellElement(const mxArray* cell_array, mwIndex row, mwIndex column, const char* name) {
    if (!mxIsCell(cell_array)) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "%s must be a cell array.", name);
    }

    const mwSize rows = mxGetM(cell_array);
    const mwSize cols = mxGetN(cell_array);

    if (row >= rows || column >= cols) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "Cell index out of bounds for %s.", name);
    }

    return mxGetCell(cell_array, row + column * rows);
}

std::vector<unsigned char> getUint8VectorFromCell(const mxArray* cell_array, mwIndex row, mwIndex column, const char* name) {
    const mxArray* vector_array = getCellElement(cell_array, row, column, name);

    if (vector_array == nullptr) {
        return {};
    }

    const mwSize n_elements = mxGetNumberOfElements(vector_array);

    if (mxIsUint8(vector_array) && !mxIsComplex(vector_array)) {
        const unsigned char* data = static_cast<const unsigned char*>(mxGetData(vector_array));
        return std::vector<unsigned char>(data, data + n_elements);
    }

    if (mxIsLogical(vector_array)) {
        const mxLogical* data = mxGetLogicals(vector_array);
        std::vector<unsigned char> result;
        result.reserve(n_elements);

        for (mwSize index = 0; index < n_elements; ++index) {
            result.push_back(data[index] ? 1U : 0U);
        }

        return result;
    }

    if (!mxIsDouble(vector_array) && !mxIsSingle(vector_array) && !mxIsInt8(vector_array) && !mxIsUint8(vector_array) &&
        !mxIsInt16(vector_array) && !mxIsUint16(vector_array) && !mxIsInt32(vector_array) && !mxIsUint32(vector_array) &&
        !mxIsInt64(vector_array) && !mxIsUint64(vector_array)) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "%s cell contents must be numeric or logical.", name);
    }

    std::vector<unsigned char> result;
    result.reserve(n_elements);

    if (mxIsDouble(vector_array)) {
        const double* data = mxGetPr(vector_array);

        for (mwSize index = 0; index < n_elements; ++index) {
            result.push_back(static_cast<unsigned char>(data[index]));
        }
    } else if (mxIsSingle(vector_array)) {
        const float* data = static_cast<const float*>(mxGetData(vector_array));

        for (mwSize index = 0; index < n_elements; ++index) {
            result.push_back(static_cast<unsigned char>(data[index]));
        }
    } else if (mxIsInt8(vector_array)) {
        const std::int8_t* data = static_cast<const std::int8_t*>(mxGetData(vector_array));

        for (mwSize index = 0; index < n_elements; ++index) {
            result.push_back(static_cast<unsigned char>(data[index]));
        }
    } else if (mxIsUint16(vector_array)) {
        const std::uint16_t* data = static_cast<const std::uint16_t*>(mxGetData(vector_array));

        for (mwSize index = 0; index < n_elements; ++index) {
            result.push_back(static_cast<unsigned char>(data[index]));
        }
    } else if (mxIsInt16(vector_array)) {
        const std::int16_t* data = static_cast<const std::int16_t*>(mxGetData(vector_array));

        for (mwSize index = 0; index < n_elements; ++index) {
            result.push_back(static_cast<unsigned char>(data[index]));
        }
    } else if (mxIsUint32(vector_array)) {
        const std::uint32_t* data = static_cast<const std::uint32_t*>(mxGetData(vector_array));

        for (mwSize index = 0; index < n_elements; ++index) {
            result.push_back(static_cast<unsigned char>(data[index]));
        }
    } else if (mxIsInt32(vector_array)) {
        const std::int32_t* data = static_cast<const std::int32_t*>(mxGetData(vector_array));

        for (mwSize index = 0; index < n_elements; ++index) {
            result.push_back(static_cast<unsigned char>(data[index]));
        }
    } else if (mxIsUint64(vector_array)) {
        const std::uint64_t* data = static_cast<const std::uint64_t*>(mxGetData(vector_array));

        for (mwSize index = 0; index < n_elements; ++index) {
            result.push_back(static_cast<unsigned char>(data[index]));
        }
    } else if (mxIsInt64(vector_array)) {
        const std::int64_t* data = static_cast<const std::int64_t*>(mxGetData(vector_array));

        for (mwSize index = 0; index < n_elements; ++index) {
            result.push_back(static_cast<unsigned char>(data[index]));
        }
    }

    return result;
}

const mxArray* getManeuverStruct(const mxArray* maneuvers, mwIndex start_trim, mwIndex goal_trim) {
    return getCellElement(maneuvers, start_trim, goal_trim, "maneuvers");
}

const mxArray* getStructField(const mxArray* structure, const char* field_name) {
    if (!mxIsStruct(structure)) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "Maneuver cell contents must be structs.");
    }

    const mxArray* field = mxGetField(structure, 0, field_name);

    if (field == nullptr) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "Maneuver struct is missing field '%s'.", field_name);
    }

    return field;
}

mxArray* translatePolygon(const mxArray* polygon, double c, double s, double tx, double ty) {
    if (!mxIsDouble(polygon) || mxIsComplex(polygon)) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "Polygon inputs must be real double matrices.");
    }

    const mwSize rows = mxGetM(polygon);
    const mwSize cols = mxGetN(polygon);

    if (rows != 2) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "Polygon matrices must have size 2xN.");
    }

    const double* input = mxGetPr(polygon);
    mxArray* translated = mxCreateDoubleMatrix(rows, cols, mxREAL);
    double* output = mxGetPr(translated);

    for (mwIndex column = 0; column < cols; ++column) {
        const mwIndex base = column * rows;
        const double x = input[base + 0];
        const double y = input[base + 1];
        output[base + 0] = (c * x) - (s * y) + tx;
        output[base + 1] = (s * x) + (c * y) + ty;
    }

    return translated;
}

mxArray* makeSingleShapeCell(const mxArray* polygon, double c, double s, double tx, double ty) {
    mxArray* shape_cell = mxCreateCellMatrix(1, 1);
    mxSetCell(shape_cell, 0, translatePolygon(polygon, c, s, tx, ty));
    return shape_cell;
}

bool callConstraintChecker(const mxArray* constraint_checker, const mxArray* shape, const mxArray* boundary_shape, mwIndex i_step) {
    mxArray* rhs[4];
    rhs[0] = const_cast<mxArray*>(constraint_checker);
    rhs[1] = const_cast<mxArray*>(shape);
    rhs[2] = const_cast<mxArray*>(boundary_shape);
    rhs[3] = mxCreateDoubleScalar(static_cast<double>(i_step));

    mxArray* result = nullptr;
    const int status = mexCallMATLAB(1, &result, 4, rhs, "feval");
    mxDestroyArray(rhs[3]);

    if (status != 0 || result == nullptr) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:CallbackFailed", "Constraint checker callback failed.");
    }

    const bool is_valid = mxGetScalar(result) != 0.0;
    mxDestroyArray(result);
    return is_valid;
}

bool isAllNan(const mxArray* matrix) {
    if (!mxIsDouble(matrix) || mxIsComplex(matrix)) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "Obstacle matrices must be real double.");
    }

    const mwSize count = mxGetNumberOfElements(matrix);
    const double* values = mxGetPr(matrix);

    for (mwSize i = 0; i < count; ++i) {
        if (!mxIsNaN(values[i])) {
            return false;
        }
    }

    return true;
}

std::vector<Segment> buildSegments(const mxArray* matrix) {
    if (mxIsEmpty(matrix)) {
        return {};
    }

    if (!mxIsDouble(matrix) || mxIsComplex(matrix) || mxGetM(matrix) != 2) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "Polyline matrices must be 2xN real double.");
    }

    const mwSize n_cols = mxGetN(matrix);
    const double* values = mxGetPr(matrix);
    std::vector<Segment> segments;

    bool has_previous = false;
    double px = 0.0;
    double py = 0.0;

    for (mwIndex col = 0; col < n_cols; ++col) {
        const double x = values[col * 2];
        const double y = values[col * 2 + 1];
        const bool is_separator = mxIsNaN(x) || mxIsNaN(y);

        if (is_separator) {
            has_previous = false;
            continue;
        }

        if (has_previous) {
            segments.push_back(Segment{px, py, x, y});
        }

        px = x;
        py = y;
        has_previous = true;
    }

    return segments;
}

double orientation(const Segment& reference, double x, double y) {
    return (reference.y2 - reference.y1) * (x - reference.x2) - (reference.x2 - reference.x1) * (y - reference.y2);
}

bool onSegment(double ax, double ay, double bx, double by, double px, double py) {
    constexpr double eps = 1e-12;
    const bool within_x = (px <= std::max(ax, bx) + eps) && (px >= std::min(ax, bx) - eps);
    const bool within_y = (py <= std::max(ay, by) + eps) && (py >= std::min(ay, by) - eps);
    return within_x && within_y;
}

int signWithTolerance(double value) {
    constexpr double eps = 1e-12;

    if (value > eps) {
        return 1;
    }

    if (value < -eps) {
        return -1;
    }

    return 0;
}

bool segmentsIntersect(const Segment& a, const Segment& b) {
    const double o1_val = (a.y2 - a.y1) * (b.x1 - a.x2) - (a.x2 - a.x1) * (b.y1 - a.y2);
    const double o2_val = (a.y2 - a.y1) * (b.x2 - a.x2) - (a.x2 - a.x1) * (b.y2 - a.y2);
    const double o3_val = (b.y2 - b.y1) * (a.x1 - b.x2) - (b.x2 - b.x1) * (a.y1 - b.y2);
    const double o4_val = (b.y2 - b.y1) * (a.x2 - b.x2) - (b.x2 - b.x1) * (a.y2 - b.y2);

    const int o1 = signWithTolerance(o1_val);
    const int o2 = signWithTolerance(o2_val);
    const int o3 = signWithTolerance(o3_val);
    const int o4 = signWithTolerance(o4_val);

    if ((o1 != o2) && (o3 != o4)) {
        return true;
    }

    if ((o1 == 0) && onSegment(a.x1, a.y1, a.x2, a.y2, b.x1, b.y1)) {
        return true;
    }

    if ((o2 == 0) && onSegment(a.x1, a.y1, a.x2, a.y2, b.x2, b.y2)) {
        return true;
    }

    if ((o3 == 0) && onSegment(b.x1, b.y1, b.x2, b.y2, a.x1, a.y1)) {
        return true;
    }

    if ((o4 == 0) && onSegment(b.x1, b.y1, b.x2, b.y2, a.x2, a.y2)) {
        return true;
    }

    return false;
}

bool hasAnyIntersection(const mxArray* polyline_a, const mxArray* polyline_b) {
    const std::vector<Segment> segments_a = buildSegments(polyline_a);
    const std::vector<Segment> segments_b = buildSegments(polyline_b);

    for (const Segment& sa : segments_a) {
        for (const Segment& sb : segments_b) {
            if (segmentsIntersect(sa, sb)) {
                return true;
            }
        }
    }

    return false;
}

bool callInterXConstraint(const InterXPayload& payload, const mxArray* shape, const mxArray* boundary_shape, mwIndex i_step) {
    if (!mxIsCell(payload.vehicle_obstacles) || !mxIsCell(payload.hdv_obstacles)) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "InterX payload obstacle containers must be cell arrays.");
    }

    const mwIndex step_idx = i_step - 1;
    const mxArray* vehicle_obstacles = getCellElement(payload.vehicle_obstacles, 0, step_idx, "vehicle_obstacles");

    if (vehicle_obstacles != nullptr && hasAnyIntersection(shape, vehicle_obstacles)) {
        return false;
    }

    const mxArray* hdv_obstacles = getCellElement(payload.hdv_obstacles, 0, step_idx, "hdv_obstacles");

    if ((hdv_obstacles != nullptr) && !isAllNan(hdv_obstacles) && hasAnyIntersection(shape, hdv_obstacles)) {
        return false;
    }

    if (hasAnyIntersection(boundary_shape, payload.lanelet_boundary)) {
        return false;
    }

    return true;
}

ConstraintMode parseConstraintMode(const mxArray* payload) {
    if (!mxIsStruct(payload)) {
        return ConstraintMode::Callback;
    }

    const mxArray* mode_field = mxGetField(payload, 0, "mode");

    if (mode_field == nullptr) {
        return ConstraintMode::Callback;
    }

    const std::string mode = getString(mode_field, "constraint_payload.mode");

    if (mode == "interx") {
        return ConstraintMode::InterX;
    }

    return ConstraintMode::Callback;
}

InterXPayload parseInterXPayload(const mxArray* payload) {
    if (!mxIsStruct(payload)) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "InterX mode requires a struct payload.");
    }

    const mxArray* vehicle_obstacles = mxGetField(payload, 0, "vehicle_obstacles");
    const mxArray* hdv_obstacles = mxGetField(payload, 0, "hdv_obstacles");
    const mxArray* lanelet_boundary = mxGetField(payload, 0, "lanelet_boundary");

    if (vehicle_obstacles == nullptr || hdv_obstacles == nullptr || lanelet_boundary == nullptr) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "InterX payload missing required fields.");
    }

    return InterXPayload{vehicle_obstacles, hdv_obstacles, lanelet_boundary};
}

double squaredDistanceToReference(const double node_pose[3], const double* reference_points, mwIndex i_step) {
    const mwSize rows = 2;
    const mwIndex column = i_step - 1;
    const mwIndex base = column * rows;
    const double dx = node_pose[0] - reference_points[base + 0];
    const double dy = node_pose[1] - reference_points[base + 1];
    return (dx * dx) + (dy * dy);
}

void writeScalarField(mxArray* structure, mwIndex index, const char* field_name, double value) {
    mxSetField(structure, index, field_name, mxCreateDoubleScalar(value));
}

} // namespace

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 14) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "Expected 14 inputs.");
    }

    if (nlhs > 1) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidOutput", "Expected at most 1 output.");
    }

    const mwSize n_expansions_max = getScalarSizeT(prhs[0], "n_expansions_max");
    const mwSize Hp = getScalarSizeT(prhs[1], "Hp");

    if (!mxIsDouble(prhs[2]) || mxIsComplex(prhs[2]) || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "root_pose must be a 3x1 double vector.");
    }

    if (!mxIsDouble(prhs[3]) || mxIsComplex(prhs[3]) || mxGetM(prhs[3]) != 2 || mxGetN(prhs[3]) != Hp) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "reference_trajectory_points must be a 2xHp double matrix.");
    }

    if (!mxIsDouble(prhs[4]) || mxIsComplex(prhs[4]) || mxGetM(prhs[4]) != 1) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "random_numbers must be a 1xN double vector.");
    }

    if (!mxIsCell(prhs[5])) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "all_successor_trims must be a cell array.");
    }

    if (!mxIsCell(prhs[6])) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "maneuvers must be a cell array.");
    }

    mxArray* trims_out = mxDuplicateArray(prhs[7]);
    mxArray* parents_out = mxDuplicateArray(prhs[8]);
    mxArray* children_out = mxDuplicateArray(prhs[9]);
    mxArray* shapes_tmp_out = duplicateCellArray(prhs[10], "shapes_tmp");

    const mwSize max_nodes = mxGetNumberOfElements(prhs[7]);
    const mwSize n_successor_trims_max = mxGetM(prhs[9]);
    const mwSize initial_n_nodes = getScalarSizeT(prhs[11], "n_nodes");

    if (initial_n_nodes == 0 || initial_n_nodes > max_nodes) {
        mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "n_nodes must be within 1..numel(trims).");
    }

    const double* root_pose_in = mxGetPr(prhs[2]);
    const double* reference_points = mxGetPr(prhs[3]);
    const double* random_numbers = mxGetPr(prhs[4]);
    const mxArray* all_successor_trims = prhs[5];
    const mxArray* maneuvers = prhs[6];
    const mxArray* constraint_checker = prhs[12];
    const mxArray* constraint_payload = prhs[13];

    const ConstraintMode constraint_mode = parseConstraintMode(constraint_payload);
    std::unique_ptr<InterXPayload> interx_payload;

    if (constraint_mode == ConstraintMode::InterX) {
        interx_payload = std::make_unique<InterXPayload>(parseInterXPayload(constraint_payload));
    }

    unsigned char* trims = getUint8Data(trims_out, "trims");
    std::uint32_t* parents = getUint32Data(parents_out, "parents");
    std::uint32_t* children = getUint32Data(children_out, "children");

    mwSize n_nodes = initial_n_nodes;
    mwSize n_expansions = 0;
    mwSize n_traversals = 0;
    bool is_finished = false;
    bool is_valid = false;
    double best_cost = std::numeric_limits<double>::infinity();
    double best_node_id = -1.0;

    double root_pose[3] = {root_pose_in[0], root_pose_in[1], root_pose_in[2]};
    const mwSize random_numbers_count = mxGetNumberOfElements(prhs[4]);

    while ((n_expansions < n_expansions_max) && !is_finished) {
        mwSize node_id = 1;
        double solution_cost = 0.0;
        double node_pose[3] = {root_pose[0], root_pose[1], root_pose[2]};
        mwIndex child_position = 0;
        mwIndex node_parent = 0;

        for (mwSize i_step = 1; i_step <= Hp; ++i_step) {
            is_valid = false;
            ++n_traversals;

            if (n_traversals > random_numbers_count) {
                mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "random_numbers is too short for the requested expansions.");
            }

            const mwIndex node_column = node_id - 1;
            std::vector<mwIndex> trim_positions;
            trim_positions.reserve(n_successor_trims_max);

            for (mwIndex row = 0; row < n_successor_trims_max; ++row) {
                const std::uint32_t child_value = children[row + node_column * n_successor_trims_max];

                if (child_value != 0) {
                    trim_positions.push_back(row + 1);
                }
            }

            const mwSize n_trims = trim_positions.size();

            if (n_trims != 0) {
                const double random_value = random_numbers[n_traversals - 1];
                mwSize child_index = static_cast<mwSize>(std::ceil(random_value * static_cast<double>(n_trims)));

                if (child_index < 1) {
                    child_index = 1;
                }

                if (child_index > n_trims) {
                    child_index = n_trims;
                }

                child_position = trim_positions[child_index - 1];
            } else {
                if (node_id != 1) {
                    const mwIndex parent_id = static_cast<mwIndex>(parents[node_id - 1]);
                    const mwIndex parent_column = parent_id - 1;
                    const mwIndex edge_row = child_position - 1;
                    children[edge_row + parent_column * n_successor_trims_max] = 0;
                    break;
                }

                is_finished = true;
                break;
            }

            const mwIndex parent_trim = static_cast<mwIndex>(trims[node_id - 1]);
            const std::vector<unsigned char> successor_trims = getUint8VectorFromCell(all_successor_trims, parent_trim - 1, i_step - 1, "all_successor_trims");

            if (child_position == 0 || child_position > successor_trims.size()) {
                mexErrMsgIdAndTxt("MonteCarloTreeSearch:InvalidInput", "child_position exceeds successor trim count.");
            }

            const mwIndex goal_trim = static_cast<mwIndex>(successor_trims[child_position - 1]);
            const mxArray* maneuver_struct = getManeuverStruct(maneuvers, parent_trim - 1, goal_trim - 1);
            const mxArray* dpose_array = getStructField(maneuver_struct, "dpose");
            const mxArray* area_without_offset_array = getStructField(maneuver_struct, "area_without_offset");
            const mxArray* area_array = getStructField(maneuver_struct, "area");
            const mxArray* area_large_offset_array = getStructField(maneuver_struct, "area_large_offset");

            const double* dpose = mxGetPr(dpose_array);
            const double c = std::cos(node_pose[2]);
            const double s = std::sin(node_pose[2]);

            const double start_x = node_pose[0];
            const double start_y = node_pose[1];
            const double start_yaw = node_pose[2];

            node_pose[0] = start_x + (c * dpose[0]) - (s * dpose[1]);
            node_pose[1] = start_y + (s * dpose[0]) + (c * dpose[1]);
            node_pose[2] = start_yaw + dpose[2];

            solution_cost += squaredDistanceToReference(node_pose, reference_points, i_step);

            const mwIndex child_row = child_position - 1;
            const std::uint32_t child_value = children[child_row + node_column * n_successor_trims_max];

            if (child_value != 1) {
                node_id = static_cast<mwSize>(child_value);
                continue;
            }

            ++n_expansions;
            node_parent = node_id;

            mxArray* shape_matrix = translatePolygon(area_array, c, s, start_x, start_y);
            mxArray* shape_without_offset_matrix = translatePolygon(area_without_offset_array, c, s, start_x, start_y);
            mxArray* shape_for_boundary_check_matrix = nullptr;
            mxArray* child_successor_cell = nullptr;

            if (i_step != Hp) {
                shape_for_boundary_check_matrix = shape_without_offset_matrix;
                const std::vector<unsigned char> child_successor_trims = getUint8VectorFromCell(all_successor_trims, goal_trim - 1, i_step, "all_successor_trims");

                if (!child_successor_trims.empty()) {
                    child_successor_cell = mxCreateNumericMatrix(1, child_successor_trims.size(), mxUINT8_CLASS, mxREAL);
                    unsigned char* child_successor_data = static_cast<unsigned char*>(mxGetData(child_successor_cell));

                    for (mwIndex index = 0; index < child_successor_trims.size(); ++index) {
                        child_successor_data[index] = child_successor_trims[index];
                    }
                }
            } else {
                shape_for_boundary_check_matrix = translatePolygon(area_large_offset_array, c, s, start_x, start_y);
            }

            if (constraint_mode == ConstraintMode::InterX) {
                is_valid = callInterXConstraint(*interx_payload, shape_matrix, shape_for_boundary_check_matrix, i_step);
            } else {
                is_valid = callConstraintChecker(constraint_checker, shape_matrix, shape_for_boundary_check_matrix, i_step);
            }

            if (!is_valid) {
                children[child_row + (node_parent - 1) * n_successor_trims_max] = 0;
                mxDestroyArray(shape_matrix);
                mxDestroyArray(shape_without_offset_matrix);

                if (i_step != Hp) {
                    if (child_successor_cell != nullptr) {
                        mxDestroyArray(child_successor_cell);
                    }
                } else {
                    mxDestroyArray(shape_for_boundary_check_matrix);
                }

                break;
            }

            if (n_nodes >= max_nodes) {
                mexErrMsgIdAndTxt("MonteCarloTreeSearch:OutOfSpace", "Preallocated node arrays are too small.");
            }

            ++n_nodes;
            parents[n_nodes - 1] = static_cast<std::uint32_t>(node_parent);
            trims[n_nodes - 1] = static_cast<unsigned char>(goal_trim);

            if (child_successor_cell != nullptr) {
                const mwSize child_successor_count = mxGetNumberOfElements(child_successor_cell);
                unsigned char* child_successor_data = static_cast<unsigned char*>(mxGetData(child_successor_cell));

                for (mwSize index = 0; index < child_successor_count; ++index) {
                    children[index + (n_nodes - 1) * n_successor_trims_max] = child_successor_data[index] != 0 ? 1U : 0U;
                }
            }

            children[child_row + (node_parent - 1) * n_successor_trims_max] = static_cast<std::uint32_t>(n_nodes);
            mxSetCell(shapes_tmp_out, n_nodes - 1, mxDuplicateArray(shape_matrix));
            mxDestroyArray(shape_matrix);
            node_id = n_nodes;

            mxDestroyArray(shape_without_offset_matrix);

            if (i_step != Hp) {
                if (child_successor_cell != nullptr) {
                    mxDestroyArray(child_successor_cell);
                }
            } else {
                mxDestroyArray(shape_for_boundary_check_matrix);
            }
        }

        if (is_valid) {
            if (solution_cost < best_cost) {
                best_cost = solution_cost;
                best_node_id = static_cast<double>(node_id);
            }

            const mwIndex node_parent_column = node_parent - 1;
            const mwIndex edge_row = child_position - 1;
            children[edge_row + node_parent_column * n_successor_trims_max] = 0;
        }
    }

    const char* field_names[] = {
        "trims",
        "parents",
        "children",
        "shapes_tmp",
        "n_nodes",
        "n_expansions",
        "best_node_id",
        "best_cost"
    };

    mxArray* result = mxCreateStructMatrix(1, 1, 8, field_names);
    mxSetField(result, 0, "trims", trims_out);
    mxSetField(result, 0, "parents", parents_out);
    mxSetField(result, 0, "children", children_out);
    mxSetField(result, 0, "shapes_tmp", shapes_tmp_out);
    writeScalarField(result, 0, "n_nodes", static_cast<double>(n_nodes));
    writeScalarField(result, 0, "n_expansions", static_cast<double>(n_expansions));
    writeScalarField(result, 0, "best_node_id", best_node_id);
    writeScalarField(result, 0, "best_cost", best_cost);

    plhs[0] = result;
}
