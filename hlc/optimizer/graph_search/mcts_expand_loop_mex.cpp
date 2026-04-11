#include "mex.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    (void)nlhs;
    (void)plhs;
    (void)nrhs;
    (void)prhs;

    mexErrMsgIdAndTxt(
        "MonteCarloTreeSearch:MexNotImplemented",
        "mcts_expand_loop_mex is not implemented yet. "
        "The MATLAB gateway will automatically fall back to mcts_expand_loop_matlab."
    );
}
