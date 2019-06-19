% Author: Akash Patel (apatel435)
% Date: 6/6/19

function retval = cartpole_state_est(true_x)

    xdim = size(true_x)(1);
    H = eye(xdim);
    retval = H * true_x;

end
