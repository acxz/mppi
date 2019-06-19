% Author: Akash Patel (apatel435)
% Date: 5/29/19

function retval = inv_pen_state_est(true_x)
    xdim = size(true_x,1);
    H = eye(xdim);
    retval = H * true_x;

end
