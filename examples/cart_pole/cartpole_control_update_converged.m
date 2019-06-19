% Author: Akash Patel (apatel435)
% Date: 6/6/19

function retval = cartpole_control_update_converged(du, iteration)

    tol = 0.01;
    max_iteration = 5;
    retval = false;
    if iteration > max_iteration
      retval = true;
    end

end
