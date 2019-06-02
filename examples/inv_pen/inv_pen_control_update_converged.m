% Author: Akash Patel (apatel435)
% Date: 5/29/19

function retval = inv_pen_control_update_converged(du, iteration)
    
    tol = 0.01;
    max_iteration = 5;
    retval = false;
    if iteration > max_iteration
      retval = true;
    end
    
endfunction