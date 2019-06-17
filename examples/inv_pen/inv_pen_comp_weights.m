% Author: Akash Patel (apatel435)
% Date: 5/29/19

function retval = inv_pen_comp_weights(traj_cost)
    
    lambda = 0.01;
    [val, ind] = min(traj_cost);
    w = exp(-1/lambda * (traj_cost - val));
    retval = w/sum(w);
    
end