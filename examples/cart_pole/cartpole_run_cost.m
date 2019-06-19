% Author: Akash Patel (apatel435)
% Date: 6/6/19

function retval = cartpole_run_cost(x)

    Q = eye(4);
    goal_state = [0; pi; 0; 0];

    retval = 1/2 * sum((x - goal_state) .* (Q * (x - goal_state)),1);

end
