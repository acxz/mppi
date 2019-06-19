% Author: Akash Patel (apatel435)
% Date: 6/6/19

function retval = cartpole_term_cost(x)

    Qf = zeros(4,4);
    Qf(1,1) = 0;
    Qf(2,2) = 700;
    Qf(3,3) = 1000;
    Qf(4,4) = 500;

    goal_state = [0; pi; 0; 0];

    retval = 1/2 * sum((x - goal_state) .* (Qf * (x - goal_state)),1);

end
