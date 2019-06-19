% Author: Akash Patel (apatel435)
% Date: 6/6/19

function is_task_complete = cartpole_is_task_complete(x, t)

    is_task_complete = false;
    if t > 5;
      is_task_complete = true;
    end

end
