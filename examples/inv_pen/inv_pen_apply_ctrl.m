% Author: Akash Patel (apatel435)
% Date: 5/29/19

function retval = inv_pen_apply_ctrl(x, u, dt)

    m = 1;
    l = 1;
    g = 9.8;
    b = 0;

    theta = x(1,:);
    theta_dot = x(2,:);

    dx_div_dt = x;

    dx_div_dt(1,:) = theta_dot;
    dx_div_dt(2,:) = -g ./l .* sin(theta) - (b.*theta_dot + u(1,:)) ./ (m.*l.^2);

    retval = x + dx_div_dt * dt;

end
