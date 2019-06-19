% Author: Akash Patel (apatel435)
% Date: 6/6/19

function retval = cartpole_apply_ctrl(x, u, dt)

    mc = 1;
    mp = 0.01;
    l = 0.25;
    g = 9.81;

    xpos = x(1,:);
    theta = x(2,:);
    xpos_dot = x(3,:);
    theta_dot = x(4,:);

    f = u(1,:);

    dx_div_dt = x;

    dx_div_dt(1,:) = xpos_dot;
    dx_div_dt(2,:) = theta_dot;
    dx_div_dt(3,:) = (1./(mc + mp .* sin(theta).^2)) .* (f + mp .* sin(theta) .* (l .* theta_dot.^2 + g .* cos(theta)));
    dx_div_dt(4,:) = (1./(l .* (mc + mp .* sin(theta).^2))) .* (-f .* cos(theta) - mp .* l .* theta_dot.^2 .* cos(theta) .* sin(theta) - (mc + mp) .* g .* sin(theta));

    retval = x + dx_div_dt * dt;

end
