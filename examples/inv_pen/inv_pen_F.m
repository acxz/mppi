% Author: Akash Patel (apatel435)
% Date: 5/29/19

function retval = inv_pen_F(x, u, dt)
    m = 1;
    l = 1;
    g = 9.8;
    b = 0;
    I = m*l^2;

    theta = x(1,:);
    theta_dot = x(2,:);

    dx_div_dt = x;

    dx_div_dt(1,:) = theta_dot;
    dx_div_dt(2,:) = -(b/I) .* theta_dot - (m*g*l/I) .* sin(theta) - u(1,:)./I;

    retval = x + dx_div_dt * dt;

end
