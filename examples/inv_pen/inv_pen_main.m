% Author: Akash Patel (apatel435)
% Date: 5/29/19

disp("MPPI implementation on inv_pen in MATLAB")

disp("starting sampling based MPC")

% TODO: Offload variables into a config file or something
num_samples = 5e3;
time_horizon = 1; % in seconds
num_timesteps = 10;
ctrl_dim = 1;
init_ctrl_seq = randn(ctrl_dim, num_timesteps);
init_state = [0; 0];
ctrl_noise_covar = [5e-1]; % ctrl_dim by ctrl_dim
learning_rate = 0.01;
per_ctrl_based_ctrl_noise = 0.999;
plot_traj = true;
print = false;
addpath(genpath('../..'));

[x_hist, u_hist, time_hist] = mppi(@inv_pen_is_task_complete,
 @inv_pen_control_update_converged, @inv_pen_comp_weights, @inv_pen_term_cost,
@inv_pen_run_cost,@inv_pen_gen_next_ctrl, @inv_pen_state_est, @inv_pen_apply_ctrl,
 @inv_pen_g, @inv_pen_F, num_samples,learning_rate, init_state, init_ctrl_seq,
 ctrl_noise_covar, time_horizon, per_ctrl_based_ctrl_noise, plot_traj, print);
 
%plot(time_hist, x_hist(1,:));
%plot(time_hist, x_hist(2,:));
%plot(time_hist, [u_hist, 0]);

disp("Finished")