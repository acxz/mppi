% Author: Akash Patel (apatel435)
% Date: 6/6/19

disp("MPPI implementation on cartpole in Octave/MATLAB")

disp("Starting sampling based MPC")

% TODO: Offload variables into a config file or something
num_samples = 5e3;
time_horizon = 1; % in seconds
num_timesteps = 20;
ctrl_dim = 1;
init_ctrl_seq = randn(ctrl_dim, num_timesteps);
init_state = [0; 0; 0; 0];
ctrl_noise_covar = [5e-1]; % ctrl_dim by ctrl_dim
learning_rate = 0.01;
per_ctrl_based_ctrl_noise = 0.999;
plot_traj = true;
print = true;
save_sampling = false; % Saves XX GB to disk. This will slow the program down.
sampling_filename = "cart_pole";
addpath(genpath('../..'));

[x_hist, u_hist, sample_x_hist, sample_u_hist, time_hist] = mppi(@cartpole_is_task_complete, ...
 @cartpole_control_update_converged, @cartpole_comp_weights, ...
 @cartpole_term_cost, @cartpole_run_cost, @cartpole_gen_next_ctrl, ...
 @cartpole_state_est, @cartpole_apply_ctrl, @cartpole_g, @cartpole_F, ...
 @cartpole_state_transform, @cartpole_control_transform, num_samples, ...
 learning_rate, init_state, init_ctrl_seq, ctrl_noise_covar, time_horizon, ...
 per_ctrl_based_ctrl_noise, plot_traj, print, save_sampling, sampling_filename);

%plot(time_hist, x_hist(1,:));
%plot(time_hist, x_hist(2,:));
%plot(time_hist, x_hist(3,:));
%plot(time_hist, x_hist(4,:));
%plot(time_hist, [u_hist(1,:), 0]);

disp("Finished")
