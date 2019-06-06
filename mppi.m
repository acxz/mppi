% Author: Akash Patel (apatel435)
% Reference: https://arxiv.org/pdf/1707.02342.pdf
% Information Theoretic Model Predictive Control:Theory and Applications to
%   Autonomous Driving
% Algorithm 1: Sampling Based MPC

function [x_hist, u_hist, time_hist] = mppi(func_is_task_complete,
  func_control_update_converged, func_comp_weights, func_term_cost,
  func_run_cost,func_gen_next_ctrl, func_state_est, func_apply_ctrl, func_g,
  func_F, num_samples, learning_rate, init_state, init_ctrl_seq,
  ctrl_noise_covar, time_horizon, per_ctrl_based_ctrl_noise)

  % initialize prereqs for algorithm
  % TODO check inputs for correct dimensionality and value ranges
  % TODO SGF
  % TODO alpha
  % TODO comments lol
  % TODO provide compute parameterized compute weights function?
  % TODO running cost does not have control should it? I mean we are
  % generating state trajectories not control trajectories but like the
  % control allows for that state trajectory to happen. Like if we have a
  % cheap state trajectory at the cost of a high control then should we
  % still forget about it in the trajectory cost.

  % Time stuff
  num_timesteps = size(init_ctrl_seq,2);
  dt = time_horizon/num_timesteps;
  time = 0;
  time_hist = [time];

  % state history
  state_dim = size(init_state,1);
  x_hist = zeros(state_dim, 1);
  x_hist = init_state;
  xo = init_state;

  % control history
  control_dim = size(init_ctrl_seq,1);
  u_hist = [];
  % A big number p much
  %du = realmax * ones(control_dim, num_timesteps);
  du = 100 * ones(control_dim, num_timesteps);

  % State trajectories
  x_traj = zeros(state_dim, num_samples, num_timesteps + 1);
  %x_sample_values = zeros(state_dim, num_samples);

  % control sequence
  u_traj = init_ctrl_seq;

  % Plot trajectory in real time
  figure(1)
  hold on

  total_timestep_num = 1;
  while(func_is_task_complete(xo, time) == false)

    x_traj(:,:,1) = repmat(xo,[1, num_samples]);
    %x_sample_values(:,:) = repmat(xo,[1, num_samples]);

    iteration = 1;
    while(func_control_update_converged(du, iteration) == false)

      traj_cost = zeros(1, num_samples);
      traj_cost += repmat(func_run_cost(xo), [1, num_samples]);

      flat_distribution = randn(control_dim, num_samples * num_timesteps);
      ctrl_noise_flat = ctrl_noise_covar * flat_distribution;
      ctrl_noise = reshape(ctrl_noise_flat, [control_dim, num_samples, num_timesteps]);

      %v_traj = repmat(reshape(u_traj, [control_dim, 1, num_timesteps]), [1, num_samples, 1]) + ctrl_noise;
      for timestep_num = 1:num_timesteps
        v_traj = u_traj(:,timestep_num) + ctrl_noise(:,:,timestep_num);

        % TODO (maybe faster) According to algorithm we dont even need to save the x_traj trajectories across timesteps
        x_traj(:,:,timestep_num+1) = func_F(x_traj(:,:,timestep_num),func_g(v_traj),dt);
        %x_traj(:,:,timestep_num+1) = func_F(x_traj(:,:,timestep_num),func_g(v_traj(:,:,timestep_num)),dt);
        %x_sample_values(:,:) = func_F(x_sample_values(:,:),func_g(v),dt);

        traj_cost += func_run_cost(x_traj(:,:,timestep_num+1)) + learning_rate * (u_traj(:,timestep_num)' * (inverse(ctrl_noise_covar) * v_traj));
        %traj_cost += func_run_cost(x_traj(:,:,timestep_num+1)) + learning_rate * (u_traj(:,timestep_num)' * (inverse(ctrl_noise_covar) * v_traj(:,:,timestep_num)));
        %traj_cost += func_run_cost(x_sample_values(:,:)) + learning_rate * (u_traj(:,timestep_num)' * (inverse(ctrl_noise_covar) * v));

        fprintf("TN: %d, IN: %d, DU: %d, Simtime: %d\n", timestep_num, iteration, mean(sum(abs(du),1)), time);
      end
      % TODO investiage the speedup here
      %traj_cost = func_run_cost(x_traj(:,:,timestep_num+1)) + learning_rate * (u_traj(:,timestep_num)' * (inverse(ctrl_noise_covar) * v_traj(:,:,timestep_num)));
      traj_cost += func_term_cost(x_traj(:,:,timestep_num+1));
      %traj_cost += func_term_cost(x_sample_values(:,:));


      w = func_comp_weights(traj_cost);
      du = reshape(sum(repmat(w, [control_dim, 1, num_timesteps]) .* ctrl_noise,2), [control_dim, num_timesteps]);

      u_traj = u_traj + du;
      iteration = iteration + 1;

    end

    u_traj(1:end-1) = u_traj(2:end);
    u_traj(end) = func_gen_next_ctrl(u_traj(end));

    xo = func_apply_ctrl(x_hist(:,total_timestep_num), u_traj(:,1), dt);
    x_hist(:,total_timestep_num+1) = xo;
    u_hist = [u_hist u_traj(:,1)];
    time = time + dt;
    time_hist = [time_hist, time];

    % Real time plotting
    xlim([0,time_hist(end)])
    state_colors = ['b', 'r', 'm', 'c'];
    ctrl_colors = ['k', 'g', 'y', 'w'];
    for sd = 1:state_dim
      plot(time_hist(total_timestep_num:total_timestep_num+1),
          x_hist(sd,total_timestep_num:total_timestep_num+1)',state_colors(mod(sd - 1,size(state_colors,2)) + 1))
    end

    if (total_timestep_num > 1)
      for cd = 1:control_dim
        plot(time_hist(total_timestep_num-1:total_timestep_num),
            u_hist(cd,total_timestep_num-1:total_timestep_num)',ctrl_colors(mod(cd - 1,size(ctrl_colors,2)) + 1))
      end
    end
    drawnow

    total_timestep_num = total_timestep_num + 1;

  end
end
