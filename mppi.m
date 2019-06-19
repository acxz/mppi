% Author: Akash Patel (apatel435)
% Reference: https://arxiv.org/pdf/1707.02342.pdf
% Information Theoretic Model Predictive Control:Theory and Applications to
%   Autonomous Driving
% Algorithm 1: Sampling Based MPC

function [x_hist, u_hist, sample_x_hist, sample_u_hist, time_hist] = mppi(func_is_task_complete, ...
  func_control_update_converged, func_comp_weights, func_term_cost, ...
  func_run_cost,func_gen_next_ctrl, func_state_est, func_apply_ctrl, func_g, ...
  func_F, func_state_transform, func_control_transform, func_filter_du, ...
  num_samples, learning_rate, init_state, init_ctrl_seq, ctrl_noise_covar, ...
  time_horizon, per_ctrl_based_ctrl_noise, plot_traj, print_verbose, ...
  print_short, save_sampling, sampling_filename)

  % TODO check inputs for correct dimensionality and value ranges
  % TODO SGF
  % TODO provide compute parameterized compute weights function?

  % time stuff
  num_timesteps = size(init_ctrl_seq, 2);
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
  sample_u_hist = [];
  % A big number p much
  %du = realmax * ones(control_dim, num_timesteps);
  du = 100 * ones(control_dim, num_timesteps);
  u_hist = [];

  % sample state history
  sample_init_state = func_state_transform(init_state);
  sample_state_dim = size(sample_init_state,1);
  sample_x_hist = zeros(sample_state_dim, 1);
  sample_x_hist = sample_init_state;
  sample_xo = sample_init_state;

  % state trajectories
  x_traj = zeros(sample_state_dim, num_samples, num_timesteps + 1);

  % control sequence
  sample_u_traj = init_ctrl_seq;

  % sampled control trajectories
  v_traj = zeros(control_dim, num_samples, num_timesteps);

  % plot trajectory in real time
  if(plot_traj)
    figure(1)
    hold on
  end

  total_timestep_num = 1;
  while(func_is_task_complete(xo, time) == false)

    x_traj(:,:,1) = repmat(sample_xo,[1, num_samples]);

    iteration = 1;
    while(func_control_update_converged(du, iteration) == false)

      traj_cost = repmat(func_run_cost(sample_xo), [1, num_samples]);

      % Noise generation
      flat_distribution = randn(control_dim, num_samples * num_timesteps);
      ctrl_noise_flat = ctrl_noise_covar * flat_distribution;
      ctrl_noise = reshape(ctrl_noise_flat, [control_dim, num_samples, num_timesteps]);

      % Compute sampled control trajectories
      ctrl_based_ctrl_noise_samples = round(per_ctrl_based_ctrl_noise * num_samples);
      if (ctrl_based_ctrl_noise_samples == 0)
        v_traj = ctrl_noise;
      elseif (ctrl_based_ctrl_noise_samples == num_samples)
        v_traj = repmat(reshape(sample_u_traj, [control_dim, 1, num_timesteps]), [1, num_samples, 1]) + ctrl_noise;
      else
        v_traj(:,1:ctrl_based_ctrl_noise_samples,:) = repmat(reshape(sample_u_traj, [control_dim, 1, num_timesteps]), [1, ctrl_based_ctrl_noise_samples, 1]) + ctrl_noise(:,1:ctrl_based_ctrl_noise_samples,:);
        v_traj(:,ctrl_based_ctrl_noise_samples+1:end,:) = ctrl_noise(:,ctrl_based_ctrl_noise_samples+1:end,:);
      end

      for timestep_num = 1:num_timesteps

        % Forward propagation
        x_traj(:,:,timestep_num+1) = func_F(x_traj(:,:,timestep_num),func_g(v_traj(:,:,timestep_num)),dt);

        traj_cost = traj_cost + func_run_cost(x_traj(:,:,timestep_num+1)) + learning_rate * (sample_u_traj(:,timestep_num)' * (inv(ctrl_noise_covar) * v_traj(:,:,timestep_num)));

        if(print_verbose)
          fprintf("TN: %d, IN: %d, DU: %d, Simtime: %d\n", timestep_num, iteration, mean(sum(abs(du),1)), time);
        end
      end

      if(save_sampling)
        save("-append", [sampling_filename '_v_traj.dat'],'v_traj');
        save("-append", [sampling_filename '_x_traj.dat'],'x_traj');
        save("-append", [sampling_filename '_traj_cost.dat'], 'traj_cost');
      end

      % TODO investiage the speedup here
      %traj_cost = func_run_cost(x_traj(:,:,timestep_num+1)) + learning_rate * (u_traj(:,timestep_num)' * (inverse(ctrl_noise_covar) * v_traj(:,:,timestep_num)));
      traj_cost = traj_cost + func_term_cost(x_traj(:,:,timestep_num+1));

      % Weight and du calculation
      w = func_comp_weights(traj_cost);
      du = reshape(sum(repmat(w, [control_dim, 1, num_timesteps]) .* ctrl_noise,2), [control_dim, num_timesteps]);

      % Filter the output from forward propagation
      du = func_filter_du(du);

      sample_u_traj = sample_u_traj + du;
      iteration = iteration + 1;
      
      if(print_short && (print_verbose == false))
          fprintf("DU: %d, Simtime: %d\n", mean(sum(abs(du),1)), time);
      end

    end

    % Transform from sample_u to u
    u = func_control_transform(sample_x_hist(:,total_timestep_num), sample_u_traj(:,1), dt);

    % Apply control and log data
    true_x = func_apply_ctrl(x_hist(:,total_timestep_num), u, dt);

    % state estimation after applying control
    xo = func_state_est(true_x);

    % Transform from state used in dynamics vs state used in control sampling
    sample_xo = func_state_transform(xo);

    % Log state data
    x_hist(:,total_timestep_num+1) = xo;
    sample_x_hist(:,total_timestep_num+1) = sample_xo;

    % Log control data
    u_hist = [u_hist u];
    sample_u_hist = [sample_u_hist sample_u_traj(:,1)];

    % Move time forward
    time = time + dt;
    time_hist = [time_hist, time];

    % Warmstart next control trajectory using past generated control trajectory
    sample_u_traj(:,1:end-1) = sample_u_traj(:,2:end);
    sample_u_traj(:, end) = func_gen_next_ctrl(sample_u_traj(:, end));

    % Real time plotting
    if(plot_traj)
      xlim([0,time_hist(end)])
      state_colors = ['b', 'r', 'm', 'c'];
      ctrl_colors = ['k', 'g', 'y', 'w'];
      for sd = (1:state_dim)
        plot(time_hist(total_timestep_num:total_timestep_num+1), ...
             x_hist(sd,total_timestep_num:total_timestep_num+1)', ...
             state_colors(mod(sd - 1,size(state_colors,2)) + 1)) ...

      end

      if (total_timestep_num > 1)
        for cd = 1:control_dim
          plot(time_hist(total_timestep_num-1:total_timestep_num), ...
               u_hist(cd,total_timestep_num-1:total_timestep_num)', ...
               ctrl_colors(mod(cd - 1,size(ctrl_colors,2)) + 1)) ...

        end
      end
      drawnow
    end

    total_timestep_num = total_timestep_num + 1;
  end
end
