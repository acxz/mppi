% Author: Akash Patel (apatel435)

function [sample_u_traj, rep_traj_cost] = mppi(func_control_update_converged, ...
  func_comp_weights, func_term_cost, func_run_cost, func_g, func_F, ...
  func_state_transform, func_filter_du, num_samples, learning_rate, ...
  init_state, init_ctrl_seq, ctrl_noise_covar, time_horizon, ...
  per_ctrl_based_ctrl_noise, print_mppi, save_sampling, sampling_filename)

  % time stuff
  num_timesteps = size(init_ctrl_seq, 2);
  dt = time_horizon / num_timesteps;

  % sample state stuff
  sample_init_state = func_state_transform(init_state);
  sample_state_dim = size(sample_init_state,1);

  % state trajectories
  x_traj = zeros(sample_state_dim, num_samples, num_timesteps + 1);
  x_traj(:,:,1) = repmat(sample_init_state,[1, num_samples]);

  % control stuff
  control_dim = size(init_ctrl_seq, 1);
  du = realmax('double') * ones(control_dim, num_timesteps);

  % control sequence
  sample_u_traj = init_ctrl_seq;

  % sampled control trajectories
  v_traj = zeros(control_dim, num_samples, num_timesteps);

  % Begin mppi
  iteration = 1;
  while(func_control_update_converged(du, iteration) == false)

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

    traj_cost = zeros(1, num_samples);
    
    for timestep_num = 1:num_timesteps

      % Forward propagation
      x_traj(:,:,timestep_num+1) = func_F(x_traj(:,:,timestep_num),func_g(v_traj(:,:,timestep_num)),dt);

      traj_cost = traj_cost + func_run_cost(x_traj(:,:,timestep_num)) + learning_rate * sample_u_traj(:,timestep_num)' * inv(ctrl_noise_covar) * (sample_u_traj(:,timestep_num) - v_traj(:,:,timestep_num));

      if(print_mppi)
        fprintf("TN: %d, IN: %d, DU: %d\n", timestep_num, iteration, mean(sum(abs(du),1)));
      end
    end

    traj_cost = traj_cost + func_term_cost(x_traj(:,:,timestep_num+1));

    if(save_sampling)
      save("-append", [sampling_filename '_v_traj.dat'],'v_traj');
      save("-append", [sampling_filename '_x_traj.dat'],'x_traj');
      save("-append", [sampling_filename '_traj_cost.dat'], 'traj_cost');
    end

    % Weight and du calculation
    w = func_comp_weights(traj_cost);
    du = reshape(sum(repmat(w, [control_dim, 1, num_timesteps]) .* ctrl_noise,2), [control_dim, num_timesteps]);

    % Filter the output from forward propagation
    du = func_filter_du(du);

    sample_u_traj = sample_u_traj + du;
    iteration = iteration + 1;

  end

  % normalize weights, in case they are not normalized
  normalized_w = w / sum(w);

  % Compute the representative trajectory cost of what actually happens
  % another way to think about this is weighted average of sample trajectory costs
  rep_traj_cost = sum(normalized_w .* traj_cost);

end
