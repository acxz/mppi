% Author: Akash Patel (apatel435)
% Reference: https://arxiv.org/pdf/1707.02342.pdf
% Information Theoretic Model Predictive Control:Theory and Applications to
%   Autonomous Driving
% Algorithm 1: Sampling Based MPC

function [x_hist, u_hist, sample_x_hist, sample_u_hist, rep_traj_cost_hist, ...
  time_hist] = mppi(func_is_task_complete, func_control_update_converged, ...
  func_comp_weights, func_term_cost, func_run_cost,func_gen_next_ctrl, ...
  func_state_est, func_apply_ctrl, func_g, func_F, func_state_transform, ...
  func_control_transform, func_filter_du, num_samples, learning_rate, ...
  init_state, init_ctrl_seq, ctrl_noise_covar, time_horizon, ...
  per_ctrl_based_ctrl_noise, plot_traj, print_verbose, print_short, ...
  save_sampling, sampling_filename)

  % time stuff
  num_timesteps = size(init_ctrl_seq, 2);
  dt = time_horizon / num_timesteps;
  time = 0;
  time_hist = [time];

  % state history
  state_dim = size(init_state, 1);
  x_hist = init_state;
  xo = init_state;

  % control history
  control_dim = size(init_ctrl_seq, 1);
  sample_u_hist = [];
  du = realmax('double') * ones(control_dim, num_timesteps);
  u_hist = [];

  % trajectory cost history
  rep_traj_cost_hist = [];

  % sample state history
  sample_init_state = func_state_transform(init_state);
  sample_state_dim = size(sample_init_state,1);
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
    state_colors = [[0 0.4470 0.7410]; [0.8500 0.3250 0.0980]; [0.4940 0.1840 0.5560]; [0.4660 0.6740 0.1880]];
    ctrl_colors = [[0.9290 0.6940 0.1250]; [0.3010 0.7450 0.9330]; [0.6350 0.0780 0.1840]];
    rep_traj_cost_color = 'k';

    state_plot = figure(1);
    title('State Value(s)')
    xlabel('Time');
    ylabel('Value');
    for sd = 1:state_dim
      state_animated_lines(sd).animatedline = octaveanimatedline(...
          'Color', state_colors(mod(sd - 1,size(state_colors,1)) + 1,:));
          %'DisplayName', ['State ' num2str(sd)]);
    end
    legend

    % Go ahead and plot the first state
    figure(state_plot)
    hold on
    for sd = 1:state_dim
      addpoints(state_animated_lines(sd).animatedline, time_hist(1), x_hist(sd,1));
    end
    legend
    drawnow

    control_plot = figure(2);
    title('Control Value(s)');
    xlabel('Time');
    ylabel('Value');
    for cd = 1:control_dim
      control_animated_lines(cd).animatedline = octaveanimatedline(...
          'Color', ctrl_colors(mod(cd - 1,size(ctrl_colors,2)) + 1,:));
          %'DisplayName', ['Control ' num2str(cd)]);
    end
    legend

    traj_cost_plot = figure(3);
    title('Trajectory Cost');
    xlabel('Time');
    ylabel('Value');
    traj_cost_line = octaveanimatedline('Color', rep_traj_cost_color);
        %'DisplayName', 'Trajectory Cost');
    legend
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

    % normalize weights, in case they are not normalized
    normalized_w = w / sum(w);

    % Compute the representative trajectory cost of what actually happens
    % another way to think about this is weighted average of sample trajectory costs
    rep_traj_cost = sum(normalized_w .* traj_cost);

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

    % Log trajectory cost data
    rep_traj_cost_hist = [rep_traj_cost_hist rep_traj_cost];

    % Move time forward
    time = time + dt;
    time_hist = [time_hist, time];

    % Warmstart next control trajectory using past generated control trajectory
    sample_u_traj(:,1:end-1) = sample_u_traj(:,2:end);
    sample_u_traj(:, end) = func_gen_next_ctrl(sample_u_traj(:, end));

    % Real time plotting
    if(plot_traj)

      figure(state_plot)
      hold on
      for sd = 1:state_dim
        addpoints(state_animated_lines(sd).animatedline, time_hist(total_timestep_num+1), x_hist(sd, total_timestep_num+1));
      end
      legend

      figure(control_plot)
      hold on
      for cd = 1:control_dim
        addpoints(control_animated_lines(cd).animatedline, time_hist(total_timestep_num), u_hist(cd, total_timestep_num))'
      end
      legend

      figure(traj_cost_plot)
      addpoints(traj_cost_line, time_hist(total_timestep_num), rep_traj_cost_hist(total_timestep_num));
      legend
    drawnow
    end

    total_timestep_num = total_timestep_num + 1;
  end
end
