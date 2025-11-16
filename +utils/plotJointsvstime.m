function plotJointsVsTime(t_vec, q_traj, qd_traj)
% utils.plotJointsVsTime Plots joint angles & velocities vs time

  if ~exist('report','dir')
    mkdir('report');
  end

  n_dof_per_arm = 7;

  % Positions
  fig3 = figure('Name','Joint Angles vs. Time','Units','normalized','Position',[0.1 0.1 0.8 0.8]);
  for i = 1:n_dof_per_arm
      subplot(n_dof_per_arm,2,2*i-1);
      plot(t_vec, q_traj(:,i), 'b-'); grid on;
      title(sprintf('Left Joint %d Angle', i));
      ylabel('Angle (rad)'); xlabel('Time (s)');
  end
  for i = 1:n_dof_per_arm
      subplot(n_dof_per_arm,2,2*i);
      plot(t_vec, q_traj(:,i+n_dof_per_arm), 'r-'); grid on;
      title(sprintf('Right Joint %d Angle', i));
      ylabel('Angle (rad)'); xlabel('Time (s)');
  end
  sgtitle('Figure 3: Joint Angle (Position) Trajectories vs. Time','FontWeight','bold');
  out3 = fullfile('report','Figure3_Joint_Angles.png');
  saveas(fig3, out3);
  fprintf('Saved Figure 3 to %s\n', out3);

  % Velocities
  fig4 = figure('Name','Joint Velocities vs. Time','Units','normalized','Position',[0.15 0.15 0.8 0.8]);
  for i = 1:n_dof_per_arm
      subplot(n_dof_per_arm,2,2*i-1);
      plot(t_vec, qd_traj(:,i), 'b-'); grid on;
      title(sprintf('Left Joint %d Velocity', i));
      ylabel('Vel (rad/s)'); xlabel('Time (s)');
  end
  for i = 1:n_dof_per_arm
      subplot(n_dof_per_arm,2,2*i);
      plot(t_vec, qd_traj(:,i+n_dof_per_arm), 'r-'); grid on;
      title(sprintf('Right Joint %d Velocity', i));
      ylabel('Vel (rad/s)'); xlabel('Time (s)');
  end
  sgtitle('Figure 4: Joint Velocity Trajectories vs. Time','FontWeight','bold');
  out4 = fullfile('report','Figure4_Joint_Velocities.png');
  saveas(fig4, out4);
  fprintf('Saved Figure 4 to %s\n', out4);
end
