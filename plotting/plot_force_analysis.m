%--------------------------------------------------------------------------
% purpose: plot contact and load torques and forces
%  input: F_contacts = contact forces along trajectory
%             F_load = load forces along trajectory
%                  t = time vector associated with trajectory
%            fig_num = figure number
% output:
%--------------------------------------------------------------------------
function [] = plot_force_analysis(F_contacts, F_load, t, fig_num)
num_agents = size(F_contacts, 2) / 3;
%--------------------------------------------------------------------------
% contact plots
%--------------------------------------------------------------------------
lstr = cell(num_agents, 1);
for ii=1:num_agents
   lstr{ii} = sprintf('contact %d', ii); 
end

figure(fig_num);
fig_num = fig_num + 1;
subplot(1, 2, 1);
plot(t, F_contacts(:, 1:3:end));
xlabel('time (s)');
ylabel('torque (Nm)');
title('contact torque vs time');
legend(lstr);

Fx = F_contacts(:, 2:3:end);
Fy = F_contacts(:, 3:3:end);
F = sqrt(Fx.^2 + Fy.^2);

subplot(1, 2, 2);
plot(t, F);
xlabel('time (s)');
ylabel('F (N)');
title('contact force vs time');
legend(lstr);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% load plots
%--------------------------------------------------------------------------
figure(fig_num);
subplot(1, 2, 1);
plot(t, F_load(:,1));
xlabel('time (s)');
ylabel('torque (Nm)');
title('load torque vs time');

Fx = F_load(:,2);
Fy = F_load(:,3);
F = sqrt(Fx.^2 + Fy.^2);

subplot(1, 2, 2);
plot(t, F);
xlabel('time (s)');
ylabel('F (N)');
title('load force vs time');
%--------------------------------------------------------------------------
end
%--------------------------------------------------------------------------