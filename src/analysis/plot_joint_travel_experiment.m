function plot_joint_travel_experiment(T)
%PLOT_JOINT_TRAVEL_EXPERIMENT The figure the joint-travel repair is read from.
% Left: the authority the manager grants, with the travel check off (the
% pre-repair behaviour) and on. Right: the joint excursion that results, against
% the mechanism's physical travel. The pair is the whole argument -- if the
% left panel shows no separation, the right panel cannot show one either, and
% the repair is a correctness fix with no performance consequence.
%
% T is optional; with no argument the figure is rebuilt from the saved
% output/results/joint_travel_authority.csv.
if nargin<1||isempty(T)
 f=project_path('results','joint_travel_authority.csv');
 assert(isfile(f),'SensoSpoon:JointTravel',...
  'No saved result at %s. Run START_PROJECT(''jointtravel'') first.',f);
 T=readtable(f,'TextType','string');
end
sw=T(T.scenario=="sine"&T.frequency_Hz==8,:);
off=sw(sw.joint_check==0,:);on=sw(sw.joint_check==1,:);
H=rcflex_parameters;C=rcflex_control_parameters;

f=figure('Color','w','Visible','off','Position',[100 100 1400 520]);
if isprop(f,'Theme');f.Theme='light';end
tiledlayout(1,2);

nexttile;hold on;
plot(off.amplitude_deg,off.gamma_auth_min,'o--','LineWidth',1.6,'MarkerSize',7);
plot(on.amplitude_deg,on.gamma_auth_min,'s-','LineWidth',1.8,'MarkerSize',7);
xline(rad2deg(C.joint_limit),'k:','LineWidth',1.4,...
 'Label','C.joint\_limit','LabelVerticalAlignment','bottom');
grid on;xlabel('tremor amplitude (deg)');ylabel('minimum \gamma_{auth}');
legend({'travel check OFF (pre-repair)','travel check ON (repaired)'},'Location','southwest');
title({'Authority granted by the predictive limiter','separation here is the repair''s only route to an effect'});
hold off;

nexttile;hold on;
plot(off.amplitude_deg,off.joint_peak_deg,'o--','LineWidth',1.6,'MarkerSize',7);
plot(on.amplitude_deg,on.joint_peak_deg,'s-','LineWidth',1.8,'MarkerSize',7);
yline(H.physical_travel_deg,'r:','LineWidth',1.6,...
 'Label','physical travel (end stops)','LabelVerticalAlignment','bottom');
grid on;xlabel('tremor amplitude (deg)');ylabel('peak joint excursion (deg)');
legend({'travel check OFF (pre-repair)','travel check ON (repaired)'},'Location','northwest');
title({'Resulting joint excursion','above the red line the plant is on its end stops'});
hold off;

exportgraphics(f,project_path('figures','joint_travel_authority.png'),'Resolution',160);
close(f);
fprintf('Wrote %s\n',project_path('figures','joint_travel_authority.png'));
end
