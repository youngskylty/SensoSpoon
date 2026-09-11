function S=START_PROJECT(action)
%START_PROJECT Single entry: setup | preview | test | simulate | control | compare | soup | benchmark | analysis | all.
% Open this file and Run for a preview. Use control to run the sensor-feedback controller.
if nargin<1;action='preview';end
root=fileparts(mfilename('fullpath'));
addpath(root,genpath(fullfile(root,'src')),fullfile(root,'tests'),fullfile(root,'models'));
for area={'assets','figures','results','cache','models'}
 p=project_path(area{1});if ~isfolder(p);mkdir(p);end
end
Simulink.fileGenControl('set','CacheFolder',project_path('cache'),...
 'CodeGenFolder',fullfile(project_path('cache'),'codegen'),'createDir',true);
S=[];action=validatestring(action,{'setup','preview','test','simulate','control','compare','soup','benchmark','analysis','all'});
if strcmp(action,'setup');return;end
H=rcflex_parameters();asset=project_path('assets','assembly.mat');
rebuild=~isfile(asset);
if ~rebuild;d=load(asset,'H');rebuild=~isequaln(d.H,H);end
if rebuild;generate_rcflex_assets(H);end
switch action
 case 'preview'
  preview_rcflex_design('assembled');preview_rcflex_design('cutaway');preview_spoon_closeup();
 case 'test'
  run_validation();
 case 'simulate'
  S=run_rcflex_analytic('voltage',6);plot_rcflex_plant(S);
  animate_rcflex_analytic(S,project_path('figures','voltage_test.mp4'),'assembled');
 case 'control'
  S=run_rcflex_control();verify_rcflex_control_numerics();plot_rcflex_control(S);
  animate_rcflex_analytic(S,project_path('figures','control_demo.mp4'),'assembled');
 case 'compare'
  animate_rcflex_comparison();
 case 'soup'
  S=run_rcflex_soup_demo(5);
 case 'benchmark'
  S=benchmark_rcflex_control();
 case 'analysis'
  evaluate_rcflex_robustness();
 case 'all'
  generate_rcflex_assets(H);build_rcflex_analytic();run_validation();
  preview_rcflex_design('assembled');preview_rcflex_design('cutaway');
  preview_spoon_closeup();preview_spoon_dimensions();
  S=run_rcflex_analytic('voltage',6);plot_rcflex_plant(S);
  animate_rcflex_analytic(S,project_path('figures','voltage_test.mp4'),'assembled');
end
end
