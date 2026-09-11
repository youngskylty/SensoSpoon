function p=project_path(area,varargin)
%PROJECT_PATH Absolute project locations; independent of MATLAB current folder.
root=fileparts(fileparts(mfilename('fullpath')));
switch char(area)
 case 'root';base=root;
 case 'assets';base=fullfile(root,'assets','geometry');
 case 'models';base=fullfile(root,'models');
 case 'figures';base=fullfile(root,'output','figures');
 case 'results';base=fullfile(root,'output','results');
 case 'cache';base=fullfile(root,'output','cache');
 otherwise;error('SensoSpoon:Path','Unknown project area: %s',area);
end
p=fullfile(base,varargin{:});
end
