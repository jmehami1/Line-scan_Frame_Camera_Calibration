disp('Robotics, Vision & Control: (c) Peter Corke 1992-2011 http://www.petercorke.com')

if verLessThan('matlab', '7.0')
    warning('You are running a very old (and unsupported) version of MATLAB.  You will very likely encounter significant problems using the toolboxes but you are on your own with this');
end
% tb = false;
rvcpath = fileparts( mfilename('fullpath') );

robotpath = fullfile(rvcpath, 'robot');
addpath(robotpath);
addpath(fullfile(rvcpath, 'common'));

clear rvcpath robotpath
