clear all;

addpath(genpath('spatial_v2_extended'))
addpath(genpath('robots'))
addpath(genpath('examples'))
addpath(genpath('utils'))
addpath(genpath('biped_3link'))

if ismac
    addpath(genpath('Casadi_3.6.3_osx64_matlab2018b'))
elseif ispc
    addpath(genpath('casadi-3.6.4-windows64-matlab2018b'))
elseif isunix 
    addpath(genpath('casadi-3.6.4-linux64-matlab2018b'))
else
    disp('Platform not supported')
end

addpath(genpath('project'))