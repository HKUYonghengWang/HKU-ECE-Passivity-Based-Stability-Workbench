% launch_hku_passivity_workbench.m
% Adds the workbench folders to the MATLAB path and launches the GUI.

clear app;
thisFile = mfilename('fullpath');
rootDir = fileparts(thisFile);
addpath(fullfile(rootDir, 'src'));
addpath(fullfile(rootDir, 'assets'));
app = HKU_ECE_PassivityWorkbenchApp;
