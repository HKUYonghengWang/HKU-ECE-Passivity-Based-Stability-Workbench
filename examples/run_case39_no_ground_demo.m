% run_case39_no_ground_demo.m
% Reproduces the paper-style IEEE 39-bus no-grounding case-study figure.
%
% Before running:
%   1) Add MATPOWER to the MATLAB path.
%   2) Add the package root, src, and demo folders to the MATLAB path.

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(rootDir, 'src'));
addpath(fullfile(rootDir, 'demo'));
hku_case39_no_ground_demo;
