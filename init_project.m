function root = init_project()
%INIT_PROJECT Add the project folders to the MATLAB path and create the output folders.
%   Called automatically by main_compare and main_twin; can also be run by hand.
root = fileparts(mfilename('fullpath'));
addpath(root, ...
    fullfile(root, 'model'), fullfile(root, 'control'), fullfile(root, 'simulation'), ...
    fullfile(root, 'simulink'), fullfile(root, 'visualization'));
for d = {'figures', 'results', 'videos'}
    p = fullfile(root, 'outputs', d{1});
    if ~isfolder(p), mkdir(p); end
end
% keep Simulink cache/code-generation files out of the source folders
cacheDir = fullfile(root, 'outputs', 'slprj');
if ~isfolder(cacheDir), mkdir(cacheDir); end
Simulink.fileGenControl('set', 'CacheFolder', cacheDir, 'CodeGenFolder', cacheDir);
end
