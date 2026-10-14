function out = main_twin(ctrlType, makeVideo)
%MAIN_TWIN Simulate the Simulink digital twin and show the 2D animated dashboard.
%   main_twin(1)          % Mamdani: 3D Mechanics Explorer + 2D animation
%   main_twin(2)          % Sugeno
%   main_twin(1, true)    % also record outputs/videos/twin_mamdani.mp4 (+ preview GIF)
if nargin < 1, ctrlType = 1; end
if nargin < 2, makeVideo = false; end
root = init_project();

assignin('base', 'CTRL_TYPE', ctrlType);
evalin('base', 'setup_twin');                       % parameters, FIS, gains, model
names = {'Mamdani', 'Sugeno'};
mdl = 'cartpole_fuzzy_twin'; load_system(fullfile(root, 'simulink', [mdl '.slx']));
out = sim(mdl);                                      % uses the base workspace

zt = out.z_twin; t = zt.Time; z = squeeze(zt.Data);
if size(z, 1) ~= numel(t), z = z'; end
F = interp1(out.F.Time, squeeze(out.F.Data), t, 'previous', 'extrap');
mode = interp1(out.mode.Time, squeeze(out.mode.Data), t, 'previous', 'extrap');
P = evalin('base', 'P');
ttl = sprintf('Digital twin: energy swing-up + %s fuzzy balancing', names{ctrlType});

video = '';
if makeVideo
    video = fullfile(root, 'outputs', 'videos', sprintf('twin_%s.mp4', lower(names{ctrlType})));
end
animate_pendulum(t, z, F, mode, P, ttl, video, 1);
if makeVideo
    make_gif(video, strrep(video, '.mp4', '.gif'));
end
end
