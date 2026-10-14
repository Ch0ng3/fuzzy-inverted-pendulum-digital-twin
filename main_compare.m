%MAIN_COMPARE Swing-up + fuzzy balancing: Mamdani vs Sugeno in four scenarios (MATLAB only).
%   Runs the closed-loop simulations in plain MATLAB (no Simulink), prints a
%   comparison table and saves:
%     outputs/results/comparison.csv, comparison.mat and the four .fis files
%     outputs/figures/scenario_1..4.png, control_surfaces.png, membership_functions.png
clear; close all;
root = init_project();
figDir = fullfile(root, 'outputs', 'figures');
resDir = fullfile(root, 'outputs', 'results');

P = pendulum_params;
K = design_lqr(P);
C = build_fis(P, K);
writeFIS(C.mamdani_theta, fullfile(resDir,'FLC_theta_mamdani')); writeFIS(C.mamdani_cart, fullfile(resDir,'FLC_cart_mamdani'));
writeFIS(C.sugeno_theta,  fullfile(resDir,'FLC_theta_sugeno'));  writeFIS(C.sugeno_cart,  fullfile(resDir,'FLC_cart_sugeno'));

% ---------------- scenarios ----------------
Pheavy = P; Pheavy.m = 1.5*P.m; Pheavy.I = Pheavy.m*(2*P.l)^2/12; Pheavy.b = 2*P.b;
S = struct('name', {}, 'Tend', {}, 'opts', {});
S(1) = struct('name', "nominal",           'Tend', 15, 'opts', struct());
S(2) = struct('name', "push at t=10 s",    'Tend', 20, 'opts', struct('kick', [10, 2.5]));
S(3) = struct('name', "model mismatch",    'Tend', 15, 'opts', struct('Pplant', Pheavy));
S(4) = struct('name', "sensor noise",      'Tend', 15, 'opts', struct('noise', [1e-3, 2e-3]));

types = ["mamdani", "sugeno"];
T = table();
Rall = cell(numel(S), 2);
for i = 1:numel(S)
    for j = 1:2
        R = simulate_pendulum(P, C, types(j), S(i).Tend, S(i).opts);
        Rall{i,j} = R;
        T = [T; table(S(i).name, types(j), R.t_catch, R.balanced, R.n_switch, ...
            rad2deg(R.bal_max_theta), rad2deg(R.bal_rms_theta), 100*R.bal_rms_x, R.t_settle, R.bal_effort, R.max_F, 100*R.max_x, ...
            'VariableNames', {'scenario','controller','t_catch_s','balanced','switches', ...
            'max_theta_deg','rms_theta_deg','rms_x_cm','settle_s','effort_N2s','max_F_N','max_x_cm'})]; %#ok<AGROW>
    end
end
disp(T);
writetable(T, fullfile(resDir, 'comparison.csv'));
save(fullfile(resDir, 'comparison.mat'), 'Rall', 'S', 'T', 'P', 'K');

% ---------------- figures ----------------
cols = lines(2);
for i = 1:numel(S)
    f = figure('Name', S(i).name, 'Position', [100 100 1100 750], 'Color', 'w');
    tl = tiledlayout(f, 4, 1, 'TileSpacing', 'compact');
    title(tl, "Swing-up + fuzzy balancing -- " + S(i).name);
    for j = 1:2
        R = Rall{i,j};
        nexttile(1); hold on; plot(R.t, rad2deg(R.thw), 'Color', cols(j,:), 'LineWidth', 1.2);
        nexttile(2); hold on; plot(R.t, R.z(:,1), 'Color', cols(j,:), 'LineWidth', 1.2);
        nexttile(3); hold on; plot(R.t, R.F, 'Color', cols(j,:), 'LineWidth', 1.0);
        nexttile(4); hold on; stairs(R.t, R.mode + 0.03*(j-1), 'Color', cols(j,:), 'LineWidth', 1.2);
    end
    nexttile(1); ylabel('\theta [deg]'); grid on; legend('Mamdani', 'Sugeno', 'Location', 'northeast');
    nexttile(2); ylabel('x [m]'); grid on;
    nexttile(3); ylabel('F [N]'); grid on;
    nexttile(4); ylabel('mode'); yticks([0 1]); yticklabels({'swing-up','balance'}); grid on; xlabel('t [s]');
    exportgraphics(f, fullfile(figDir, sprintf('scenario_%d.png', i)), 'Resolution', 150);
end

% control surfaces of the angle controller
f = figure('Position', [100 100 1100 450], 'Color', 'w');
subplot(1,2,1); gensurf(C.mamdani_theta); title('Mamdani: u(e_\theta, \Delta e_\theta)');
subplot(1,2,2); gensurf(C.sugeno_theta);  title('Sugeno: u(e_\theta, \Delta e_\theta)');
exportgraphics(f, fullfile(figDir, 'control_surfaces.png'), 'Resolution', 150);

% membership functions
f = figure('Position', [100 100 1100 380], 'Color', 'w');
subplot(1,2,1); plotmf(C.mamdani_theta, 'input', 1); title('Input MFs (e_\theta, normalized)');
subplot(1,2,2); plotmf(C.mamdani_theta, 'output', 1); title('Mamdani output MFs (u, normalized)');
exportgraphics(f, fullfile(figDir, 'membership_functions.png'), 'Resolution', 150);
fprintf('Figures saved in %s\n', figDir);
