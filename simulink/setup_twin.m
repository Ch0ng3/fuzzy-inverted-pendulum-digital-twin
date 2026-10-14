%SETUP_TWIN Prepare the base workspace for the Simulink digital twin (and build it if missing).
%   Defines P (parameters), FIS_* (fuzzy systems), SC_* (normalization/output
%   gains), CTRL_TYPE and Tend. After running it you can open
%   simulink/cartpole_fuzzy_twin.slx and press Run.
%       CTRL_TYPE = 1  -> Mamdani fuzzy balancing
%       CTRL_TYPE = 2  -> Sugeno fuzzy balancing
root = init_project();

P = pendulum_params;
K = design_lqr(P);
C = build_fis(P, K);

% fuzzy inference systems used by the Fuzzy Logic Controller blocks
FIS_theta_mam = C.mamdani_theta;  FIS_cart_mam = C.mamdani_cart;
FIS_theta_sug = C.sugeno_theta;   FIS_cart_sug = C.sugeno_cart;

% normalization / output gains (from the LQR calibration in build_fis)
SC_mam = C.mamdani_scale; SC_mam.th_max = C.th_max; SC_mam.x_max = C.x_max;
SC_sug = C.sugeno_scale;  SC_sug.th_max = C.th_max; SC_sug.x_max = C.x_max;

if ~exist('CTRL_TYPE', 'var'), CTRL_TYPE = 1; end   % 1 = Mamdani, 2 = Sugeno
if ~exist('Tend', 'var'), Tend = 15; end            % simulation time [s]

if ~isfile(fullfile(root, 'simulink', 'cartpole_fuzzy_twin.slx'))
    build_twin_model(true);
end
fprintf('Workspace ready: open simulink/cartpole_fuzzy_twin.slx and press Run (CTRL_TYPE = %d: 1 Mamdani, 2 Sugeno).\n', CTRL_TYPE);
