function mdl = build_twin_model(openEditor)
%BUILD_TWIN_MODEL Generate the Simulink model 'cartpole_fuzzy_twin.slx'.
%
%   Plant  : Simscape Multibody cart-pendulum (the "digital twin", 3D view in
%            Mechanics Explorer), plus the equation model in parallel for validation.
%   Control: discrete controller (Ts) = energy swing-up + switching logic
%            + two fuzzy balancing controllers (Mamdani and Sugeno, Fuzzy Logic
%            Controller blocks) selected by CTRL_TYPE (1 = Mamdani, 2 = Sugeno).
%
%   Run setup_twin first (it defines P, FIS_*, SC_* and CTRL_TYPE in the base workspace).

if nargin < 1, openEditor = true; end
mdl = 'cartpole_fuzzy_twin';
if bdIsLoaded(mdl), close_system(mdl, 0); end
here = fileparts(mfilename('fullpath'));
f = fullfile(here, [mdl '.slx']);
if isfile(f), delete(f); end
load_system('sm_lib'); load_system('nesl_utility'); load_system('fuzblock'); load_system('simulink');
new_system(mdl);
FLC = ['fuzblock/Fuzzy Logic ' newline 'Controller'];

set_param(mdl, 'Solver', 'ode23t', 'MaxStep', '1e-3', 'RelTol', '1e-5', 'AbsTol', '1e-6', 'StopTime', 'Tend');

%% ======================= PLANT: Simscape Multibody =======================
pl = [mdl '/Digital Twin (Simscape Multibody)'];
add_block('simulink/Ports & Subsystems/Subsystem', pl, 'Position', [520 80 760 260]);
delete_line(pl, 'In1/1', 'Out1/1'); delete_block([pl '/In1']); delete_block([pl '/Out1']);

blk = @(lib, name, pos) add_block(lib, [pl '/' name], 'Position', pos);
blk('sm_lib/Frames and Transforms/World Frame', 'World', [30 200 70 240]);
blk('sm_lib/Utilities/Mechanism Configuration', 'Mechanism Configuration', [30 290 90 330]);
blk('nesl_utility/Solver Configuration', 'Solver Configuration', [30 370 90 410]);
set_param([pl '/Mechanism Configuration'], 'UniformGravity', 'Constant', 'GravityVector', '[0 -P.g 0]');

% rail (visual only)
blk('sm_lib/Frames and Transforms/Rigid Transform', 'Rail offset', [130 100 170 140]);
set_param([pl '/Rail offset'], 'TranslationMethod', 'StandardAxis', 'TranslationStandardAxis', '-Y', ...
    'TranslationStandardOffset', '0.06', 'TranslationStandardOffsetUnits', 'm');
blk('sm_lib/Body Elements/Brick Solid', 'Rail', [230 90 290 150]);
set_param([pl '/Rail'], 'BrickDimensions', '[2*P.xmax+0.4 0.03 0.08]', 'BrickDimensionsUnits', 'm', ...
    'GraphicDiffuseColor', '[0.5 0.5 0.55]');

% prismatic joint along world X: rotate the base frame so that its Z axis = world X
blk('sm_lib/Frames and Transforms/Rigid Transform', 'Z to X', [130 200 170 240]);
set_param([pl '/Z to X'], 'RotationMethod', 'StandardAxis', 'RotationStandardAxis', '+Y', ...
    'RotationAngle', '90', 'RotationAngleUnits', 'deg');
blk('sm_lib/Joints/Prismatic Joint', 'Cart joint (x)', [230 190 290 270]);
set_param([pl '/Cart joint (x)'], 'TorqueActuationMode', 'InputTorque', ...
    'SensePosition', 'on', 'SenseVelocity', 'on', ...
    'DampingCoefficient', 'P.b', 'DampingCoefficientUnits', 'N*s/m', ...
    'PositionTargetSpecify', 'on', 'PositionTargetPriority', 'High', ...
    'PositionTargetValue', 'P.x0(1)', 'PositionTargetValueUnits', 'm');

% cart body (frame axes: X = -world Z, Y = world Y, Z = world X)
blk('sm_lib/Body Elements/Brick Solid', 'Cart', [400 120 460 180]);
set_param([pl '/Cart'], 'BrickDimensions', '[0.16 0.08 0.25]', 'BrickDimensionsUnits', 'm', ...
    'InertiaType', 'Custom', 'Mass', 'P.M', 'MassUnits', 'kg', ...
    'MomentsOfInertia', '[1e-3 1e-3 1e-3]', 'GraphicDiffuseColor', '[0.15 0.35 0.75]');

% back to world orientation, then the revolute joint about world Z
blk('sm_lib/Frames and Transforms/Rigid Transform', 'X to Z', [400 220 440 260]);
set_param([pl '/X to Z'], 'RotationMethod', 'StandardAxis', 'RotationStandardAxis', '+Y', ...
    'RotationAngle', '-90', 'RotationAngleUnits', 'deg');
blk('sm_lib/Joints/Revolute Joint', 'Pendulum joint (q)', [500 210 560 290]);
set_param([pl '/Pendulum joint (q)'], 'SensePosition', 'on', 'SenseVelocity', 'on', ...
    'DampingCoefficient', 'P.c', 'DampingCoefficientUnits', 'N*m*s/rad', ...
    'PositionTargetSpecify', 'on', 'PositionTargetPriority', 'High', ...
    'PositionTargetValue', '-P.x0(3)', 'PositionTargetValueUnits', 'rad');   % theta = -q

% pendulum rod: centre of mass at distance l along +Y of the joint follower frame
blk('sm_lib/Frames and Transforms/Rigid Transform', 'To rod centre', [620 220 660 260]);
set_param([pl '/To rod centre'], 'TranslationMethod', 'StandardAxis', 'TranslationStandardAxis', '+Y', ...
    'TranslationStandardOffset', 'P.l', 'TranslationStandardOffsetUnits', 'm');
blk('sm_lib/Body Elements/Brick Solid', 'Rod', [720 210 780 270]);
set_param([pl '/Rod'], 'BrickDimensions', '[0.025 2*P.l 0.025]', 'BrickDimensionsUnits', 'm', ...
    'InertiaType', 'Custom', 'Mass', 'P.m', 'MassUnits', 'kg', ...
    'MomentsOfInertia', '[P.I 1e-6 P.I]', 'GraphicDiffuseColor', '[0.85 0.25 0.15]');

% force input and sensors
add_block('simulink/Sources/In1', [pl '/F'], 'Position', [130 300 160 314]);
blk('nesl_utility/Simulink-PS Converter', 'F to PS', [190 295 220 320]);
set_param([pl '/F to PS'], 'Unit', 'N');
names = {'x', 'xd', 'q', 'qd'}; units = {'m', 'm/s', 'rad', 'rad/s'};
for i = 1:4
    blk('nesl_utility/PS-Simulink Converter', ['PS ' names{i}], [620 300+50*(i-1) 650 325+50*(i-1)]);
    set_param([pl '/PS ' names{i}], 'Unit', units{i});
end
add_block('simulink/Signal Routing/Mux', [pl '/Mux'], 'Inputs', '4', 'Position', [760 300 765 480]);
add_block('simulink/Math Operations/Gain', [pl '/theta = -q'], 'Gain', '[1;1;-1;-1]', ...
    'Multiplication', 'Element-wise(K.*u)', 'Position', [800 375 840 405]);
add_block('simulink/Sinks/Out1', [pl '/z'], 'Position', [880 383 910 397]);

ph = @(b) get_param([pl '/' b], 'PortHandles');
conn = @(a, b) add_line(pl, a, b, 'autorouting', 'on');
W = ph('World'); MC = ph('Mechanism Configuration'); SC = ph('Solver Configuration');
RO = ph('Rail offset'); RA = ph('Rail'); ZX = ph('Z to X'); PJ = ph('Cart joint (x)');
CA = ph('Cart'); XZ = ph('X to Z'); RJ = ph('Pendulum joint (q)'); TR = ph('To rod centre'); RD = ph('Rod');
conn(W.RConn(1), MC.RConn(1)); conn(W.RConn(1), SC.RConn(1));
conn(W.RConn(1), RO.LConn(1)); conn(RO.RConn(1), RA.RConn(1));
conn(W.RConn(1), ZX.LConn(1)); conn(ZX.RConn(1), PJ.LConn(1));
conn(PJ.RConn(1), CA.RConn(1)); conn(PJ.RConn(1), XZ.LConn(1));
conn(XZ.RConn(1), RJ.LConn(1)); conn(RJ.RConn(1), TR.LConn(1)); conn(TR.RConn(1), RD.RConn(1));
FP = ph('F to PS'); FI = ph('F');
conn(FI.Outport(1), FP.Inport(1)); conn(FP.RConn(1), PJ.LConn(2));        % force actuation
sens = {PJ.RConn(2), PJ.RConn(3), RJ.RConn(2), RJ.RConn(3)};              % p, v outputs
MX = ph('Mux');
for i = 1:4
    C = ph(['PS ' names{i}]);
    conn(sens{i}, C.LConn(1)); conn(C.Outport(1), MX.Inport(i));
end
G = ph('theta = -q'); Z = ph('z');
conn(MX.Outport(1), G.Inport(1)); conn(G.Outport(1), Z.Inport(1));

%% ======================= REFERENCE: equation model =======================
eq = [mdl '/Equation model (reference)'];
add_block('simulink/Ports & Subsystems/Subsystem', eq, 'Position', [520 330 760 410]);
delete_line(eq, 'In1/1', 'Out1/1'); delete_block([eq '/In1']); delete_block([eq '/Out1']);
add_block('simulink/Sources/In1', [eq '/F'], 'Position', [30 53 60 67]);
add_block('simulink/User-Defined Functions/MATLAB Function', [eq '/dynamics'], 'Position', [120 40 260 100]);
set_mfun([eq '/dynamics'], sprintf(['function dz = dynamics(z, F, P)\n' ...
    '%% nonlinear cart-pendulum equations (same as pendulum_dynamics.m)\n' ...
    'xd = z(2); th = z(3); thd = z(4); s = sin(th); c = cos(th);\n' ...
    'A = [P.M + P.m, P.m*P.l*c; P.m*P.l*c, P.I + P.m*P.l^2];\n' ...
    'rhs = [F + P.m*P.l*thd^2*s - P.b*xd; P.m*P.g*P.l*s - P.c*thd];\n' ...
    'acc = A \\ rhs;\n' ...
    'dz = [xd; acc(1); thd; acc(2)];\n']), {'P'});
add_block('simulink/Continuous/Integrator', [eq '/Integrator'], 'InitialCondition', 'P.x0', 'Position', [320 50 350 80]);
add_block('simulink/Sinks/Out1', [eq '/z'], 'Position', [400 58 430 72]);
add_line(eq, 'Integrator/1', 'dynamics/1', 'autorouting', 'on');
add_line(eq, 'F/1', 'dynamics/2', 'autorouting', 'on');
add_line(eq, 'dynamics/1', 'Integrator/1', 'autorouting', 'on');
add_line(eq, 'Integrator/1', 'z/1', 'autorouting', 'on');

%% ======================= CONTROLLER (discrete, Ts) =======================
ct = [mdl '/Controller (Ts = 10 ms)'];
add_block('simulink/Ports & Subsystems/Subsystem', ct, 'Position', [140 80 360 260]);
delete_line(ct, 'In1/1', 'Out1/1'); delete_block([ct '/In1']); delete_block([ct '/Out1']);
add_block('simulink/Sources/In1', [ct '/z'], 'Position', [20 200 50 214]);
add_block('simulink/Discrete/Zero-Order Hold', [ct '/ZOH'], 'SampleTime', 'P.Ts', 'Position', [80 195 110 225]);

% swing-up law + switching logic (hysteresis) in a MATLAB Function block
add_block('simulink/User-Defined Functions/MATLAB Function', [ct '/Swing-up and mode logic'], 'Position', [170 170 330 260]);
set_mfun([ct '/Swing-up and mode logic'], sprintf([ ...
    'function [F_su, mode, thw, E] = swingup(z, mode_prev, P)\n' ...
    '%% energy-based swing-up (Astrom-Furuta) + switching with hysteresis\n' ...
    'x = z(1); xd = z(2); th = z(3); thd = z(4);\n' ...
    'thw = atan2(sin(th), cos(th));\n' ...
    'mode = mode_prev;\n' ...
    'if mode_prev < 0.5 && abs(thw) < P.th_catch\n    mode = 1;\n' ...
    'elseif mode_prev > 0.5 && abs(thw) > P.th_release\n    mode = 0;\nend\n' ...
    'J = P.I + P.m*P.l^2;\n' ...
    'E = 0.5*J*thd^2 + P.m*P.g*P.l*(cos(th) - 1);\n' ...
    'E_ref = P.E_margin*P.m*P.g*P.l;\n' ...
    'a = P.ke*(E - E_ref)*thd*cos(th);\n' ...
    'a = min(max(a, -P.amax), P.amax) - P.kx_su*x - P.kv_su*xd;\n' ...
    'thdd = (P.m*P.g*P.l*sin(th) - P.c*thd - P.m*P.l*cos(th)*a)/J;\n' ...
    'F_su = (P.M + P.m)*a + P.m*P.l*(cos(th)*thdd - thd^2*sin(th)) + P.b*xd;\n']), {'P'});
add_block('simulink/Discrete/Unit Delay', [ct '/mode memory'], 'SampleTime', 'P.Ts', 'InitialCondition', '0', ...
    'Position', [190 290 230 320], 'Orientation', 'left');

% fuzzy balancing: normalized inputs -> FLC -> output gain, for Mamdani and Sugeno
add_block('simulink/Signal Routing/Demux', [ct '/Demux z'], 'Outputs', '4', 'Position', [170 390 175 560]);
types = {'Mamdani', 'Sugeno'}; tags = {'mam', 'sug'};
for k = 1:2
    y0 = 360 + (k-1)*260;
    sb = [ct '/Fuzzy balance ' types{k}];
    add_block('simulink/Ports & Subsystems/Subsystem', sb, 'Position', [420 y0 620 y0+180]);
    delete_line(sb, 'In1/1', 'Out1/1'); delete_block([sb '/In1']); delete_block([sb '/Out1']);
    inN = {'theta', 'thetadot', 'x', 'xdot'};
    gains = {'1/SC_%s.th_max', '1/SC_%s.thd_max', '1/SC_%s.x_max', '1/SC_%s.xd_max'};
    for i = 1:4
        add_block('simulink/Sources/In1', [sb '/' inN{i}], 'Position', [20 30+60*(i-1) 50 44+60*(i-1)]);
        add_block('simulink/Math Operations/Gain', [sb '/norm ' inN{i}], 'Gain', sprintf(gains{i}, tags{k}), ...
            'Position', [90 25+60*(i-1) 150 49+60*(i-1)]);
        add_block('simulink/Discontinuities/Saturation', [sb '/sat ' inN{i}], 'UpperLimit', '1', 'LowerLimit', '-1', ...
            'Position', [180 25+60*(i-1) 210 49+60*(i-1)]);
        add_line(sb, [inN{i} '/1'], ['norm ' inN{i} '/1']);
        add_line(sb, ['norm ' inN{i} '/1'], ['sat ' inN{i} '/1']);
    end
    add_block('simulink/Signal Routing/Mux', [sb '/mux th'], 'Inputs', '2', 'Position', [250 30 255 110]);
    add_block('simulink/Signal Routing/Mux', [sb '/mux x'], 'Inputs', '2', 'Position', [250 150 255 230]);
    add_block(FLC, [sb '/FLC angle'], 'FIS', sprintf('FIS_theta_%s', tags{k}), 'Position', [300 50 400 90]);
    add_block(FLC, [sb '/FLC cart'],  'FIS', sprintf('FIS_cart_%s', tags{k}),  'Position', [300 170 400 210]);
    add_block('simulink/Math Operations/Gain', [sb '/Ku angle'], 'Gain', sprintf('SC_%s.Ku_th', tags{k}), 'Position', [430 55 480 85]);
    add_block('simulink/Math Operations/Gain', [sb '/Ku cart'],  'Gain', sprintf('SC_%s.Ku_x', tags{k}),  'Position', [430 175 480 205]);
    add_block('simulink/Math Operations/Add', [sb '/sum'], 'Position', [520 110 550 150]);
    add_block('simulink/Sinks/Out1', [sb '/F_bal'], 'Position', [590 123 620 137]);
    add_line(sb, 'sat theta/1', 'mux th/1'); add_line(sb, 'sat thetadot/1', 'mux th/2');
    add_line(sb, 'sat x/1', 'mux x/1');      add_line(sb, 'sat xdot/1', 'mux x/2');
    add_line(sb, 'mux th/1', 'FLC angle/1'); add_line(sb, 'mux x/1', 'FLC cart/1');
    add_line(sb, 'FLC angle/1', 'Ku angle/1'); add_line(sb, 'FLC cart/1', 'Ku cart/1');
    add_line(sb, 'Ku angle/1', 'sum/1'); add_line(sb, 'Ku cart/1', 'sum/2');
    add_line(sb, 'sum/1', 'F_bal/1');
end

% selectors: Mamdani/Sugeno, then swing-up/balance, then actuator saturation
add_block('simulink/Sources/Constant', [ct '/CTRL_TYPE'], 'Value', 'CTRL_TYPE', 'Position', [640 560 690 580]);
add_block('simulink/Signal Routing/Switch', [ct '/Mamdani or Sugeno'], 'Criteria', 'u2 > Threshold', ...
    'Threshold', '1.5', 'Position', [700 430 740 610]);
add_block('simulink/Signal Routing/Switch', [ct '/Swing-up or balance'], 'Criteria', 'u2 > Threshold', ...
    'Threshold', '0.5', 'Position', [800 200 840 450]);
add_block('simulink/Discontinuities/Saturation', [ct '/Force limit'], 'UpperLimit', 'P.Fmax', 'LowerLimit', '-P.Fmax', ...
    'Position', [880 310 910 340]);
add_block('simulink/Sinks/Out1', [ct '/F'], 'Position', [960 318 990 332]);
add_block('simulink/Sinks/Out1', [ct '/mode'], 'Position', [960 378 990 392]);
add_block('simulink/Sinks/Out1', [ct '/E'], 'Position', [960 438 990 452]);

add_line(ct, 'z/1', 'ZOH/1');
add_line(ct, 'ZOH/1', 'Swing-up and mode logic/1', 'autorouting', 'on');
add_line(ct, 'mode memory/1', 'Swing-up and mode logic/2', 'autorouting', 'on');
add_line(ct, 'Swing-up and mode logic/2', 'mode memory/1', 'autorouting', 'on');
add_line(ct, 'ZOH/1', 'Demux z/1', 'autorouting', 'on');
for k = 1:2
    sb = ['Fuzzy balance ' types{k}];
    add_line(ct, 'Swing-up and mode logic/3', [sb '/1'], 'autorouting', 'on');   % wrapped theta
    add_line(ct, 'Demux z/4', [sb '/2'], 'autorouting', 'on');                   % thetadot
    add_line(ct, 'Demux z/1', [sb '/3'], 'autorouting', 'on');                   % x
    add_line(ct, 'Demux z/2', [sb '/4'], 'autorouting', 'on');                   % xdot
end
add_line(ct, 'Fuzzy balance Mamdani/1', 'Mamdani or Sugeno/3', 'autorouting', 'on');
add_line(ct, 'Fuzzy balance Sugeno/1',  'Mamdani or Sugeno/1', 'autorouting', 'on');
add_line(ct, 'CTRL_TYPE/1', 'Mamdani or Sugeno/2', 'autorouting', 'on');
add_line(ct, 'Mamdani or Sugeno/1', 'Swing-up or balance/1', 'autorouting', 'on');
add_line(ct, 'Swing-up and mode logic/2', 'Swing-up or balance/2', 'autorouting', 'on');
add_line(ct, 'Swing-up and mode logic/1', 'Swing-up or balance/3', 'autorouting', 'on');
add_line(ct, 'Swing-up or balance/1', 'Force limit/1', 'autorouting', 'on');
add_line(ct, 'Force limit/1', 'F/1', 'autorouting', 'on');
add_line(ct, 'Swing-up and mode logic/2', 'mode/1', 'autorouting', 'on');
add_line(ct, 'Swing-up and mode logic/4', 'E/1', 'autorouting', 'on');
% Demux z/3 (raw theta) is not used by the fuzzy branch (it uses the wrapped angle)
add_block('simulink/Sinks/Terminator', [ct '/unused'], 'Position', [220 500 240 520]);
add_line(ct, 'Demux z/3', 'unused/1', 'autorouting', 'on');

%% ======================= TOP LEVEL: wiring, logging, scopes =======================
add_line(mdl, 'Controller (Ts = 10 ms)/1', 'Digital Twin (Simscape Multibody)/1', 'autorouting', 'on');
add_line(mdl, 'Controller (Ts = 10 ms)/1', 'Equation model (reference)/1', 'autorouting', 'on');
add_line(mdl, 'Digital Twin (Simscape Multibody)/1', 'Controller (Ts = 10 ms)/1', 'autorouting', 'on');

logs = {'Digital Twin (Simscape Multibody)/1', 'z_twin', [880 120 940 140];
        'Equation model (reference)/1',        'z_eq',   [880 360 940 380];
        'Controller (Ts = 10 ms)/1',           'F',      [400 20 460 40];
        'Controller (Ts = 10 ms)/2',           'mode',   [400 280 460 300];
        'Controller (Ts = 10 ms)/3',           'E',      [400 320 460 340]};
for i = 1:size(logs, 1)
    add_block('simulink/Sinks/To Workspace', [mdl '/log ' logs{i,2}], 'VariableName', logs{i,2}, ...
        'SaveFormat', 'Timeseries', 'Position', logs{i,3});
    add_line(mdl, logs{i,1}, ['log ' logs{i,2} '/1'], 'autorouting', 'on');
end
add_block('simulink/Signal Routing/Demux', [mdl '/Demux twin'], 'Outputs', '4', 'Position', [820 170 825 260]);
add_line(mdl, 'Digital Twin (Simscape Multibody)/1', 'Demux twin/1', 'autorouting', 'on');
add_block('simulink/Sinks/Scope', [mdl '/Scope x, theta, F'], 'NumInputPorts', '3', 'Position', [900 170 940 260]);
add_line(mdl, 'Demux twin/1', 'Scope x, theta, F/1', 'autorouting', 'on');
add_line(mdl, 'Demux twin/3', 'Scope x, theta, F/2', 'autorouting', 'on');
add_line(mdl, 'Controller (Ts = 10 ms)/1', 'Scope x, theta, F/3', 'autorouting', 'on');
add_block('simulink/Sinks/Terminator', [mdl '/t2'], 'Position', [860 200 875 210]);
add_block('simulink/Sinks/Terminator', [mdl '/t4'], 'Position', [860 245 875 255]);
add_line(mdl, 'Demux twin/2', 't2/1', 'autorouting', 'on');
add_line(mdl, 'Demux twin/4', 't4/1', 'autorouting', 'on');

annotation = Simulink.Annotation([mdl '/README'], ...
    'Text', sprintf(['Cart-pendulum: energy swing-up + fuzzy balancing (Mamdani / Sugeno).\n' ...
    'Run setup_twin.m first. CTRL_TYPE = 1 (Mamdani) or 2 (Sugeno).\n' ...
    'The 3D digital twin opens in Mechanics Explorer when the simulation starts.']));
annotation.Position = [140 470 520 520];

try   % open the 3D Mechanics Explorer when the simulation starts (Simscape Multibody setting)
    set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', onoff(openEditor));
catch
end
save_system(mdl, f);
fprintf('Model saved: %s\n', f);
end

function s = onoff(b)
if b, s = 'on'; else, s = 'off'; end
end

function set_mfun(blk, code, params)
% Set the code of a MATLAB Function block and mark some inputs as parameters.
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = code;
for i = 1:numel(params)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', params{i});
    d.Scope = 'Parameter';
end
end
