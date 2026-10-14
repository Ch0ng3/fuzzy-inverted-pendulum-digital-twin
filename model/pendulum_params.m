function P = pendulum_params()
%PENDULUM_PARAMS Physical and control parameters of the cart-pendulum system.
%   Angle convention: theta = 0 upright, theta = pi hanging down,
%   positive theta = counter-clockwise; x positive to the right.

% --- physical parameters (uniform rod on a cart) ---
P.M  = 0.5;          % cart mass [kg]
P.m  = 0.2;          % pendulum mass [kg]
P.l  = 0.3;          % pivot -> centre of mass [m] (rod length 2*l = 0.6 m)
P.I  = P.m*(2*P.l)^2/12;   % rod inertia about its centre of mass [kg m^2]
P.b  = 0.1;          % cart viscous friction [N s/m]
P.c  = 0.002;        % pivot viscous friction [N m s/rad]
P.g  = 9.81;         % gravity [m/s^2]

% --- actuator and track ---
P.Fmax   = 15;       % force saturation [N]
P.xmax   = 1.0;      % half track length [m] (soft limit used by the controllers)

% --- control timing ---
P.Ts     = 0.01;     % controller sampling period [s] (100 Hz)

% --- swing-up (energy based) ---
P.ke     = 10;       % energy-pumping gain [m/s^2 per J]
P.amax   = 4;        % max cart acceleration during swing-up [m/s^2]
P.kx_su  = 4.0;      % cart centering gains during swing-up
P.kv_su  = 4.0;      % (tuned: catch ~4.3 s, 3 swings, |x| < 0.3 m)
P.E_margin = 0.10;   % pump to E_ref = E_margin*m*g*l above the upright energy (robust to losses)

% --- switching swing-up -> balance (with hysteresis) ---
P.th_catch   = 0.35; % switch to balance when |theta| < th_catch [rad] (~20 deg)
P.th_release = 0.80; % fall back to swing-up when |theta| > th_release [rad]

% --- initial state [x, xdot, theta, thetadot] ---
P.x0 = [0; 0; pi - 0.05; 0];   % hanging down, slightly off
end
