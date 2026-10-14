function R = simulate_pendulum(P, C, fisType, Tend, opts)
%SIMULATE_PENDULUM Closed-loop simulation in MATLAB (no Simulink): the plant is
%   integrated with RK4 at 1 kHz, the controller runs at P.Ts with zero-order hold.
%
%   opts (optional struct):
%     .z0     initial state [x; xdot; theta; thetadot]     (default P.x0)
%     .Pplant parameters of the REAL plant (model mismatch) (default P)
%     .kick   [t0, dthetadot]: sudden push on the pendulum at time t0
%     .noise  [std_x, std_theta]: measurement noise (Gaussian) on x and theta
%     .seed   random seed for the noise

if nargin < 5, opts = struct(); end
if ~isfield(opts, 'z0'),     opts.z0 = P.x0; end
if ~isfield(opts, 'Pplant'), opts.Pplant = P; end
if ~isfield(opts, 'kick'),   opts.kick = []; end
if ~isfield(opts, 'noise'),  opts.noise = [0 0]; end
if ~isfield(opts, 'seed'),   opts.seed = 1; end
rng(opts.seed);

dt = 1e-3; nsub = round(P.Ts/dt);
N = round(Tend/P.Ts);
Pp = opts.Pplant;

R.t = (0:N)'*P.Ts;
R.z = zeros(N+1, 4); R.F = zeros(N+1,1); R.mode = zeros(N+1,1); R.E = zeros(N+1,1);
z = opts.z0; mode = 0; zprev_meas = z; kicked = false;
R.z(1,:) = z';
for k = 1:N
    % sudden push on the pendulum
    if ~isempty(opts.kick) && ~kicked && R.t(k) >= opts.kick(1)
        z(4) = z(4) + opts.kick(2); kicked = true;
    end
    % measurement: noisy positions, velocities by finite differences (as on real hardware)
    zm = z;
    if any(opts.noise)
        zm(1) = z(1) + opts.noise(1)*randn;
        zm(3) = z(3) + opts.noise(2)*randn;
        zm(2) = (zm(1) - zprev_meas(1))/P.Ts;
        zm(4) = (zm(3) - zprev_meas(3))/P.Ts;
        zprev_meas = zm;
    end
    [F, mode, info] = hybrid_controller(zm, mode, P, C, fisType);
    R.F(k) = F; R.mode(k) = mode; R.E(k) = info.E;
    for s = 1:nsub                                   % RK4 with ZOH force
        k1 = pendulum_dynamics(z, F, Pp);
        k2 = pendulum_dynamics(z + dt/2*k1, F, Pp);
        k3 = pendulum_dynamics(z + dt/2*k2, F, Pp);
        k4 = pendulum_dynamics(z + dt*k3, F, Pp);
        z = z + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    end
    R.z(k+1,:) = z';
end
R.F(end) = R.F(end-1); R.mode(end) = R.mode(end-1); R.E(end) = R.E(end-1);
R.thw = atan2(sin(R.z(:,3)), cos(R.z(:,3)));

% ---- metrics ----
iCatch = find(R.mode == 1, 1);
R.t_catch = NaN; if ~isempty(iCatch), R.t_catch = R.t(iCatch); end
last = R.t >= Tend - 2;                                   % last 2 seconds
R.balanced = all(R.mode(last) == 1) && all(abs(R.thw(last)) < 0.05);
R.max_x = max(abs(R.z(:,1)));
R.max_F = max(abs(R.F));
R.n_switch = sum(abs(diff(R.mode)));
% balance-phase metrics (from the catch on)
if ~isempty(iCatch)
    b = iCatch:numel(R.t);
    R.bal_max_theta = max(abs(R.thw(b)));
    R.bal_rms_theta = sqrt(mean(R.thw(b).^2));
    R.bal_rms_x     = sqrt(mean(R.z(b,1).^2));
    R.bal_effort    = sum(R.F(b).^2)*P.Ts;               % integral of F^2
    % settling: last time |theta| > 0.02 rad (~1 deg) after the catch
    out = find(abs(R.thw(b)) > 0.02, 1, 'last');
    R.t_settle = R.t(b(1) + max(out, 1) - 1) - R.t_catch;
else
    [R.bal_max_theta, R.bal_rms_theta, R.bal_rms_x, R.bal_effort, R.t_settle] = deal(NaN);
end
R.fisType = fisType;
end
