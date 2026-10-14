function [K, sys] = design_lqr(P)
%DESIGN_LQR Linearize the cart-pendulum around the upright position and compute
%   an LQR gain. The LQR is only a REFERENCE: its gains are used to scale the
%   fuzzy controllers so that, for small errors, they behave like a good linear
%   controller (F = -K z).

% numerical linearization around z = 0 (upright), F = 0
z0 = zeros(4,1); h = 1e-6;
A = zeros(4); B = zeros(4,1);
f0 = pendulum_dynamics(z0, 0, P);
for i = 1:4
    dz = z0; dz(i) = h;
    A(:,i) = (pendulum_dynamics(dz, 0, P) - f0)/h;
end
B(:,1) = (pendulum_dynamics(z0, h, P) - f0)/h;

Q = diag([10, 1, 100, 1]);   % weights on x, xdot, theta, thetadot
R = 0.1;
K = lqr(A, B, Q, R);
sys = ss(A, B, eye(4), 0);
end
