function dz = pendulum_dynamics(z, F, P)
%PENDULUM_DYNAMICS Nonlinear cart-pendulum model.
%   z = [x; xdot; theta; thetadot], F = horizontal force on the cart [N].
%   Equations (Lagrange, theta = 0 upright):
%     (M+m) xdd + m l cos(th) thdd - m l thd^2 sin(th) + b xd = F
%     m l cos(th) xdd + (I + m l^2) thdd - m g l sin(th) + c thd = 0

xd = z(2); th = z(3); thd = z(4);
s = sin(th); c = cos(th);

A = [P.M + P.m,      P.m*P.l*c;
     P.m*P.l*c,      P.I + P.m*P.l^2];
rhs = [F + P.m*P.l*thd^2*s - P.b*xd;
       P.m*P.g*P.l*s - P.c*thd];
acc = A \ rhs;

dz = [xd; acc(1); thd; acc(2)];
end
