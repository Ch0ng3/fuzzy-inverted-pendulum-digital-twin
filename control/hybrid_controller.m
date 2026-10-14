function [F, mode, info] = hybrid_controller(z, mode, P, C, fisType)
%HYBRID_CONTROLLER Energy-based swing-up + fuzzy balancing, with hysteresis.
%   z = [x; xdot; theta; thetadot], mode = 0 (swing-up) or 1 (balance),
%   fisType = "mamdani" or "sugeno".

x = z(1); xd = z(2); th = z(3); thd = z(4);
thw = atan2(sin(th), cos(th));          % angle wrapped to [-pi, pi], 0 = upright

% ---- mode switching with hysteresis ----
if mode == 0 && abs(thw) < P.th_catch
    mode = 1;                            % close enough to upright: catch it
elseif mode == 1 && abs(thw) > P.th_release
    mode = 0;                            % lost it: go back to swing-up
end

J = P.I + P.m*P.l^2;
E = 0.5*J*thd^2 + P.m*P.g*P.l*(cos(th) - 1);    % energy, 0 at upright rest

if mode == 0
    % ---- swing-up: pump energy towards E_ref >= 0 (Astrom-Furuta) ----
    E_ref = P.E_margin*P.m*P.g*P.l;     % slightly above the upright energy
    a = P.ke * (E - E_ref) * thd * cos(th);
    a = min(max(a, -P.amax), P.amax);
    a = a - P.kx_su*x - P.kv_su*xd;      % keep the cart near the centre
    % force that produces cart acceleration a (inverse dynamics)
    thdd = (P.m*P.g*P.l*sin(th) - P.c*thd - P.m*P.l*cos(th)*a) / J;
    F = (P.M + P.m)*a + P.m*P.l*(cos(th)*thdd - thd^2*sin(th)) + P.b*xd;
    uth = NaN; ux = NaN;
else
    % ---- fuzzy balancing ----
    S = C.(char(fisType + "_scale"));
    in_th = [thw/C.th_max, thd/S.thd_max];
    in_x  = [x/C.x_max,    xd/S.xd_max];
    in_th = min(max(in_th, -1), 1);       % stay inside the universe of discourse
    in_x  = min(max(in_x,  -1), 1);
    uth = evalfis(C.(char(fisType + "_theta")), in_th);
    ux  = evalfis(C.(char(fisType + "_cart")),  in_x);
    F = S.Ku_th*uth + S.Ku_x*ux;
end

F = min(max(F, -P.Fmax), P.Fmax);
info = struct('E', E, 'uth', uth, 'ux', ux);
end
