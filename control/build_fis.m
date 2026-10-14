function C = build_fis(P, K)
%BUILD_FIS Build the fuzzy balancing controllers (Mamdani and Sugeno).
%
%   Two PD-like fuzzy controllers are summed:
%       F = Ku_th * FLC(theta/th_max, thetadot/thd_max) + Ku_x * FLC(x/x_max, xdot/xd_max)
%   Each FLC has two normalized inputs in [-1, 1] with 7 linguistic values
%   (NB NM NS Z PS PM PB), the classical PD-like rule table u = sat(e + de)
%   (49 rules) and a normalized output in [-1, 1].
%     - Mamdani: min AND, min implication, max aggregation, centroid (COG).
%     - Sugeno : min AND, zero-order (constant) outputs, weighted average.
%   The same rule table is used by both, so the comparison is fair.
%
%   Scaling: the normalization ranges are chosen so that, around the upright
%   position, the slope of each FLC matches the LQR gain K (small-signal
%   equivalence). Far from the origin the FLC saturates smoothly.

labels = {'NB','NM','NS','Z','PS','PM','PB'};
c = (-3:3)/3;                      % centres of the 7 fuzzy sets
w = 1/3;                           % half width of the triangles

% ---------- normalization ranges (inputs) ----------
C.th_max  = P.th_catch;            % [rad]   angle range of the balance controller
C.x_max   = P.xmax;                % [m]     cart position range

for type = ["mamdani", "sugeno"]
    for loop = ["theta", "cart"]
        if type == "mamdani"
            fis = mamfis('Name', char("FLC_" + loop + "_mamdani"), 'AndMethod', 'min', ...
                'ImplicationMethod', 'min', 'AggregationMethod', 'max', ...
                'DefuzzificationMethod', 'centroid');
        else
            fis = sugfis('Name', char("FLC_" + loop + "_sugeno"), 'AndMethod', 'min', ...
                'DefuzzificationMethod', 'wtaver');
        end
        if loop == "theta", inNames = ["e_theta", "de_theta"]; else, inNames = ["e_x", "de_x"]; end

        % inputs: triangles inside, shoulders (trapezoids) at the ends
        for in = inNames
            fis = addInput(fis, [-1 1], 'Name', char(in));
            for k = 1:7
                if k == 1
                    fis = addMF(fis, char(in), 'trapmf', [-2 -2 c(1) c(2)], 'Name', labels{k});
                elseif k == 7
                    fis = addMF(fis, char(in), 'trapmf', [c(6) c(7) 2 2], 'Name', labels{k});
                else
                    fis = addMF(fis, char(in), 'trimf', [c(k)-w c(k) c(k)+w], 'Name', labels{k});
                end
            end
        end

        % output
        fis = addOutput(fis, [-1 1], 'Name', 'u');
        for k = 1:7
            if type == "mamdani"
                fis = addMF(fis, 'u', 'trimf', [c(k)-w c(k) c(k)+w], 'Name', labels{k});
            else
                fis = addMF(fis, 'u', 'constant', c(k), 'Name', labels{k});
            end
        end

        % rule table: output index = sat(i + j)   (PD-like, MacVicar-Whelan)
        rules = zeros(49, 5); r = 0;
        for i = 1:7
            for j = 1:7
                r = r + 1;
                o = min(max((i-4) + (j-4), -3), 3) + 4;
                rules(r,:) = [i j o 1 1];          % weight 1, AND
            end
        end
        fis = addRule(fis, rules);

        C.(char(type + "_" + loop)) = fis;
    end
end

% ---------- small-signal calibration against the LQR ----------
% LQR (F = -K z): F = -K(1) x - K(2) xdot - K(3) theta - K(4) thetadot
kx = -K(1); kv = -K(2); kth = -K(3); kthd = -K(4);

% slopes of the normalized FLC at the origin (numerical), for each FIS type
h = 0.02;
for type = ["mamdani", "sugeno"]
    fisT = C.(char(type + "_theta"));
    s_e  = evalfis(fisT, [h 0]) / h;          % du/de_n
    s_de = evalfis(fisT, [0 h]) / h;          % du/dde_n
    % F_theta = Ku * u(theta/th_max, thd/thd_max): slope wrt theta = Ku*s_e/th_max
    Ku_th  = kth * C.th_max / s_e;
    thd_max = Ku_th * s_de / kthd;
    Ku_x   = kx * C.x_max / s_e;
    xd_max = Ku_x * s_de / kv;
    C.(char(type + "_scale")) = struct('Ku_th', Ku_th, 'thd_max', thd_max, ...
        'Ku_x', Ku_x, 'xd_max', xd_max, 'slope_e', s_e, 'slope_de', s_de);
end
C.labels = labels;
end
