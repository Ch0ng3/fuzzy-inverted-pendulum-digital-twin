function animate_pendulum(t, z, F, mode, P, ttl, videoFile, speed)
%ANIMATE_PENDULUM 2D animated dashboard of the cart-pendulum (digital twin view).
%   animate_pendulum(t, z, F, mode, P, title, videoFile, speed)
%     t, z (N x 4: x, xdot, theta, thetadot), F, mode: simulation results
%     videoFile: optional .mp4 path to record the animation ('' = no video)
%     speed:     playback speed factor (1 = real time)

if nargin < 6 || isempty(ttl), ttl = 'Cart-pendulum'; end
if nargin < 7, videoFile = ''; end
if nargin < 8, speed = 1; end
fps = 30;
tf = (t(1):speed/fps:t(end))';                       % animation frames
zi = interp1(t, z, tf); Fi = interp1(t, F, tf); mi = interp1(t, mode, tf, 'previous');
thw = atan2(sin(z(:,3)), cos(z(:,3)));

L = 2*P.l; cartW = 0.25; cartH = 0.10; trackX = P.xmax + 0.2;
f = figure('Color', 'w', 'Position', [60 60 1280 720], 'Name', ttl, 'Visible', onoff(isempty(videoFile)));
tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, ttl, 'FontWeight', 'bold');

% ---- scene ----
ax = nexttile(tl, 1, [3 1]); hold(ax, 'on'); axis(ax, 'equal'); box(ax, 'on');
xlim(ax, [-trackX trackX]); ylim(ax, [-L-0.25, L+0.25]);
plot(ax, [-trackX trackX], [-cartH/2 -cartH/2], 'Color', [0.4 0.4 0.4], 'LineWidth', 3);   % track
plot(ax, [-P.xmax -P.xmax; P.xmax P.xmax]', [-0.1 0.1; -0.1 0.1]', 'r', 'LineWidth', 2);   % limits
cart = rectangle(ax, 'Position', [-cartW/2 -cartH/2 cartW cartH], 'Curvature', 0.15, ...
    'FaceColor', [0.15 0.35 0.75], 'EdgeColor', 'k');
rod  = plot(ax, [0 0], [0 L], 'Color', [0.85 0.25 0.15], 'LineWidth', 6);
bob  = plot(ax, 0, L, 'o', 'MarkerSize', 12, 'MarkerFaceColor', [0.85 0.25 0.15], 'MarkerEdgeColor', 'k');
trail = plot(ax, nan, nan, ':', 'Color', [0.85 0.25 0.15 0.5]);
arrow = quiver(ax, 0, 0, 0, 0, 0, 'Color', [0 0.6 0], 'LineWidth', 2.5, 'MaxHeadSize', 0.8);
txt = text(ax, -trackX+0.05, L+0.15, '', 'FontSize', 12, 'FontWeight', 'bold');
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');

% ---- live plots ----
a1 = nexttile(tl, 2); hold(a1, 'on'); grid(a1, 'on');
plot(a1, t, rad2deg(thw), 'Color', [0.8 0.8 0.8]); h1 = plot(a1, nan, nan, 'Color', [0.85 0.25 0.15], 'LineWidth', 1.5);
ylabel(a1, '\theta [deg]'); xlim(a1, [t(1) t(end)]); ylim(a1, [-190 190]); yline(a1, 0, 'k:');
a2 = nexttile(tl, 4); hold(a2, 'on'); grid(a2, 'on');
plot(a2, t, z(:,1), 'Color', [0.8 0.8 0.8]); h2 = plot(a2, nan, nan, 'Color', [0.15 0.35 0.75], 'LineWidth', 1.5);
ylabel(a2, 'x [m]'); xlim(a2, [t(1) t(end)]); ylim(a2, [-P.xmax P.xmax]);
a3 = nexttile(tl, 6); hold(a3, 'on'); grid(a3, 'on');
plot(a3, t, F, 'Color', [0.8 0.8 0.8]); h3 = plot(a3, nan, nan, 'Color', [0 0.6 0], 'LineWidth', 1.5);
ylabel(a3, 'F [N]'); xlabel(a3, 't [s]'); xlim(a3, [t(1) t(end)]); ylim(a3, [-P.Fmax P.Fmax]);
cur = [xline(a1, 0, 'k'), xline(a2, 0, 'k'), xline(a3, 0, 'k')];

if ~isempty(videoFile)
    vw = VideoWriter(videoFile, 'MPEG-4'); vw.FrameRate = fps; vw.Quality = 90; open(vw);
end
tr = nan(numel(tf), 2);
for k = 1:numel(tf)
    x = zi(k,1); th = zi(k,3);
    px = x + L*sin(th); py = L*cos(th);                 % rod tip (theta = 0 upright)
    set(cart, 'Position', [x-cartW/2, -cartH/2, cartW, cartH]);
    set(rod, 'XData', [x px], 'YData', [0 py]); set(bob, 'XData', px, 'YData', py);
    tr(k,:) = [px py]; k0 = max(1, k-45); set(trail, 'XData', tr(k0:k,1), 'YData', tr(k0:k,2));
    set(arrow, 'XData', x, 'YData', -cartH/2-0.08, 'UData', 0.03*Fi(k), 'VData', 0);
    if mi(k) > 0.5, md = 'BALANCE (fuzzy)'; col = [0 0.5 0]; else, md = 'SWING-UP (energy)'; col = [0.8 0.4 0]; end
    set(txt, 'String', sprintf('t = %5.2f s   %s', tf(k), md), 'Color', col);
    n = t <= tf(k);
    set(h1, 'XData', t(n), 'YData', rad2deg(thw(n)));
    set(h2, 'XData', t(n), 'YData', z(n,1));
    set(h3, 'XData', t(n), 'YData', F(n));
    set(cur, 'Value', tf(k));
    if isempty(videoFile)
        drawnow; pause(1/fps);
    else
        drawnow; writeVideo(vw, getframe(f));
    end
end
if ~isempty(videoFile), close(vw); close(f); fprintf('Video saved: %s\n', videoFile); end
end

function s = onoff(b)
if b, s = 'on'; else, s = 'off'; end
end
