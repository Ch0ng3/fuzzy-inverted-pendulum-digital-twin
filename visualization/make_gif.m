function make_gif(videoFile, gifFile, width, fps)
%MAKE_GIF Convert an .mp4 animation into a small looping GIF (e.g. for the README on GitHub).
if nargin < 3, width = 720; end
if nargin < 4, fps = 10; end
v = VideoReader(videoFile);
step = 1/fps; t = 0; first = true;
while t < v.Duration
    v.CurrentTime = t;
    img = imresize(readFrame(v), width/v.Width);
    [A, map] = rgb2ind(img, 128);
    if first
        imwrite(A, map, gifFile, 'gif', 'LoopCount', Inf, 'DelayTime', step); first = false;
    else
        imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', step);
    end
    t = t + step;
end
fprintf('GIF saved: %s\n', gifFile);
end
