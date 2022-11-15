addpath('quaternion_library');	% include quatenrion library
close all;

% p
figure(1);
plot3(p(:,1), p(:,2), p(:,3));
axis([0 1 0 1 -5.5 -4.5]);

% p
figure(2);
subplot(3,1,1);
plot(p(:,1));
subplot(3,1,2);
plot(p(:,2));
subplot(3,1,3);
plot(p(:,3));

% v
figure(3);
subplot(3,1,1);
plot(v(:,1));
subplot(3,1,2);
plot(v(:,2));
subplot(3,1,3);
plot(v(:,3));

% v
figure(4);
subplot(3,1,1);
plot(w(:,1));
subplot(3,1,2);
plot(w(:,2));
subplot(3,1,3);
plot(w(:,3));

% control
figure(5);
subplot(4,1,1);
plot(T);
subplot(4,1,2);
plot(M(:,1));
subplot(4,1,3);
plot(M(:,2));
subplot(4,1,4);
plot(M(:,3));

% q
R = quatern2rotMat(q);
SixDOFanimation(p, R, ...
                'SamplePlotFreq', 1, 'Trail', 'Off', ...
                'Position', [9 39 1280 720], ...
                'AxisLength', 0.1, 'ShowArrowHead', true, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', 30);           
            

