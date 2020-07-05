clear;
close all;
clc;
% reading file
fileID = fopen('data.txt', 'r');
formatSpec = '%f';
size = [3 1000];
data = fscanf(fileID, formatSpec, size);

Obstacle = [-1, -1;
            0, 2;
            4, 2;
            5, 4;
            5, 5;
            5, 6;
            5, 9;
            8, 9;
            7, 9;
            12, 12];

figure(1);
%Plot Result
set(gca, 'fontsize', 16, 'fontname', 'times');
for i = 1:length(Obstacle)
    plot(Obstacle(i, 1), Obstacle(i, 2),'o'); hold on;
end
plot(data(2, :), data(3, :),'-.b'); hold on;
title('Dynamic Window Approach', 'fontsize', 16, 'fontname', 'times');
xlabel('X (m)', 'fontsize', 16, 'fontname', 'times');
ylabel('Y (m)', 'fontsize', 16, 'fontname', 'times');
legend('Ground Truth','Obstacle');
grid on;
axis equal;

% Auto save graph
saveas(gcf, 'DWA.png');