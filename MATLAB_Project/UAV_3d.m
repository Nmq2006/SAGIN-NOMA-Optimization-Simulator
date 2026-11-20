% Project 4: Mo phong quy dao bay UAV 3D theo Waypoints
clear; clc; close all;

% 1. Dinh nghia cac diem moc (Waypoints: [X, Y, Z] theo met)
waypoints = [ 0   0   0;
             50  30  20;
            100  80  50;
            150  40  40;
            200 100  10;
              0   0   0]; % Quay ve vi tri xuat phat

% Thoi gian den tung waypoint (giay)
timePoints = [0, 10, 25, 40, 55, 75];

% 2. Tao quy dao noi suy muot (Trapezoidal / Polynomial Trajectory)
t = 0:0.5:timePoints(end); % Thoi gian lay mau
trajectory = minjerkpolytraj(waypoints', timePoints, length(t));
uavPositions = trajectory';

% 3. Ve truc quan quy dao bay 3D
figure('Color', 'w');
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'rs--', ...
    'LineWidth', 1.5, 'MarkerSize', 8, 'MarkerFaceColor', 'r'); hold on;
plot3(uavPositions(:,1), uavPositions(:,2), uavPositions(:,3), 'b-', 'LineWidth', 2);
grid on; box on;
xlabel('Toa do X (m)'); ylabel('Toa do Y (m)'); zlabel('Do cao Z (m)');
title('Mo phong quy dao bay 3D cua UAV qua cac Waypoints');
legend('Diem Waypoint chi dinh', 'Quy dao bay thuc te');
view(45, 30);