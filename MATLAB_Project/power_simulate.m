% Project 2: Toi uu hoa phan bo cong suat (Water-filling) bang fmincon
clear; clc; close all;

% 1. Thong so he thong
N = 4;                           % So luong kenh con
h = [0.8; 0.5; 0.3; 0.1];        % He so kenh: Chuyen thanh VECTOR COT (4x1)
sigma2 = 0.01;                   % Cong suat nhieu
P_total = 1.0;                   % Tong cong suat phat toi da (Watt)

% 2. Ham muc tieu: Ep p(:) va h(:) ve vector cot de sum luon ra 1 SCALAR
% Capacity = sum(log2(1 + p_i * |h_i|^2 / sigma2))
objFun = @(p) -sum(log2(1 + (p(:) .* h(:)) / sigma2));

% 3. Rang buoc:
% - Tong cong suat: sum(p) <= P_total
A_ineq = ones(1, N);
b_ineq = P_total;

% - Cong suat tung kenh khong am: 0 <= p_i <= P_total
lb = zeros(N, 1);
ub = P_total * ones(N, 1);

% 4. Diem khoi tao: chia deu cong suat ban dau (4x1)
p0 = ones(N, 1) * (P_total / N);

% 5. Thiet lap va chay fmincon
options = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'Algorithm', 'interior-point');

[p_opt, min_val, exitflag] = fmincon(objFun, p0, A_ineq, b_ineq, [], [], lb, ub, [], options);

max_sum_rate = -min_val; % Tinh lai Sum-rate cuc dai (bps/Hz)

% 6. In ket qua ra Command Window
fprintf('\n==============================================\n');
fprintf('          KET QUA TOI UU CONG SUAT            \n');
fprintf('==============================================\n');
for i = 1:N
    fprintf('Kenh %d (Gain = %.2f): Cong suat p_%d = %.4f W\n', i, h(i), i, p_opt(i));
end
fprintf('----------------------------------------------\n');
fprintf('Tong cong suat tieu thu : %.4f W / %.2f W\n', sum(p_opt), P_total);
fprintf('Tong dung luong (Sum-Rate): %.4f bps/Hz\n', max_sum_rate);
fprintf('==============================================\n');

% 7. Ve do thi phan bo cong suat
figure('Name', 'Water-filling Optimization Result', 'Color', 'w');
bar(1:N, p_opt, 0.5, 'FaceColor', [0.15 0.5 0.8]);
grid on;
xlabel('Kenh con (Sub-channel)', 'FontSize', 11);
ylabel('Cong suat phan bo p_i (W)', 'FontSize', 11);
title('Phan bo cong suat toi uu (Kenh gain tot duoc cap nhieu cong suat hon)', 'FontSize', 12);
set(gca, 'XTick', 1:N);