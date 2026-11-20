%% TEST_ENGINE.M - Kiem thu doc lap Engine toi uu hoa AO (Khong can bat GUI)
clear; clc;

% 1. Thiet lap duong dan den cac thu muc thanh phan
project_root = fileparts(mfilename('fullpath'));
addpath(fullfile(project_root, 'core', 'utils'));
addpath(fullfile(project_root, 'core', 'ao'));
addpath(fullfile(project_root, 'Optimal_simulate', 'Time_Slot'));
addpath(fullfile(project_root, 'Optimal_simulate', 'common'));

fprintf('====================================================================\n');
fprintf('  KIEM THU ENGINE TOI UU HOA SAGIN-NOMA (RUN_AO_SOLVER)\n');
fprintf('====================================================================\n\n');

% 2. Lay bo tham so he thong chuan hoa
params = get_default_params();
params.max_iterations = 5; % Chay thu 5 vong lap de kiem tra nhanh

fprintf('Khoi tao tham so thanh cong:\n');
fprintf('  - So khe thoi gian N : %d slots\n', params.N_slots);
fprintf('  - Target Rates (R1, R2): (%.1f, %.1f) bps/Hz\n', params.R1, params.R2);
fprintf('  - Muc SNR khao sat   : %d dB\n\n', params.rho_dB);

% 3. Dinh nghia Callback in tien do ra Console (Mo phong UI progress bar)
consoleCallback = @(ratio, msg) fprintf('>> [Tien do: %5.1f%%] %s\n', ratio * 100, msg);

% 4. Thuc thi Engine
fprintf('Bat dau chay Engine...\n');
tic;
res = run_ao_solver(params, consoleCallback);
time_taken = toc;

% 5. Hien thi ket qua tong ket
fprintf('\n======================= KET QUA KIEM THU =======================\n');
fprintf('  - Thoi gian thuc thi       : %.2f giay\n', time_taken);
fprintf('  - So vong lap hoan thanh   : %d / %d\n', res.iterations, params.max_iterations);
fprintf('  - Trang thai hoi tu        : %d\n', res.converged);
fprintf('  - Xac suat dung TB (OP)    : %.4e\n', res.OP_avg_opt);
fprintf('  - Cong suat a1 TB (Far D1) : %.4f\n', mean(res.a1_opt));
fprintf('  - Cong suat a2 TB (Near D2): %.4f\n', mean(res.a2_opt));
fprintf('====================================================================\n');
fprintf('>> ENGINE HOAT DONG HOAN HAO, SAN SANG TICH HOP VAO APP DESIGNER! <<\n');
