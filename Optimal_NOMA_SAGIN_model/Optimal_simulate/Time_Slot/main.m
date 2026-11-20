%% =========================================================================
%% ĐỀ TÀI NCKH: ĐÁNH GIÁ HIỆU NĂNG VÀ TỐI ƯU HÓA MẠNG SAGIN-NOMA DÙNG UAV
%% SCRIPT CHÍNH: main.m (MÔ HÌNH 2D Oxz THEO TIME-SLOTTING & RANH GIỚI HÌNH HỌC)
%% Thư mục: Optimal_simulate/Time_Slot/
%% =========================================================================

if ~exist('KEEP_WORKSPACE', 'var')
    clear;
end
clc; close all;

%% -------------------------------------------------------------------------
%% KHÂU 1: THIẾT LẬP THAM SỐ HỆ THỐNG 2D (Oxz) CHUẨN HÓA
%% -------------------------------------------------------------------------
fprintf('================================================================================\n');
fprintf('   KHỞI TẠO THAM SỐ MÔ HÌNH 2D (Oxz) CHUẨN HÓA THEO TIME-SLOTTING & SIC BOUNDARY\n');
fprintf('================================================================================\n');

% 1. Tọa độ chuẩn hóa không thứ nguyên [x, z] trong mặt phẳng (Oxz) (Đồng nhất với folder AO)
pos_S  = [0.0, 1.0];           % Vệ tinh nguồn S tại đỉnh z = 1.0
pos_D2 = [1.0, 0.0];           % Người dùng gần D2 tại x = 1.0
pos_D1 = [1.5, 0.0];           % Người dùng xa D1 tại x = 1.5

% Hành lang bay an toàn của UAV trong mặt phẳng (Oxz): [x, z]
uav_lb = [0.2, 0.2];           % [Xmin, Zmin]
uav_ub = [1.8, 0.6];           % [Xmax, Zmax]

% 2. Tham số kênh truyền và fading Rician
nu_SU = 2.0;                   % Số mũ suy hao chặng Vệ tinh - UAV (Free-space LoS)
nu_UD = 2.5;                   % Số mũ suy hao chặng UAV - Người dùng mặt đất
K_SU  = 4;                     % Hệ số Rician chặng 1 (~6 dB)
K_UD  = 2;                     % Hệ số Rician chặng 2 (~3 dB)
N_UAV = 4;                     % Số lượng UAV trong cụm (cơ chế PRS)

% 3. Yêu cầu tốc độ truyền và ngưỡng giải mã SINR (Công thức 10 & 11)
R1 = 0.8;                      % Target rate D1 (bps/Hz)
R2 = 1.2;                      % Target rate D2 (bps/Hz)
gamma_th1 = 2^(2 * R1) - 1;
gamma_th2 = 2^(2 * R2) - 1;

% 4. Tham số Time-slotting & Khí động học UAV
T_total = 20.0;                % Tổng thời gian bay khảo sát (giây)
N_slots = 10;                  % Số lượng khe thời gian rời rạc N
dt      = T_total / N_slots;   % Độ dài mỗi khe dt (giây)
V_max   = 0.12;                % Vận tốc tối đa cho phép của UAV (đơn vị chuẩn hóa / giây)

% Điểm xuất phát (q_start) và điểm đích (q_end) của UAV trong mặt phẳng Oxz
q_start = [0.35, 0.50];        % Vị trí ban đầu gần vệ tinh
q_end   = [1.10, 0.30];        % Vị trí đích tiếp cận người dùng gần D2

% 5. Ranh giới hình học cố định vị thế giải mã SIC (User Ordering Constraint)
% Đường trung trực giữa D2(1.0, 0) và D1(1.5, 0) là x = 1.25.
% Đảm bảo: ||q[n] - pos_D2|| <= ||q[n] - pos_D1|| - SIC_margin
SIC_margin = 0.04;             % Biên độ an toàn hình học

% 6. Tỉ số tín hiệu trên nhiễu trung bình rho tại điểm thiết kế (dB)
rho_dB = 20;                   % Mức SNR khảo sát hội tụ (20 dB)
rho = 10^(rho_dB / 10);

% 7. Cấu hình tiêu chuẩn hội tụ thuật toán AO
max_iterations = 20;           % Số vòng lặp tối đa
epsilon = 1e-4;                % Ngưỡng dừng sai số tương đối hàm mục tiêu |F(k+1) - F(k)|

% Đóng gói tất cả tham số vào struct params
params.pos_S          = pos_S;
params.pos_D1         = pos_D1;
params.pos_D2         = pos_D2;
params.uav_lb         = uav_lb;
params.uav_ub         = uav_ub;
params.nu_SU          = nu_SU;
params.nu_UD          = nu_UD;
params.K_SU           = K_SU;
params.K_UD           = K_UD;
params.N_UAV          = N_UAV;
params.gamma_th1      = gamma_th1;
params.gamma_th2      = gamma_th2;
params.T_total        = T_total;
params.N_slots        = N_slots;
params.dt             = dt;
params.V_max          = V_max;
params.q_start        = q_start;
params.q_end          = q_end;
params.SIC_margin     = SIC_margin;
params.max_iterations = max_iterations;
params.epsilon        = epsilon;

% Khởi tạo cấu hình ban đầu: Quỹ đạo bay thẳng đều và Phân bổ công suất cố định
Q_init = zeros(N_slots, 2);
for n = 1:N_slots
    alpha = (n - 1) / (N_slots - 1);
    Q_init(n, :) = (1 - alpha) * q_start + alpha * q_end;
end
a1_init = 0.75 * ones(N_slots, 1);

fprintf('--> Tham số Time-slotting: T = %.1fs, N = %d khe, dt = %.2fs\n', T_total, N_slots, dt);
fprintf('--> Giới hạn dịch chuyển tối đa mỗi khe: V_max * dt = %.3f\n', V_max * dt);
fprintf('--> Ngưỡng công suất SIC khả thi: alpha1 > %.4f (Khởi tạo alpha1 = %.2f)\n\n', ...
        gamma_th1 / (1 + gamma_th1), a1_init(1));

%% -------------------------------------------------------------------------
%% KHÂU 2: CHẠY THỬ NGHIỆM THUẬT TOÁN AO TIME-SLOTTING TẠI SNR THIẾT KẾ (20 dB)
%% -------------------------------------------------------------------------
fprintf('================================================================================\n');
fprintf('   CHẠY THỬ NGHIỆM THUẬT TOÁN AO 2D (Oxz) TIME-SLOTTING (SNR = %d dB)         \n', rho_dB);
fprintf('================================================================================\n');

res_ao = run_timeslot_ao(rho, params, Q_init, a1_init);

fprintf('\n------------------ KẾT QUẢ TỐI ƯU HỆ THỐNG ------------------\n');
fprintf('Xác suất dừng TB tối ưu : OP_total* = %.6e\n', res_ao.OP_avg_opt);
fprintf('Hệ số công suất TB tối ưu: alpha1* = %.4f | alpha2* = %.4f\n', ...
        mean(res_ao.a1_opt), mean(res_ao.a2_opt));
fprintf('Số vòng lặp thực hiện   : %d vòng\n', res_ao.iterations);
fprintf('-------------------------------------------------------------\n\n');

%% -------------------------------------------------------------------------
%% KHÂU 3: VẼ HÌNH 1 - ĐỒ THỊ ĐƯỜNG CONG HỘI TỤ (TƯƠNG TỰ FOLDER AO)
%% -------------------------------------------------------------------------
figure('Name', 'Convergence Curve of Time-Slot AO 2D', 'Color', 'w', 'Position', [100, 100, 680, 500]);
iter_vec = 0:(length(res_ao.history_F) - 1);

semilogy(iter_vec, res_ao.history_F, '-o', ...
    'LineWidth', 2, 'Color', [0 0.4470 0.7410], ...
    'MarkerSize', 7, 'MarkerFaceColor', [0 0.4470 0.7410]);

grid on; grid minor;
xlabel('Số vòng lặp (Iteration k)', 'FontSize', 12);
ylabel('Xác suất dừng toàn phần trung bình OP_{total}', 'FontSize', 12);
title(['Đường cong hội tụ của Thuật toán AO Time-Slotting 2D (SNR = ' num2str(rho_dB) ' dB)'], 'FontSize', 12, 'FontWeight', 'bold');
xlim([0, max(iter_vec)]);
saveas(gcf, 'Figure1_Convergence.png');

%% -------------------------------------------------------------------------
%% KHÂU 4: KHẢO SÁT HIỆU NĂNG THEO DẢI SNR (SO SÁNH 4 BENCHMARK NHƯ FOLDER AO)
%% -------------------------------------------------------------------------
fprintf('================================================================================\n');
fprintf('   KHẢO SÁT VÀ SO SÁNH HIỆU NĂNG QUA CÁC MỨC SNR (10 dB - 26 dB)                \n');
fprintf('================================================================================\n');

snr_dB_vec = 10:2:26; % Dải SNR khảo sát từ 10 dB đến 26 dB
num_snr = length(snr_dB_vec);

op_proposed    = zeros(1, num_snr);
op_fixed_power = zeros(1, num_snr);
op_fixed_traj  = zeros(1, num_snr);
op_no_opt      = zeros(1, num_snr);

for idx = 1:num_snr
    curr_snr_db = snr_dB_vec(idx);
    curr_rho = 10^(curr_snr_db / 10);
    fprintf('--> Đang tính toán tại SNR = %2d dB...\n', curr_snr_db);

    % 1. Đề xuất: Tối ưu đồng thời Quỹ đạo & Công suất (Joint AO)
    res_curr = run_timeslot_ao(curr_rho, params, Q_init, a1_init);
    op_proposed(idx) = res_curr.OP_avg_opt;

    % 2. Benchmark 1: Công suất cố định (Chỉ tối ưu quỹ đạo UAV Q*)
    [~, op_fixed_power(idx)] = solve_subproblem_trajectory(a1_init, Q_init, curr_rho, params);

    % 3. Benchmark 2: Quỹ đạo cố định (Chỉ tối ưu phân bổ công suất a1*)
    [~, op_slots_bench2] = solve_subproblem_power(Q_init, curr_rho, params);
    op_fixed_traj(idx) = mean(op_slots_bench2);

    % 4. Benchmark 3: Không tối ưu (Quỹ đạo thẳng & Công suất cố định)
    op_slots_bench3 = zeros(N_slots, 1);
    for n = 1:N_slots
        [op_slots_bench3(n), ~, ~] = calculate_outage_slot(a1_init(n), Q_init(n, :), curr_rho, params);
    end
    op_no_opt(idx) = mean(op_slots_bench3);
end
fprintf('Khảo sát dải SNR hoàn tất!\n\n');

% Lưu kết quả benchmark AO theo dải SNR để folder DRL có thể nạp độc lập
save('results_ao_benchmark.mat', 'snr_dB_vec', 'op_proposed', 'op_fixed_power', 'op_fixed_traj', 'op_no_opt', 'res_ao');
fprintf('--> Đã lưu kết quả đối chuẩn vào results_ao_benchmark.mat\n\n');

%% -------------------------------------------------------------------------
%% KHÂU 5: VẼ HÌNH 2 - ĐỒ THỊ SO SÁNH HIỆU NĂNG XÁC SUẤT DỪNG (CHUẨN FOLDER AO)
%% -------------------------------------------------------------------------
figure('Name', 'Performance Comparison vs SNR', 'Color', 'w', 'Position', [150, 150, 740, 520]);

semilogy(snr_dB_vec, op_proposed,    '-ro', 'LineWidth', 2.0, 'MarkerSize', 7, 'MarkerFaceColor', 'r'); hold on;
semilogy(snr_dB_vec, op_fixed_power, '-.d', 'LineWidth', 1.8, 'MarkerSize', 7, 'Color', [0 0.6 0], 'MarkerFaceColor', [0 0.6 0]);
semilogy(snr_dB_vec, op_fixed_traj,  '--bs', 'LineWidth', 1.8, 'MarkerSize', 7, 'MarkerFaceColor', 'b');
semilogy(snr_dB_vec, op_no_opt,      ':kx',  'LineWidth', 1.6, 'MarkerSize', 8, 'MarkerFaceColor', 'k');

grid on; grid minor;
xlabel('Tỉ số tín hiệu trên nhiễu \rho (dB)', 'FontSize', 12);
ylabel('Xác suất dừng toàn phần trung bình OP_{total}', 'FontSize', 12);
title('So sánh hiệu năng Xác suất dừng qua dải SNR (Mô hình Time-Slotting)', 'FontSize', 12, 'FontWeight', 'bold');
legend({'Đề xuất: Tối ưu đồng thời (Joint AO)', ...
        'Benchmark 1: Công suất cố định (Tối ưu quỹ đạo Q^*)', ...
        'Benchmark 2: Quỹ đạo cố định (Tối ưu công suất a^*)', ...
        'Benchmark 3: Không tối ưu (Quỹ đạo thẳng & Công suất cố định)'}, ...
        'Location', 'southwest', 'FontSize', 10);
saveas(gcf, 'Figure2_Performance_vs_SNR.png');

%% -------------------------------------------------------------------------
%% KHÂU 6: VẼ HÌNH 3 - BỐ TRÍ KHÔNG GIAN 2D (Oxz) TỐI GIẢN CHUẨN IEEE CHO TIME-SLOTTING
%% -------------------------------------------------------------------------
figure('Name', 'Spatial Deployment 2D Oxz - Time-Slotting', 'Color', 'w', 'Position', [150, 150, 820, 540]);
hold on;

% 1. Vùng bay an toàn của UAV (Màu xanh pastel nhạt, viền nét đứt thanh mảnh giống folder AO)
rectangle('Position', [uav_lb(1), uav_lb(2), uav_ub(1)-uav_lb(1), uav_ub(2)-uav_lb(2)], ...
    'LineStyle', '--', 'EdgeColor', [0.45 0.65 0.85], 'LineWidth', 1.2, ...
    'FaceColor', [0.92, 0.96, 1.0]);

h_region = patch(NaN, NaN, [0.92, 0.96, 1.0], ...
    'EdgeColor', [0.45 0.65 0.85], 'LineStyle', '--', 'LineWidth', 1.2);

% 2. Ranh giới hình học SIC (Đường trung trực x = 1.25 trừ margin)
x_sic_bound = 0.5 * (pos_D1(1) + pos_D2(1)) - SIC_margin;
h_sic = plot([x_sic_bound, x_sic_bound], [uav_lb(2), uav_ub(2)], ...
    'r--', 'LineWidth', 1.8);
text(x_sic_bound - 0.02, uav_ub(2) + 0.04, sprintf('SIC Boundary (x \\leq %.2f)', x_sic_bound), ...
    'Color', 'r', 'FontSize', 9.5, 'FontWeight', 'bold', 'HorizontalAlignment', 'right');

% 3. Đường kẻ Mặt đất (Ground Line z = 0)
h_ground = plot([-0.18, 1.88], [0, 0], 'k-', 'LineWidth', 1.8);
text(-0.16, 0.04, 'Ground (z = 0)', 'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'bottom', 'FontSize', 9.5, 'FontAngle', 'italic');

% 4. Vệ tinh S (Dấu sao đen)
h_sat = plot(pos_S(1), pos_S(2), 'k*', 'MarkerSize', 11, 'LineWidth', 1.8);
text(pos_S(1) + 0.04, pos_S(2), 'S (0, 1.0)', ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', ...
    'FontSize', 10, 'FontWeight', 'bold');

% 5. Hai người dùng mặt đất D1, D2 (Chấm đen)
h_user = plot([pos_D2(1), pos_D1(1)], [pos_D2(2), pos_D1(2)], ...
    'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'k');
text(pos_D2(1), -0.05, 'D_2 (1.0, 0)', 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'top', 'FontSize', 9.5, 'FontWeight', 'bold');
text(pos_D1(1), -0.05, 'D_1 (1.5, 0)', 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'top', 'FontSize', 9.5, 'FontWeight', 'bold');

% 6. Quỹ đạo ban đầu: Đường thẳng nét đứt màu xám
h_traj_init = plot(Q_init(:, 1), Q_init(:, 2), 'k:d', ...
    'LineWidth', 1.2, 'MarkerSize', 5, 'MarkerFaceColor', 'w');

% 7. Quỹ đạo tối ưu Time-Slotting: Đường bay liền màu xanh đậm với các điểm mốc khe n
h_traj_opt = plot(res_ao.Q_opt(:, 1), res_ao.Q_opt(:, 2), '-o', ...
    'Color', [0.0, 0.45, 0.74], 'LineWidth', 2.0, ...
    'MarkerSize', 6, 'MarkerFaceColor', [0.0, 0.45, 0.74]);

% Vẽ mũi tên hướng chuyển động giữa các khe bằng quiver
for n = 1:(N_slots - 1)
    p1 = res_ao.Q_opt(n, :);
    p2 = res_ao.Q_opt(n + 1, :);
    v = p2 - p1;
    quiver(p1(1), p1(2), v(1), v(2), 0, 'Color', [0.0, 0.45, 0.74], ...
           'LineWidth', 1.5, 'MaxHeadSize', 0.4);
end

% Nhãn các mốc thời gian khe n
for n = 1:2:N_slots
    text(res_ao.Q_opt(n, 1) + 0.02, res_ao.Q_opt(n, 2) + 0.03, sprintf('n=%d', n), ...
        'FontSize', 8.5, 'FontWeight', 'bold', 'Color', [0.0, 0.45, 0.74]);
end

% Đánh dấu điểm bắt đầu q_start và điểm kết thúc q_end
plot(q_start(1), q_start(2), 'kp', 'MarkerSize', 11, 'MarkerFaceColor', 'c');
text(q_start(1) - 0.03, q_start(2) + 0.04, 'q_{start}', 'FontSize', 9.5, 'FontWeight', 'bold');

plot(q_end(1), q_end(2), 'kp', 'MarkerSize', 11, 'MarkerFaceColor', 'm');
text(q_end(1) + 0.03, q_end(2) - 0.03, 'q_{end}', 'FontSize', 9.5, 'FontWeight', 'bold');

% 8. Tinh chỉnh khung nhìn, lưới nét chấm mờ
grid on;
set(gca, 'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.7, 'GridLineStyle', ':');
box on;
xlim([-0.20, 1.95]);
ylim([-0.15, 1.18]);
xlabel('Tọa độ ngang chuẩn hóa x', 'FontSize', 11);
ylabel('Độ cao chuẩn hóa z', 'FontSize', 11);
title('Mô hình bố trí không gian 2D (Oxz) & Quỹ đạo bay UAV Time-Slotting', 'FontSize', 12, 'FontWeight', 'bold');

legend([h_sat, h_traj_init, h_traj_opt, h_user, h_sic, h_region, h_ground], ...
    {'Vệ tinh S', 'Quỹ đạo thẳng Q^{(0)}', 'Quỹ đạo tối ưu Q^* (AO)', ...
     'Người dùng D_1, D_2', 'Ranh giới SIC (D_2 Near)', 'Vùng bay UAV', 'Mặt đất'}, ...
    'Location', 'northeast', 'FontSize', 8.5);

saveas(gcf, 'Figure3_Spatial_Trajectory_2D_Oxz.png');

%% -------------------------------------------------------------------------
%% KHÂU 7: VẼ HÌNH 4 - CHI TIẾT PHÂN BỔ CÔNG SUẤT VÀ XÁC SUẤT DỪNG QUA TỪNG KHE
%% -------------------------------------------------------------------------
figure('Name', 'TimeSlot Power and Outage Details', 'Color', 'w', 'Position', [100, 100, 950, 420]);
slots_vec = 1:N_slots;

% Subplot 1: Phân bổ công suất NOMA qua từng khe thời gian
subplot(1, 2, 1);
hold on; box on; grid on;
p_bar = bar(slots_vec, [res_ao.a1_opt, res_ao.a2_opt], 'stacked');
p_bar(1).FaceColor = [0.85, 0.33, 0.10]; % Màu cam đỏ cho a1 (Far-user)
p_bar(2).FaceColor = [0.00, 0.45, 0.74]; % Màu xanh dương cho a2 (Near-user)
a1_min = params.gamma_th1 / (1 + params.gamma_th1);
yline(a1_min, 'k--', sprintf('Ngưỡng SIC \\alpha_{1,min} = %.3f', a1_min), ...
      'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left', 'FontSize', 8.5);
xlabel('Chỉ số khe thời gian n', 'FontSize', 11);
ylabel('Hệ số phân bổ công suất', 'FontSize', 11);
title('(a) Phân bổ công suất NOMA (\alpha_1[n], \alpha_2[n])', 'FontSize', 11, 'FontWeight', 'bold');
ylim([0, 1.15]);
legend({'\alpha_1[n] (D_1 Far)', '\alpha_2[n] (D_2 Near)'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8.5);

% Subplot 2: Xác suất dừng của D1, D2 và hệ thống qua từng khe
subplot(1, 2, 2);
hold on; box on; grid on;
h1 = plot(slots_vec, res_ao.OP_D1_slots, '-s', 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.8, ...
     'MarkerSize', 6, 'MarkerFaceColor', [0.85, 0.33, 0.10], 'DisplayName', 'OP D_1 (Far)');
h2 = plot(slots_vec, res_ao.OP_D2_slots, '-^', 'Color', [0.47, 0.67, 0.19], 'LineWidth', 1.8, ...
     'MarkerSize', 6, 'MarkerFaceColor', [0.47, 0.67, 0.19], 'DisplayName', 'OP D_2 (Near)');
h3 = plot(slots_vec, res_ao.OP_slots_opt, '-o', 'Color', [0.49, 0.18, 0.56], 'LineWidth', 2.0, ...
     'MarkerSize', 6, 'MarkerFaceColor', [0.49, 0.18, 0.56], 'DisplayName', 'OP Toàn phần');
yline(res_ao.OP_avg_opt, 'b--', sprintf('OP Trung bình = %.4e', res_ao.OP_avg_opt), ...
      'LineWidth', 1.5, 'FontSize', 8.5, 'HandleVisibility', 'off');
set(gca, 'YScale', 'log');
xlabel('Chỉ số khe thời gian n', 'FontSize', 11);
ylabel('Xác suất dừng (Log-scale)', 'FontSize', 11);
title('(b) Xác suất dừng qua các khe thời gian', 'FontSize', 11, 'FontWeight', 'bold');
ylim([1e-6, 1.2]);
legend([h1, h2, h3], {'OP D_1 (Far)', 'OP D_2 (Near)', 'OP Toàn phần'}, 'Location', 'southwest', 'FontSize', 8.5);

saveas(gcf, 'Figure4_TimeSlot_Power_Outage.png');

fprintf('================================================================================\n');
fprintf('   HOÀN THÀNH TOÀN BỘ MÔ PHỎNG VÀ XUẤT 4 ĐỒ THỊ CHUẨN IEEE TẠI THƯ MỤC!         \n');
fprintf('   1. Figure1_Convergence.png\n');
fprintf('   2. Figure2_Performance_vs_SNR.png\n');
fprintf('   3. Figure3_Spatial_Trajectory_2D_Oxz.png\n');
fprintf('   4. Figure4_TimeSlot_Power_Outage.png\n');
fprintf('================================================================================\n');
