%% =========================================================================
%% ĐỀ TÀI NCKH: ĐÁNH GIÁ HIỆU NĂNG VÀ TỐI ƯU HÓA MẠNG SAGIN-NOMA DÙNG UAV
%% SCRIPT CHÍNH: main.m (MÔ HÌNH 2D Oxz - CHUẨN HÓA KHÔNG THỨ NGUYÊN)
%% =========================================================================

if ~exist('KEEP_WORKSPACE', 'var')
    clear;
end
clc; close all;

%% -------------------------------------------------------------------------
%% KHÂU 1: THIẾT LẬP THAM SỐ HỆ THỐNG 2D (Oxz) CHUẨN HÓA
%% -------------------------------------------------------------------------
fprintf('=====================================================\n');
fprintf('   KHỞI TẠO THAM SỐ MÔ HÌNH 2D (Oxz) CHUẨN HÓA       \n');
fprintf('=====================================================\n');

% 1. Tọa độ chuẩn hóa không thứ nguyên [x, z] trong mặt phẳng (Oxz)
pos_S  = [0.0, 1.0];           % Vệ tinh nguồn S tại đỉnh z = 1.0
pos_D2 = [1.0, 0.0];           % Người dùng gần D2 tại x = 1.0
pos_D1 = [1.5, 0.0];           % Người dùng xa D1 tại x = 1.5

% Hành lang bay an toàn của UAV trong mặt phẳng (Oxz): [x, z]
uav_lb = [0.2, 0.2];           % [Xmin, Hmin]
uav_ub = [1.8, 0.6];           % [Xmax, Hmax]

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

% 4. Tỉ số tín hiệu trên nhiễu trung bình rho tại điểm thiết kế (dB)
rho_dB = 20;                   % Mức SNR khảo sát hội tụ (20 dB)
rho = 10^(rho_dB / 10);

% 5. Cấu hình tiêu chuẩn hội tụ thuật toán AO (Công thức 26 & 27)
max_iterations = 25;           % Số vòng lặp tối đa
epsilon = 1e-4;                % Ngưỡng dừng hàm mục tiêu |F(k+1) - F(k)|
delta_q = 1e-3;                % Ngưỡng dừng khoảng cách UAV ||q(k+1) - q(k)||
delta_a = 1e-3;                % Ngưỡng dừng sai khác công suất |alpha1(k+1) - alpha1(k)|

% Điểm mốc tham chiếu cho các trường hợp Benchmark
fixed_q  = [0.5, 0.4];         % Vị trí khởi tạo của UAV
fixed_a1 = 0.75;               % Phân bổ công suất cố định (thỏa mãn SIC)

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
params.max_iterations = max_iterations;
params.epsilon        = epsilon;
params.delta_q        = delta_q;
params.delta_a        = delta_a;

fprintf('Tham số thiết lập thành công!\n');
fprintf('Ngưỡng SIC tối thiểu của alpha1: %.4f (Khởi tạo alpha1 = %.2f)\n\n', ...
    gamma_th1 / (1 + gamma_th1), fixed_a1);

%% -------------------------------------------------------------------------
%% KHÂU 2: CHẠY THỬ NGHIỆM THUẬT TOÁN AO TẠI MỨC SNR THIẾT KẾ
%% -------------------------------------------------------------------------
fprintf('=====================================================\n');
fprintf('   CHẠY THỬ NGHIỆM THUẬT TOÁN AO 2D (SNR = %d dB)    \n', rho_dB);
fprintf('=====================================================\n');

res_ao = run_alternating_optimization(rho, params, fixed_q, fixed_a1);

fprintf('\n------------------ KẾT QUẢ TỐI ƯU ------------------\n');
fprintf('Hệ số công suất tối ưu  : alpha1* = %.4f | alpha2* = %.4f\n', ...
    res_ao.optimal_alpha1, res_ao.optimal_alpha2);
fprintf('Tọa độ UAV tối ưu qU*   : x* = %.4f, z* = %.4f\n', ...
    res_ao.optimal_q(1), res_ao.optimal_q(2));
fprintf('Xác suất dừng tối ưu    : OP_total* = %.6e\n', res_ao.optimal_OP);
fprintf('Số vòng lặp thực hiện   : %d vòng\n', length(res_ao.history_F) - 1);
fprintf('----------------------------------------------------\n\n');

%% -------------------------------------------------------------------------
%% KHÂU 3: VẼ HÌNH 1 - ĐỒ THỊ ĐƯỜNG CONG HỘI TỤ
%% -------------------------------------------------------------------------
figure('Name', 'Convergence Curve of AO 2D', 'Color', 'w', 'Position', [100, 100, 680, 500]);
iter_vec = 0:(length(res_ao.history_F) - 1);

semilogy(iter_vec, res_ao.history_F, '-o', ...
    'LineWidth', 2, 'Color', [0 0.4470 0.7410], ...
    'MarkerSize', 7, 'MarkerFaceColor', [0 0.4470 0.7410]);

grid on; grid minor;
xlabel('Số vòng lặp (Iteration k)', 'FontSize', 12);
ylabel('Xác suất dừng toàn phần OP_{total}', 'FontSize', 12);
title(['Đường cong hội tụ của Thuật toán AO 2D (SNR = ' num2str(rho_dB) ' dB)'], 'FontSize', 13);
xlim([0, max(iter_vec)]);
saveas(gcf, 'Figure1_Convergence.png');
fprintf('--> Đã lưu Figure1_Convergence.png\n\n');

%% -------------------------------------------------------------------------
%% KHÂU 4: KHẢO SÁT HIỆU NĂNG THEO DẢI SNR (SO SÁNH CÁC BENCHMARK)
%% -------------------------------------------------------------------------
fprintf('=====================================================\n');
fprintf('   KHẢO SÁT VÀ SO SÁNH HIỆU NĂNG QUA CÁC MỨC SNR    \n');
fprintf('=====================================================\n');

snr_dB_vec = 10:2:26; % Dải SNR khảo sát từ 10 dB đến 26 dB
num_snr = length(snr_dB_vec);

op_proposed    = zeros(1, num_snr);
op_fixed_pos   = zeros(1, num_snr);
op_fixed_power = zeros(1, num_snr);
op_no_opt      = zeros(1, num_snr);

% Lưu trữ hệ số phân bổ công suất và OP thành phần qua dải SNR
alpha1_opt_vec = zeros(num_snr, 1);
alpha2_opt_vec = zeros(num_snr, 1);
op_d1_opt_vec  = zeros(num_snr, 1);
op_d2_opt_vec  = zeros(num_snr, 1);

for idx = 1:num_snr
    curr_snr_db = snr_dB_vec(idx);
    curr_rho = 10^(curr_snr_db / 10);
    fprintf('--> Đang tính toán tại SNR = %2d dB...\n', curr_snr_db);

    % 1. Thuật toán đề xuất: Tối ưu đồng thời Joint AO
    res_curr = run_alternating_optimization(curr_rho, params, fixed_q, fixed_a1);
    op_proposed(idx)    = res_curr.optimal_OP;
    alpha1_opt_vec(idx) = res_curr.optimal_alpha1;
    alpha2_opt_vec(idx) = res_curr.optimal_alpha2;
    [~, op_d1_opt_vec(idx), op_d2_opt_vec(idx)] = calculate_outage_probability(res_curr.optimal_alpha1, res_curr.optimal_q, curr_rho, params);

    % 2. Benchmark 1: Vị trí UAV cố định (chỉ tối ưu phân bổ công suất)
    [~, op_fixed_pos(idx)] = solve_subproblem_power(fixed_q, curr_rho, params);

    % 3. Benchmark 2: Phân bổ công suất cố định (chỉ tối ưu vị trí UAV)
    [~, op_fixed_power(idx)] = solve_subproblem_position(fixed_a1, fixed_q, curr_rho, params);

    % 4. Benchmark 3: Không tối ưu (cố định cả vị trí và công suất)
    op_no_opt(idx) = calculate_outage_probability(fixed_a1, fixed_q, curr_rho, params);

    % 5. Kiểm chứng Monte-Carlo cho giải pháp Đề xuất Joint AO (10^5 mẫu thử)
    N_mc = 100000;
    s_su = sqrt(params.K_SU / (params.K_SU + 1));
    sig_su = sqrt(1 / (2 * (params.K_SU + 1)));
    s_ud = sqrt(params.K_UD / (params.K_UD + 1));
    sig_ud = sqrt(1 / (2 * (params.K_UD + 1)));

    [lambda_SU, lambda_UD1, lambda_UD2] = calculate_channel_params(res_curr.optimal_q, params);

    % Chặng 1 PRS qua N_UAV
    h_su = (s_su + sig_su*randn(params.N_UAV, N_mc)) + 1i*(sig_su*randn(params.N_UAV, N_mc));
    gain_su = max(abs(h_su).^2, [], 1) * lambda_SU;

    denom = curr_rho * (res_curr.optimal_alpha1 - res_curr.optimal_alpha2 * params.gamma_th1);
    Theta1 = params.gamma_th1 / denom;
    Theta2 = max(Theta1, params.gamma_th2 / (curr_rho * res_curr.optimal_alpha2));

    succ_hop1_D1 = (gain_su >= Theta1);
    succ_hop1_D2 = (gain_su >= Theta2);

    % Chặng 2 NOMA
    h_d1 = (s_ud + sig_ud*randn(1, N_mc)) + 1i*(sig_ud*randn(1, N_mc));
    h_d2 = (s_ud + sig_ud*randn(1, N_mc)) + 1i*(sig_ud*randn(1, N_mc));
    gain_d1 = abs(h_d1).^2 * lambda_UD1;
    gain_d2 = abs(h_d2).^2 * lambda_UD2;

    succ_hop2_D1 = (gain_d1 >= Theta1);
    succ_hop2_D2 = (gain_d2 >= Theta2);

    op_d1_mc = 1 - mean(succ_hop1_D1 & succ_hop2_D1);
    op_d2_mc = 1 - mean(succ_hop1_D2 & succ_hop2_D2);
    op_mc_proposed(idx) = 1 - (1 - op_d1_mc) * (1 - op_d2_mc);
end
fprintf('Tính toán hoàn tất!\n\n');

%% -------------------------------------------------------------------------
%% KHÂU 5: VẼ HÌNH 2 - ĐỒ THỊ SO SÁNH HIỆU NĂNG XÁC SUẤT DỪNG & KIỂM CHỨNG MONTE-CARLO
%% -------------------------------------------------------------------------
figure('Name', 'Performance Comparison vs SNR', 'Color', 'w', 'Position', [150, 150, 740, 520]);

semilogy(snr_dB_vec, op_proposed,    '-r',   'LineWidth', 2.2); hold on;
semilogy(snr_dB_vec, op_mc_proposed, 'ko',   'LineWidth', 1.4, 'MarkerSize', 6);
semilogy(snr_dB_vec, op_fixed_power, '-.d',  'LineWidth', 1.8, 'MarkerSize', 7, 'Color', [0 0.6 0], 'MarkerFaceColor', [0 0.6 0]);
semilogy(snr_dB_vec, op_fixed_pos,   '--bs', 'LineWidth', 1.8, 'MarkerSize', 7, 'MarkerFaceColor', 'b');
semilogy(snr_dB_vec, op_no_opt,      ':mx',  'LineWidth', 1.6, 'MarkerSize', 8, 'MarkerFaceColor', 'm');

grid on; grid minor;
xlabel('Tỉ số tín hiệu trên nhiễu \rho (dB)', 'FontSize', 12);
ylabel('Xác suất dừng toàn phần OP_{total}', 'FontSize', 12);
title('Đối chuẩn hiệu năng Xác suất dừng & Kiểm chứng Monte-Carlo', 'FontSize', 13);
legend({'Giải tích đề xuất (Joint AO)', ...
        'Mô phỏng Monte-Carlo (10^5 mẫu)', ...
        'Benchmark 1: Công suất cố định (Tối ưu vị trí)', ...
        'Benchmark 2: Vị trí UAV cố định (Tối ưu công suất)', ...
        'Benchmark 3: Không tối ưu (Vị trí & Công suất cố định)'}, ...
        'Location', 'southwest', 'FontSize', 9.5);
saveas(gcf, 'Figure2_Performance_vs_SNR.png');
fprintf('--> Đã lưu Figure2_Performance_vs_SNR.png\n\n');

%% -------------------------------------------------------------------------
%% KHÂU 6: VẼ HÌNH 3 - BỐ TRÍ KHÔNG GIAN 2D (Oxz) TỐI GIẢN CHUẨN IEEE
%% -------------------------------------------------------------------------
figure('Name', 'Spatial Deployment 2D Oxz', 'Color', 'w', 'Position', [150, 150, 780, 490]);
hold on;

% 1. Vùng bay an toàn của UAV (Màu xanh pastel nhạt, viền nét đứt thanh mảnh)
rectangle('Position', [uav_lb(1), uav_lb(2), uav_ub(1)-uav_lb(1), uav_ub(2)-uav_lb(2)], ...
    'LineStyle', '--', 'EdgeColor', [0.45 0.65 0.85], 'LineWidth', 1.2, ...
    'FaceColor', [0.92, 0.96, 1.0]);

h_region = patch(NaN, NaN, [0.92, 0.96, 1.0], ...
    'EdgeColor', [0.45 0.65 0.85], 'LineStyle', '--', 'LineWidth', 1.2);

% 2. Đường kẻ Mặt đất (Ground Line z = 0)
h_ground = plot([-0.18, 1.88], [0, 0], 'k-', 'LineWidth', 1.8);
text(-0.16, 0.04, 'Ground (z = 0)', 'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'bottom', 'FontSize', 9.5, 'FontAngle', 'italic');

% 3. Vệ tinh S (Dấu sao đen) - Nhãn lệch phải, không bị đè
h_sat = plot(pos_S(1), pos_S(2), 'k*', 'MarkerSize', 11, 'LineWidth', 1.8);
text(pos_S(1) + 0.04, pos_S(2), 'S (0, 1.0)', ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', ...
    'FontSize', 10, 'FontWeight', 'bold');

% 4. Hai người dùng mặt đất D1, D2 (Chấm đen) - Hiện đầy đủ tọa độ bên dưới đường kẻ
h_user = plot([pos_D2(1), pos_D1(1)], [pos_D2(2), pos_D1(2)], ...
    'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'k');
text(pos_D2(1), -0.04, 'D_2 (1.0, 0)', 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'top', 'FontSize', 9.5, 'FontWeight', 'bold');
text(pos_D1(1), -0.04, 'D_1 (1.5, 0)', 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'top', 'FontSize', 9.5, 'FontWeight', 'bold');

% 5. Mũi tên dịch chuyển từ U^(0) đến U^* (Dừng vừa chạm mép, không đè lên marker/chữ)
v_dir = res_ao.optimal_q - fixed_q;
v_unit = v_dir / norm(v_dir);
p_start = fixed_q + 0.03 * v_unit;
p_vec = v_dir - 0.06 * v_unit;
quiver(p_start(1), p_start(2), p_vec(1), p_vec(2), 0, 'k--', ...
       'LineWidth', 1.2, 'MaxHeadSize', 0.22);

% 6. UAV ban đầu: Tam giác viền đen (Nhãn đặt thẳng đứng PHÍA TRÊN, cách xa đỉnh tam giác)
h_uav_init = plot(fixed_q(1), fixed_q(2), 'k^', 'MarkerSize', 8, 'LineWidth', 1.5, 'MarkerFaceColor', 'w');
text(fixed_q(1), fixed_q(2) + 0.06, 'U^{(0)} (0.5, 0.4)', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
    'FontSize', 10, 'FontWeight', 'bold');

% 7. UAV tối ưu: Tam giác đen tô đặc (Nhãn đặt PHÍA BÊN PHẢI, hoàn toàn trống trải)
h_uav_opt = plot(res_ao.optimal_q(1), res_ao.optimal_q(2), 'k^', ...
    'MarkerSize', 9, 'MarkerFaceColor', 'k');
text(res_ao.optimal_q(1) + 0.04, res_ao.optimal_q(2) + 0.02, ...
    sprintf('U^* (%.2f, %.2f)', res_ao.optimal_q(1), res_ao.optimal_q(2)), ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', ...
    'FontSize', 10, 'FontWeight', 'bold');

% 8. Tinh chỉnh khung nhìn, lưới nét chấm mờ
grid on;
set(gca, 'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.7, 'GridLineStyle', ':');
box on;
xlim([-0.20, 1.95]);
ylim([-0.15, 1.18]);
xlabel('Tọa độ ngang chuẩn hóa x', 'FontSize', 11);
ylabel('Độ cao chuẩn hóa z', 'FontSize', 11);
title('Mô hình bố trí không gian 2D (Oxz) của mạng SAGIN', 'FontSize', 12, 'FontWeight', 'bold');

% Legend đầy đủ, rõ ràng và tách biệt
legend([h_sat, h_uav_init, h_uav_opt, h_user, h_region, h_ground], ...
    {'Vệ tinh S', 'UAV ban đầu U^{(0)}', 'UAV tối ưu U^* (AO)', 'Người dùng D_1, D_2', 'Vùng bay UAV', 'Mặt đất'}, ...
    'Location', 'northeast', 'FontSize', 9);

saveas(gcf, 'Figure3_Spatial_Trajectory_2D_Oxz.png');
fprintf('--> Đã lưu Figure3_Spatial_Trajectory_2D_Oxz.png\n\n');

%% -------------------------------------------------------------------------
%% KHÂU 7: VẼ HÌNH 4 - CHI TIẾT PHÂN BỔ CÔNG SUẤT VÀ XÁC SUẤT DỪNG (CHUẨN TIME_SLOT & DRL)
%% -------------------------------------------------------------------------
figure('Name', 'AO Power and Outage Details', 'Color', 'w', 'Position', [100, 100, 950, 420]);

% Subplot 1: Phân bổ công suất NOMA qua dải SNR (Stacked Bar)
subplot(1, 2, 1);
hold on; box on; grid on;
p_bar = bar(snr_dB_vec, [alpha1_opt_vec, alpha2_opt_vec], 'stacked');
p_bar(1).FaceColor = [0.85, 0.33, 0.10]; % Màu cam đỏ cho \alpha_1 (Far-user D1)
p_bar(2).FaceColor = [0.00, 0.45, 0.74]; % Màu xanh dương cho \alpha_2 (Near-user D2)
a1_min = params.gamma_th1 / (1 + params.gamma_th1);
yline(a1_min, 'k--', sprintf('Ngưỡng SIC \\alpha_{1,min} = %.3f', a1_min), ...
      'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left', 'FontSize', 8.5);
xlabel('Tỉ số tín hiệu trên nhiễu \rho (dB)', 'FontSize', 11);
ylabel('Hệ số phân bổ công suất', 'FontSize', 11);
title('(a) Phân bổ công suất NOMA (\alpha_1^*, \alpha_2^*) theo SNR', 'FontSize', 11, 'FontWeight', 'bold');
ylim([0, 1.15]);
legend({'\alpha_1^* (D_1 Far)', '\alpha_2^* (D_2 Near)'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8.5);

% Subplot 2: Xác suất dừng chi tiết của D1, D2 và hệ thống qua dải SNR
subplot(1, 2, 2);
hold on; box on; grid on;
plot(snr_dB_vec, op_d1_opt_vec, '-s', 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.8, ...
     'MarkerSize', 6, 'MarkerFaceColor', [0.85, 0.33, 0.10], 'DisplayName', 'OP D_1 (Far)');
plot(snr_dB_vec, op_d2_opt_vec, '-^', 'Color', [0.47, 0.67, 0.19], 'LineWidth', 1.8, ...
     'MarkerSize', 6, 'MarkerFaceColor', [0.47, 0.67, 0.19], 'DisplayName', 'OP D_2 (Near)');
plot(snr_dB_vec, op_proposed, '-o', 'Color', [0.49, 0.18, 0.56], 'LineWidth', 2.0, ...
     'MarkerSize', 6, 'MarkerFaceColor', [0.49, 0.18, 0.56], 'DisplayName', 'OP Toàn phần');
set(gca, 'YScale', 'log');
xlabel('Tỉ số tín hiệu trên nhiễu \rho (dB)', 'FontSize', 11);
ylabel('Xác suất dừng (Log-scale)', 'FontSize', 11);
title('(b) Xác suất dừng chi tiết qua dải SNR', 'FontSize', 11, 'FontWeight', 'bold');
ylim([1e-4, 1.2]);
legend('Location', 'southwest', 'FontSize', 8.5);

saveas(gcf, 'Figure4_TimeSlot_Power_Outage.png');
saveas(gcf, 'Figure4_Power_Allocation_Outage.png');
fprintf('--> Đã lưu Figure4_TimeSlot_Power_Outage.png & Figure4_Power_Allocation_Outage.png\n');

%% Hình 4b: Tiến trình phân bổ công suất và xác suất dừng qua các vòng lặp hội tụ (SNR = 20 dB)
figure('Name', 'AO Iteration Power and Outage Details', 'Color', 'w', 'Position', [100, 100, 950, 420]);
iter_vec = 0:(length(res_ao.history_F) - 1);
alpha1_iter = res_ao.history_alpha(:);
alpha2_iter = 1.0 - alpha1_iter;

subplot(1, 2, 1);
hold on; box on; grid on;
p_bar_it = bar(iter_vec, [alpha1_iter, alpha2_iter], 'stacked');
p_bar_it(1).FaceColor = [0.85, 0.33, 0.10];
p_bar_it(2).FaceColor = [0.00, 0.45, 0.74];
yline(a1_min, 'k--', sprintf('Ngưỡng SIC \\alpha_{1,min} = %.3f', a1_min), ...
      'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left', 'FontSize', 8.5);
xlabel('Số vòng lặp (Iteration k)', 'FontSize', 11);
ylabel('Hệ số phân bổ công suất', 'FontSize', 11);
title('(a) Phân bổ công suất NOMA (\alpha_1^{(k)}, \alpha_2^{(k)}) qua các vòng lặp', 'FontSize', 11, 'FontWeight', 'bold');
ylim([0, 1.15]);
legend({'\alpha_1^{(k)} (D_1 Far)', '\alpha_2^{(k)} (D_2 Near)'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8.5);

subplot(1, 2, 2);
hold on; box on; grid on;
plot(iter_vec, res_ao.history_OP_D1, '-s', 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.8, ...
     'MarkerSize', 6, 'MarkerFaceColor', [0.85, 0.33, 0.10], 'DisplayName', 'OP D_1 (Far)');
plot(iter_vec, res_ao.history_OP_D2, '-^', 'Color', [0.47, 0.67, 0.19], 'LineWidth', 1.8, ...
     'MarkerSize', 6, 'MarkerFaceColor', [0.47, 0.67, 0.19], 'DisplayName', 'OP D_2 (Near)');
plot(iter_vec, res_ao.history_F, '-o', 'Color', [0.49, 0.18, 0.56], 'LineWidth', 2.0, ...
     'MarkerSize', 6, 'MarkerFaceColor', [0.49, 0.18, 0.56], 'DisplayName', 'OP Toàn phần');
set(gca, 'YScale', 'log');
xlabel('Số vòng lặp (Iteration k)', 'FontSize', 11);
ylabel('Xác suất dừng (Log-scale)', 'FontSize', 11);
title(['(b) Xác suất dừng qua các vòng lặp (SNR = ' num2str(rho_dB) ' dB)'], 'FontSize', 11, 'FontWeight', 'bold');
ylim([1e-4, 1.2]);
legend('Location', 'northeast', 'FontSize', 8.5);

saveas(gcf, 'Figure4_Power_Convergence.png');
fprintf('--> Đã lưu Figure4_Power_Convergence.png\n');

%% -------------------------------------------------------------------------
%% KHÂU 8: XUẤT ĐỒ THỊ TỔNG HỢP 4 TRONG 1 (CHUẨN FOLDER TIME_SLOT & DRL)
%% -------------------------------------------------------------------------
plot_results(res_ao, snr_dB_vec, alpha1_opt_vec, alpha2_opt_vec, op_d1_opt_vec, op_d2_opt_vec, op_proposed, fixed_q, params, 'fig_AO_Optimization_Results.png');

fprintf('================================================================================\n');
fprintf('   HOÀN THÀNH TOÀN BỘ MÔ PHỎNG AO 2D VÀ XUẤT ĐỒ THỊ CHUẨN IEEE TẠI THƯ MỤC!     \n');
fprintf('   1. Figure1_Convergence.png\n');
fprintf('   2. Figure2_Performance_vs_SNR.png\n');
fprintf('   3. Figure3_Spatial_Trajectory_2D_Oxz.png\n');
fprintf('   4. Figure4_TimeSlot_Power_Outage.png (Figure4_Power_Allocation_Outage.png)\n');
fprintf('   5. Figure4_Power_Convergence.png\n');
fprintf('   6. fig_AO_Optimization_Results.png\n');
fprintf('================================================================================\n');
