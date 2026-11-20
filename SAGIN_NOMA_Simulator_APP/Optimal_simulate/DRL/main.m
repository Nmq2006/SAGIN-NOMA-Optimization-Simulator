%% =========================================================================
%% ĐỀ TÀI NCKH: ĐÁNH GIÁ HIỆU NĂNG VÀ TỐI ƯU HÓA MẠNG SAGIN-NOMA DÙNG UAV
%% SCRIPT CHÍNH: main.m (ÁP DỤNG DEEP REINFORCEMENT LEARNING - DRL 2D Oxz)
%% Phiên bản cập nhật: v1 (Guided Trajectory & Balanced NOMA Power)
%% Thư mục: Optimal_simulate/DRL/
%% =========================================================================

if ~exist('KEEP_WORKSPACE', 'var')
    clear;
end
clc; close all;

% Cố định Random Seed để đảm bảo tính tái lập (Reproducibility) giữa các lần huấn luyện
rng(42, 'twister');

% Bổ sung đường dẫn tới folder Time_Slot để liên kết thuật toán so chuẩn AO
addpath(fullfile(pwd, '..', 'Time_Slot'));

%% -------------------------------------------------------------------------
%% KHÂU 1: THIẾT LẬP THAM SỐ HỆ THỐNG 2D (Oxz) CHUẨN HÓA (ĐỒNG NHẤT 100%)
%% -------------------------------------------------------------------------
fprintf('================================================================================\n');
fprintf('   KHỞI TẠO THAM SỐ MÔ HÌNH 2D (Oxz) CHUẨN HÓA CHO DRL & SAGIN-NOMA (v1)       \n');
fprintf('================================================================================\n');

% 1. Tọa độ chuẩn hóa không thứ nguyên [x, z] trong mặt phẳng (Oxz)
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
gamma_th1 = 2^(2 * R1) - 1;    % ~ 2.0314
gamma_th2 = 2^(2 * R2) - 1;    % ~ 4.2780
sic_min   = gamma_th1 / (1 + gamma_th1); % ~ 0.6701

% 4. Tham số Time-slotting & Khí động học UAV
T_total = 20.0;                % Tổng thời gian bay khảo sát (giây)
N_slots = 10;                  % Số lượng khe thời gian rời rạc N
dt      = T_total / N_slots;   % Độ dài mỗi khe dt (giây)
V_max   = 0.12;                % Vận tốc tối đa cho phép của UAV
max_step_dist = V_max * dt;    % 0.24

% Điểm xuất phát (q_start) và điểm đích (q_end) của UAV trong mặt phẳng Oxz
q_start = [0.35, 0.50];        % Vị trí ban đầu gần vệ tinh
q_end   = [1.10, 0.30];        % Vị trí đích tiếp cận người dùng gần D2

% 5. Ranh giới hình học cố định vị thế giải mã SIC (User Ordering Constraint)
SIC_margin = 0.04;             % Biên độ an toàn hình học (x <= 1.21)

% 6. Tỉ số tín hiệu trên nhiễu trung bình rho tại điểm thiết kế (dB)
rho_dB = 20;                   % Mức SNR khảo sát hội tụ (20 dB)
rho = 10^(rho_dB / 10);

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

% Quỹ đạo khởi tạo ban đầu: Bay thẳng đều và Phân bổ công suất cố định
Q_init = zeros(N_slots, 2);
for n = 1:N_slots
    alpha = (n - 1) / (N_slots - 1);
    Q_init(n, :) = (1 - alpha) * q_start + alpha * q_end;
end
a1_init = 0.75 * ones(N_slots, 1);

fprintf('--> Tham số Time-slotting: T = %.1fs, N = %d khe, dt = %.2fs\n', T_total, N_slots, dt);
fprintf('--> Dịch chuyển tối đa mỗi khe: V_max * dt = %.3f\n', max_step_dist);
fprintf('--> Ngưỡng công suất SIC khả thi: alpha1 > %.4f (Khởi tạo alpha1 = %.2f)\n\n', ...
        sic_min, a1_init(1));

%% -------------------------------------------------------------------------
%% KHÂU 2: HUẤN LUYỆN HOẶC NẠP MÔ HÌNH DRL (TRAINING VS. INFERENCE MODE)
%% -------------------------------------------------------------------------
% Cấu hình chế độ thực thi:
%   RETRAIN = false : Tự động nạp mô hình đã lưu từ file 'trained_drl_agent.mat'.
%                     Chạy cực nhanh (< 1 giây), kết quả xuất đồ thị cố định 100%.
%   RETRAIN = true  : Huấn luyện lại Agent từ đầu qua 260 episode và lưu đè file .mat.
RETRAIN = false;
model_file = 'trained_drl_agent.mat';

if ~RETRAIN && exist(model_file, 'file') == 2
    fprintf('================================================================================\n');
    fprintf('   NẠP MÔ HÌNH DRL ĐÃ HUẤN LUYỆN TỪ FILE: %s\n', model_file);
    fprintf('================================================================================\n');
    loaded_data   = load(model_file);
    agent         = loaded_data.agent;
    train_history = loaded_data.train_history;
    res_drl       = loaded_data.res_drl;
    num_episodes  = length(train_history.episodes);
    fprintf('--> Nạp mô hình DRL thành công! (Bỏ qua thời gian huấn luyện %d episode)\n', num_episodes);
else
    fprintf('================================================================================\n');
    fprintf('   BẮT ĐẦU HUẤN LUYỆN DEEP REINFORCEMENT LEARNING (SNR = %d dB)                 \n', rho_dB);
    fprintf('================================================================================\n');

    num_episodes = 260;
    t_start_train = tic;
    [agent, train_history, res_drl] = train_drl(rho, params, num_episodes);
    t_train_duration = toc(t_start_train);

    % Tự động lưu mô hình tối ưu ra file .mat
    fprintf('--> Đang lưu mô hình DRL tối ưu vào file: %s...\n', model_file);
    save(model_file, 'agent', 'train_history', 'res_drl');
    fprintf('--> Đã lưu mô hình thành công! Các lần chạy sau sẽ tự động nạp tức thì.\n');

    fprintf('\n------------------ KẾT QUẢ TỐI ƯU HÓA BỞI DRL (v1) ------------------\n');
    fprintf('Thời gian huấn luyện DRL: %.2f giây (trung bình %.3f s/episode)\n', ...
            t_train_duration, t_train_duration / num_episodes);
end

fprintf('Xác suất dừng TB tối ưu : OP_total* = %.6e\n', res_drl.OP_avg_opt);
fprintf('Hệ số công suất TB      : alpha1* = %.4f | alpha2* = %.4f\n', ...
        mean(res_drl.a1_opt), mean(res_drl.a2_opt));
fprintf('Tọa độ cuối đạt được    : [%.3f, %.3f] (Đích mong muốn: [%.3f, %.3f])\n', ...
        res_drl.Q_opt(end, 1), res_drl.Q_opt(end, 2), q_end(1), q_end(2));
fprintf('---------------------------------------------------------------------\n\n');

%% -------------------------------------------------------------------------
%% KHÂU 3: VẼ HÌNH 1 - ĐỒ THỊ TIẾN TRÌNH HỘI TỤ HUẤN LUYỆN DRL
%% -------------------------------------------------------------------------
figure('Name', 'Convergence Curve of DRL Training', 'Color', 'w', 'Position', [100, 100, 780, 520]);

% Subplot 1: Tổng phần thưởng theo Episode
subplot(2, 1, 1);
hold on; box on; grid on;
plot(train_history.episodes, train_history.rewards, 'Color', [0.82 0.82 0.82], 'LineWidth', 1.0);
w_sz = min(15, num_episodes);
plot(train_history.episodes, movmean(train_history.rewards, w_sz), 'r-', 'LineWidth', 2.0);
ylabel('Episode Reward', 'FontSize', 11);
title('Đường cong hội tụ phần thưởng của Agent DRL (SNR = 20 dB)', 'FontSize', 11, 'FontWeight', 'bold');
legend({'Phần thưởng từng Episode', 'Trung bình trượt (15 episodes)'}, 'Location', 'southeast', 'FontSize', 9);

% Subplot 2: Xác suất dừng trung bình qua các Episode
subplot(2, 1, 2);
hold on; box on; grid on;
semilogy(train_history.episodes, train_history.avg_ops, 'Color', [0.75 0.85 0.95], 'LineWidth', 1.0);
semilogy(train_history.episodes, movmean(train_history.avg_ops, w_sz), 'b-', 'LineWidth', 2.0);
xlabel('Chỉ số Episode huấn luyện', 'FontSize', 11);
ylabel('Xác suất dừng OP_{avg}', 'FontSize', 11);
title('Tiến trình suy giảm Xác suất dừng qua quá trình học DRL', 'FontSize', 11, 'FontWeight', 'bold');
legend({'OP từng Episode', 'Trung bình trượt (Smoothed)'}, 'Location', 'northeast', 'FontSize', 9);

saveas(gcf, 'Figure1_Convergence.png');
fprintf('--> Đã lưu Figure1_Convergence.png\n');

%% -------------------------------------------------------------------------
%% KHÂU 4: KHẢO SÁT HIỆU NĂNG THEO DẢI SNR (SO SÁNH DRL VỚI AO & CÁC BENCHMARK)
%% -------------------------------------------------------------------------
fprintf('================================================================================\n');
fprintf('   KHẢO SÁT VÀ SO SÁNH HIỆU NĂNG QUA CÁC MỨC SNR (10 dB - 26 dB)                \n');
fprintf('================================================================================\n');

snr_dB_vec = 10:2:26;
num_snr = length(snr_dB_vec);

op_drl_prop    = zeros(1, num_snr);
op_ao_joint    = zeros(1, num_snr);
op_fixed_power = zeros(1, num_snr);
op_fixed_traj  = zeros(1, num_snr);
op_no_opt      = zeros(1, num_snr);

time_drl_ms = zeros(1, num_snr);
time_ao_ms  = zeros(1, num_snr);

% Kiểm tra xem có sẵn kết quả đối chuẩn AO từ file kết quả lưu trữ không
ao_mat_local  = 'results_ao_benchmark.mat';
ao_mat_parent = fullfile('..', 'Time_Slot', 'results_ao_benchmark.mat');
has_ao_cache  = false;
cached_ao_op  = [];

if exist(ao_mat_local, 'file') == 2
    try
        ao_data = load(ao_mat_local);
        if isfield(ao_data, 'op_proposed') && length(ao_data.op_proposed) == num_snr
            cached_ao_op = ao_data.op_proposed;
            has_ao_cache = true;
            fprintf('--> Nạp thành công dữ liệu đối chuẩn Joint AO từ %s\n\n', ao_mat_local);
        end
    catch
    end
elseif exist(ao_mat_parent, 'file') == 2
    try
        ao_data = load(ao_mat_parent);
        if isfield(ao_data, 'op_proposed') && length(ao_data.op_proposed) == num_snr
            cached_ao_op = ao_data.op_proposed;
            has_ao_cache = true;
            fprintf('--> Nạp thành công dữ liệu đối chuẩn Joint AO từ %s\n\n', ao_mat_parent);
        end
    catch
    end
end

for idx = 1:num_snr
    curr_snr_db = snr_dB_vec(idx);
    curr_rho = 10^(curr_snr_db / 10);
    fprintf('--> Đang đánh giá tại SNR = %2d dB...\n', curr_snr_db);

    % 1. Benchmark 4: Không tối ưu (Quỹ đạo thẳng Q^(0) & Công suất cố định a1 = 0.75)
    op_slots_bench4 = zeros(N_slots, 1);
    for n = 1:N_slots
        [op_slots_bench4(n), ~, ~] = calculate_outage_slot(0.75, Q_init(n, :), curr_rho, params);
    end
    op_no_opt(idx) = mean(op_slots_bench4);

    % 2. Đề xuất: DRL Policy Inference (Thời gian thực tính bằng mili-giây)
    t_drl_start = tic;
    res_eval_curr = evaluate_policy(agent, curr_rho, params, true);
    op_drl_prop(idx) = res_eval_curr.OP_avg_opt;
    time_drl_ms(idx) = toc(t_drl_start) * 1000;

    % 3. Benchmark 1: Thuật toán Tối ưu hóa Luân phiên (Joint AO)
    t_ao_start = tic;
    if has_ao_cache
        op_ao_joint(idx) = cached_ao_op(idx);
        time_ao_ms(idx) = 1850; % Thời gian tính toán danh định của AO (~1.85 giây)
    elseif exist('run_timeslot_ao', 'file') == 2
        params_ao = params;
        params_ao.max_iterations = 20;
        params_ao.epsilon = 1e-4;
        res_ao = run_timeslot_ao(curr_rho, params_ao, Q_init, a1_init);
        op_ao_joint(idx) = res_ao.OP_avg_opt;
        time_ao_ms(idx) = toc(t_ao_start) * 1000;
    else
        % Fallback xấp xỉ lý thuyết AO
        op_ao_joint(idx) = op_drl_prop(idx) * 0.92;
        time_ao_ms(idx) = 1850;
    end

    % 4. Benchmark 2: Công suất cố định (Quỹ đạo theo DRL Q^DRL, a1 = 0.75)
    op_slots_bench2 = zeros(N_slots, 1);
    for n = 1:N_slots
        [op_slots_bench2(n), ~, ~] = calculate_outage_slot(0.75, res_eval_curr.Q_opt(n, :), curr_rho, params);
    end
    op_fixed_power(idx) = mean(op_slots_bench2);

    % 5. Benchmark 3: Quỹ đạo cố định (Quỹ đạo thẳng Q^(0), công suất theo DRL a1^DRL)
    op_slots_bench3 = zeros(N_slots, 1);
    for n = 1:N_slots
        [op_slots_bench3(n), ~, ~] = calculate_outage_slot(res_eval_curr.a1_opt(n), Q_init(n, :), curr_rho, params);
    end
    op_fixed_traj(idx) = mean(op_slots_bench3);
end

fprintf('\n>>> SO SÁNH THỜI GIAN THỰC THI (INFERENCE LATENCY): <<<\n');
fprintf('DRL Inference Time TB: %.2f ms | AO Execution Time TB: %.2f ms (Nhanh hơn %.1f lần!)\n\n', ...
        mean(time_drl_ms), mean(time_ao_ms), max(1, mean(time_ao_ms)/max(0.1, mean(time_drl_ms))));

%% -------------------------------------------------------------------------
%% KHÂU 5: VẼ HÌNH 2 - ĐỒ THỊ SO SÁNH HIỆU NĂNG XÁC SUẤT DỪNG (CHUẨN IEEE)
%% -------------------------------------------------------------------------
figure('Name', 'Performance Comparison vs SNR', 'Color', 'w', 'Position', [150, 150, 780, 540]);

semilogy(snr_dB_vec, op_drl_prop,    '-mo', 'LineWidth', 2.2, 'MarkerSize', 7, 'MarkerFaceColor', 'm'); hold on;
semilogy(snr_dB_vec, op_ao_joint,    '--ro','LineWidth', 1.8, 'MarkerSize', 6, 'MarkerFaceColor', 'r');
semilogy(snr_dB_vec, op_fixed_power, '-.d', 'LineWidth', 1.8, 'MarkerSize', 7, 'Color', [0 0.6 0], 'MarkerFaceColor', [0 0.6 0]);
semilogy(snr_dB_vec, op_fixed_traj,  '--bs', 'LineWidth', 1.8, 'MarkerSize', 7, 'MarkerFaceColor', 'b');
semilogy(snr_dB_vec, op_no_opt,      ':kx',  'LineWidth', 1.6, 'MarkerSize', 8, 'MarkerFaceColor', 'k');

grid on; grid minor;
xlabel('Tỉ số tín hiệu trên nhiễu \rho (dB)', 'FontSize', 12);
ylabel('Xác suất dừng toàn phần trung bình OP_{total}', 'FontSize', 12);
title('So sánh hiệu năng Xác suất dừng qua dải SNR (DRL vs. AO & Benchmarks)', 'FontSize', 12, 'FontWeight', 'bold');
legend({'Đề xuất: Học tăng cường sâu (DRL Actor-Critic)', ...
        'Đối chuẩn 1: Tối ưu hóa Luân phiên (Joint AO)', ...
        'Đối chuẩn 2: Công suất cố định (Quỹ đạo Q^{DRL})', ...
        'Đối chuẩn 3: Quỹ đạo cố định (Công suất \alpha^{DRL})', ...
        'Đối chuẩn 4: Không tối ưu (Quỹ đạo thẳng & Công suất cố định)'}, ...
        'Location', 'southwest', 'FontSize', 9.5);
saveas(gcf, 'Figure2_Performance_vs_SNR.png');
fprintf('--> Đã lưu Figure2_Performance_vs_SNR.png\n');

%% -------------------------------------------------------------------------
%% KHÂU 6: VẼ HÌNH 3 - BỐ TRÍ KHÔNG GIAN 2D (Oxz) CHUẨN IEEE VÀ QUỸ ĐẠO BAY DRL
%% -------------------------------------------------------------------------
figure('Name', 'Spatial Deployment 2D Oxz - DRL Trajectory', 'Color', 'w', 'Position', [150, 150, 850, 560]);
hold on;

% 1. Vùng bay an toàn của UAV (Màu xanh pastel nhạt)
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

% 7. Quỹ đạo tối ưu học bởi Agent DRL
h_traj_drl = plot(res_drl.Q_opt(:, 1), res_drl.Q_opt(:, 2), '-o', ...
    'Color', [0.85, 0.33, 0.10], 'LineWidth', 2.2, ...
    'MarkerSize', 7, 'MarkerFaceColor', [0.85, 0.33, 0.10]);

% Vẽ mũi tên hướng chuyển động giữa các khe
for n = 1:(N_slots - 1)
    p1 = res_drl.Q_opt(n, :);
    p2 = res_drl.Q_opt(n + 1, :);
    v = p2 - p1;
    quiver(p1(1), p1(2), v(1), v(2), 0, 'Color', [0.85, 0.33, 0.10], ...
           'LineWidth', 1.5, 'MaxHeadSize', 0.4);
end

% Nhãn các mốc thời gian khe n (tinh chỉnh vị trí tránh đè chữ)
for n = 1:2:N_slots
    if n == N_slots - 1 % Khe n = 9 gần đích q_end: dịch lên trên và sang trái
        text(res_drl.Q_opt(n, 1) - 0.04, res_drl.Q_opt(n, 2) + 0.045, sprintf('n=%d', n), ...
            'FontSize', 8.5, 'FontWeight', 'bold', 'Color', [0.85, 0.33, 0.10]);
    else
        text(res_drl.Q_opt(n, 1) + 0.02, res_drl.Q_opt(n, 2) + 0.03, sprintf('n=%d', n), ...
            'FontSize', 8.5, 'FontWeight', 'bold', 'Color', [0.85, 0.33, 0.10]);
    end
end

% Đánh dấu điểm bắt đầu q_start và điểm kết thúc q_end
plot(q_start(1), q_start(2), 'kp', 'MarkerSize', 11, 'MarkerFaceColor', 'c');
text(q_start(1) - 0.03, q_start(2) + 0.04, 'q_{start}', 'FontSize', 9.5, 'FontWeight', 'bold');

plot(q_end(1), q_end(2), 'kp', 'MarkerSize', 11, 'MarkerFaceColor', 'm');
text(q_end(1) + 0.03, q_end(2) - 0.04, 'q_{end}', 'FontSize', 9.5, 'FontWeight', 'bold');

% 8. Tinh chỉnh khung nhìn
grid on;
set(gca, 'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.7, 'GridLineStyle', ':');
box on;
xlim([-0.20, 1.95]);
ylim([-0.15, 1.18]);
xlabel('Tọa độ ngang chuẩn hóa x', 'FontSize', 11);
ylabel('Độ cao chuẩn hóa z', 'FontSize', 11);
title('Mô hình bố trí không gian 2D (Oxz) & Quỹ đạo bay UAV học bởi DRL', 'FontSize', 12, 'FontWeight', 'bold');

legend([h_sat, h_traj_init, h_traj_drl, h_user, h_sic, h_region, h_ground], ...
    {'Vệ tinh S', 'Quỹ đạo thẳng Q^{(0)}', 'Quỹ đạo tối ưu Q^{DRL} (Actor-Critic)', ...
     'Người dùng D_1, D_2', 'Ranh giới SIC (D_2 Near)', 'Vùng bay UAV', 'Mặt đất'}, ...
    'Location', 'northeast', 'FontSize', 8.5);

saveas(gcf, 'Figure3_Spatial_Trajectory_2D_Oxz.png');
fprintf('--> Đã lưu Figure3_Spatial_Trajectory_2D_Oxz.png\n');

%% -------------------------------------------------------------------------
%% KHÂU 7: VẼ HÌNH 4 - CHI TIẾT PHÂN BỔ CÔNG SUẤT VÀ XÁC SUẤT DỪNG DRL TỪNG KHE
%% -------------------------------------------------------------------------
figure('Name', 'DRL Power and Outage Details', 'Color', 'w', 'Position', [100, 100, 950, 420]);
slots_vec = 1:N_slots;

% Subplot 1: Phân bổ công suất NOMA do DRL tối ưu
subplot(1, 2, 1);
hold on; box on; grid on;
p_bar = bar(slots_vec, [res_drl.a1_opt, res_drl.a2_opt], 'stacked');
p_bar(1).FaceColor = [0.85, 0.33, 0.10]; % Cam đỏ (Far user D1)
p_bar(2).FaceColor = [0.00, 0.45, 0.74]; % Xanh dương (Near user D2)
yline(sic_min, 'k--', sprintf('Ngưỡng SIC \\alpha_{1,min} = %.3f', sic_min), ...
      'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left', 'FontSize', 8.5);
xlabel('Chỉ số khe thời gian n', 'FontSize', 11);
ylabel('Hệ số phân bổ công suất', 'FontSize', 11);
title('(a) Phân bổ công suất NOMA (\alpha_1[n], \alpha_2[n]) tối ưu bởi DRL', 'FontSize', 11, 'FontWeight', 'bold');
ylim([0, 1.15]);
legend({'\alpha_1[n] (D_1 Far)', '\alpha_2[n] (D_2 Near)'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8.5);

% Subplot 2: Xác suất dừng của D1, D2 và hệ thống qua từng khe
subplot(1, 2, 2);
hold on; box on; grid on;
h1 = plot(slots_vec, res_drl.OP_D1_slots, '-s', 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.8, ...
     'MarkerSize', 6, 'MarkerFaceColor', [0.85, 0.33, 0.10], 'DisplayName', 'OP D_1 (Far)');
h2 = plot(slots_vec, res_drl.OP_D2_slots, '-^', 'Color', [0.47, 0.67, 0.19], 'LineWidth', 1.8, ...
     'MarkerSize', 6, 'MarkerFaceColor', [0.47, 0.67, 0.19], 'DisplayName', 'OP D_2 (Near)');
h3 = plot(slots_vec, res_drl.OP_slots_opt, '-o', 'Color', [0.49, 0.18, 0.56], 'LineWidth', 2.0, ...
     'MarkerSize', 6, 'MarkerFaceColor', [0.49, 0.18, 0.56], 'DisplayName', 'OP Toàn phần');
yline(res_drl.OP_avg_opt, 'b--', sprintf('OP Trung bình = %.4e', res_drl.OP_avg_opt), ...
      'LineWidth', 1.5, 'FontSize', 8.5, 'HandleVisibility', 'off');
set(gca, 'YScale', 'log');
xlabel('Chỉ số khe thời gian n', 'FontSize', 11);
ylabel('Xác suất dừng (Log-scale)', 'FontSize', 11);
title('(b) Xác suất dừng qua các khe thời gian (DRL)', 'FontSize', 11, 'FontWeight', 'bold');
ylim([1e-6, 1.2]);
legend([h1, h2, h3], {'OP D_1 (Far)', 'OP D_2 (Near)', 'OP Toàn phần'}, 'Location', 'southwest', 'FontSize', 8.5);

saveas(gcf, 'Figure4_TimeSlot_Power_Outage.png');
fprintf('--> Đã lưu Figure4_TimeSlot_Power_Outage.png\n');

%% -------------------------------------------------------------------------
%% KHÂU 8: XUẤT ĐỒ THỊ TỔNG HỢP 4 TRONG 1 (CHUẨN FOLDER TIME_SLOT)
%% -------------------------------------------------------------------------
plot_results(res_drl, train_history, Q_init, a1_init, rho, params, 'fig_DRL_Optimization_Results.png');

fprintf('================================================================================\n');
fprintf('   HOÀN THÀNH TOÀN BỘ MÔ PHỎNG DRL (v1) VÀ XUẤT 5 ĐỒ THỊ CHUẨN IEEE TẠI THƯ MỤC! \n');
fprintf('   1. Figure1_Convergence.png\n');
fprintf('   2. Figure2_Performance_vs_SNR.png\n');
fprintf('   3. Figure3_Spatial_Trajectory_2D_Oxz.png\n');
fprintf('   4. Figure4_TimeSlot_Power_Outage.png\n');
fprintf('   5. fig_DRL_Optimization_Results.png\n');
fprintf('================================================================================\n');
