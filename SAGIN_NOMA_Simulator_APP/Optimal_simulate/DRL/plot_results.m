function plot_results(res_drl, train_history, Q_init, a1_init, rho_val, params, save_filename)
% PLOT_RESULTS: Trực quan hóa kết quả tối ưu hóa DRL trong mặt phẳng 2D (Oxz) theo Time-Slotting
%
% Tạo đồ thị tổng hợp 4 trong 1 chuẩn IEEE đồng nhất với folder Time_Slot:
%   1. Quỹ đạo bay 2D (Oxz) & Ranh giới hình học SIC (So sánh Quỹ đạo thẳng & DRL)
%   2. Phân bổ công suất NOMA (a1[n], a2[n]) qua các khe thời gian
%   3. Xác suất dừng qua các khe thời gian (D1, D2 và Toàn phần)
%   4. Đường cong học DRL (Phần thưởng tích lũy & Xác suất dừng qua các Episode)

    if nargin < 7 || isempty(save_filename)
        save_filename = 'fig_DRL_Optimization_Results.png';
    end

    N = params.N_slots;
    slots_vec = 1:N;

    figure('Name', 'Ket Qua Toi Uu DRL Time-Slotting 2D Oxz', ...
           'Color', 'w', 'Position', [80, 80, 1150, 820]);

    c_blue   = [0.00, 0.45, 0.74];
    c_red    = [0.85, 0.33, 0.10];
    c_purple = [0.49, 0.18, 0.56];
    c_green  = [0.47, 0.67, 0.19];

    %% --- ĐỒ THỊ 1: BỐ TRÍ KHÔNG GIAN 2D (Oxz) & QUỸ ĐẠO BAY DRL ---
    subplot(2, 2, 1);
    hold on; box on; grid on;

    % Vùng bay UAV (màu xanh pastel)
    rectangle('Position', [params.uav_lb(1), params.uav_lb(2), ...
                           params.uav_ub(1) - params.uav_lb(1), params.uav_ub(2) - params.uav_lb(2)], ...
              'LineStyle', '--', 'EdgeColor', [0.45 0.65 0.85], 'LineWidth', 1.2, ...
              'FaceColor', [0.92, 0.96, 1.0]);

    % Ranh giới hình học SIC
    x_sic_bound = 0.5 * (params.pos_D1(1) + params.pos_D2(1)) - params.SIC_margin;
    plot([x_sic_bound, x_sic_bound], [params.uav_lb(2), params.uav_ub(2)], ...
         'r--', 'LineWidth', 1.8, 'DisplayName', 'Ranh giới SIC (D_2 Near)');

    % Mặt đất
    plot([-0.18, 1.88], [0, 0], 'k-', 'LineWidth', 1.6, 'DisplayName', 'Mặt đất (z=0)');

    % Vệ tinh S và người dùng
    plot(params.pos_S(1), params.pos_S(2), 'k*', 'MarkerSize', 10, 'LineWidth', 1.8, 'DisplayName', 'Vệ tinh S');
    plot(params.pos_D1(1), params.pos_D1(2), 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'k', 'DisplayName', 'D_1, D_2');
    plot(params.pos_D2(1), params.pos_D2(2), 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'k');

    % Quỹ đạo ban đầu (đường thẳng nối q_start và q_end)
    plot(Q_init(:, 1), Q_init(:, 2), 'k:d', 'LineWidth', 1.2, 'MarkerSize', 5, ...
         'MarkerFaceColor', 'w', 'DisplayName', 'Quỹ đạo thẳng Q^{(0)}');

    % Quỹ đạo tối ưu DRL
    plot(res_drl.Q_opt(:, 1), res_drl.Q_opt(:, 2), '-o', 'Color', c_blue, 'LineWidth', 2.0, ...
         'MarkerSize', 6, 'MarkerFaceColor', c_blue, 'DisplayName', 'Quỹ đạo tối ưu Q^{DRL}');

    % Mũi tên hướng bay giữa các khe
    for n = 1:(N - 1)
        p1 = res_drl.Q_opt(n, :);
        p2 = res_drl.Q_opt(n + 1, :);
        v_step = p2 - p1;
        quiver(p1(1), p1(2), v_step(1), v_step(2), 0, 'Color', c_blue, 'LineWidth', 1.2, 'MaxHeadSize', 0.35);
    end

    % Đánh dấu điểm bắt đầu q_start và đích q_end
    plot(params.q_start(1), params.q_start(2), 'kp', 'MarkerSize', 9, 'MarkerFaceColor', 'c');
    plot(params.q_end(1), params.q_end(2), 'kp', 'MarkerSize', 9, 'MarkerFaceColor', 'm');
    text(params.q_start(1) - 0.03, params.q_start(2) + 0.04, 'q_{start}', 'FontSize', 8, 'FontWeight', 'bold');
    text(params.q_end(1) + 0.03, params.q_end(2) - 0.04, 'q_{end}', 'FontSize', 8, 'FontWeight', 'bold');

    title('1. Bố trí không gian 2D (Oxz) & Quỹ đạo bay DRL', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Tọa độ ngang chuẩn hóa x', 'FontSize', 10);
    ylabel('Độ cao chuẩn hóa z', 'FontSize', 10);
    xlim([-0.20, 1.95]);
    ylim([-0.15, 1.18]);
    legend('Location', 'northeast', 'FontSize', 7.5);

    %% --- ĐỒ THỊ 2: PHÂN BỔ CÔNG SUẤT NOMA THEO THỜI GIAN ---
    subplot(2, 2, 2);
    hold on; box on; grid on;

    power_bar = bar(slots_vec, [res_drl.a1_opt, res_drl.a2_opt], 'stacked');
    power_bar(1).FaceColor = c_red;
    power_bar(2).FaceColor = c_blue;

    a1_min = params.gamma_th1 / (1 + params.gamma_th1);
    yline(a1_min, 'k--', sprintf('Ngưỡng SIC a_{1,min} = %.3f', a1_min), ...
          'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left', 'FontSize', 8);

    title('2. Phân bổ công suất NOMA (a_1[n], a_2[n]) qua các khe', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Chỉ số khe thời gian n', 'FontSize', 10);
    ylabel('Hệ số công suất', 'FontSize', 10);
    ylim([0, 1.15]);
    legend({'a_1[n] (Far-user D_1)', 'a_2[n] (Near-user D_2)'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8.5);

    %% --- ĐỒ THỊ 3: XÁC SUẤT DỪNG QUA TỪNG KHE THỜI GIAN ---
    subplot(2, 2, 3);
    hold on; box on; grid on;

    plot(slots_vec, res_drl.OP_D1_slots, '-s', 'Color', c_red, 'LineWidth', 1.8, ...
         'MarkerSize', 6, 'MarkerFaceColor', c_red, 'DisplayName', 'P_{out} D_1 (Far)');
    plot(slots_vec, res_drl.OP_D2_slots, '-^', 'Color', c_green, 'LineWidth', 1.8, ...
         'MarkerSize', 6, 'MarkerFaceColor', c_green, 'DisplayName', 'P_{out} D_2 (Near)');
    plot(slots_vec, res_drl.OP_slots_opt, '-o', 'Color', c_purple, 'LineWidth', 2.0, ...
         'MarkerSize', 6, 'MarkerFaceColor', c_purple, 'DisplayName', 'P_{out} Toàn phần');

    yline(res_drl.OP_avg_opt, 'b--', sprintf('OP Trung bình = %.4e', res_drl.OP_avg_opt), ...
          'LineWidth', 1.5, 'FontSize', 8);

    set(gca, 'YScale', 'log');
    title('3. Xác suất dừng qua các khe thời gian (Log-scale)', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Chỉ số khe thời gian n', 'FontSize', 10);
    ylabel('Xác suất dừng', 'FontSize', 10);
    ylim([1e-6, 1.2]);
    legend('Location', 'southwest', 'FontSize', 8);

    %% --- ĐỒ THỊ 4: ĐƯỜNG CONG HỘI TỤ HUẤN LUYỆN DRL ---
    subplot(2, 2, 4);
    hold on; box on; grid on;

    episodes = train_history.episodes;
    rewards  = train_history.rewards;

    % Vẽ đường phần thưởng gốc và đường trung bình động (Moving Average)
    plot(episodes, rewards, 'Color', [0.8 0.8 0.8], 'LineWidth', 1.0, 'DisplayName', 'Reward Episode');
    window_sz = min(15, length(rewards));
    reward_smooth = movmean(rewards, window_sz);
    plot(episodes, reward_smooth, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Trung bình trượt (Smoothed)');

    title('4. Tiến trình hội tụ Phần thưởng DRL qua các Episode', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Chỉ số Episode', 'FontSize', 10);
    ylabel('Tổng phần thưởng (Reward)', 'FontSize', 10);
    legend('Location', 'southeast', 'FontSize', 8.5);

    % Lưu đồ thị ra file
    saveas(gcf, save_filename);
    fprintf('--> Đã lưu đồ thị tổng hợp: %s\n', save_filename);
end
