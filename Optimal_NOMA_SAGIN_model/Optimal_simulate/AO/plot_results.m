function plot_results(res_ao, snr_dB_vec, alpha1_opt_vec, alpha2_opt_vec, op_d1_opt_vec, op_d2_opt_vec, op_proposed, fixed_q, params, save_filename)
% PLOT_RESULTS: Trực quan hóa toàn diện kết quả tối ưu hóa AO trong mặt phẳng 2D (Oxz)
% Đồ thị 4 trong 1 đồng nhất với phong cách IEEE của folder Time_Slot và DRL:
%   1. Bố trí không gian 2D (Oxz) & Vị trí tối ưu UAV
%   2. Phân bổ công suất NOMA (alpha1, alpha2) theo dải SNR
%   3. Xác suất dừng chi tiết (D1, D2, Toàn phần) theo dải SNR
%   4. Đường cong hội tụ của thuật toán AO

    if nargin < 10 || isempty(save_filename)
        save_filename = 'fig_AO_Optimization_Results.png';
    end

    fig = figure('Name', 'Ket Qua Toi Uu AO 2D Oxz', ...
                 'Color', 'w', 'Position', [80, 80, 1150, 820]);

    c_blue   = [0.00, 0.45, 0.74];
    c_red    = [0.85, 0.33, 0.10];
    c_purple = [0.49, 0.18, 0.56];
    c_green  = [0.47, 0.67, 0.19];

    %% --- ĐỒ THỊ 1: BỐ TRÍ KHÔNG GIAN 2D (Oxz) & VỊ TRÍ TỐI ƯU UAV ---
    subplot(2, 2, 1);
    hold on; box on; grid on;

    % Vùng bay UAV (màu xanh pastel)
    rectangle('Position', [params.uav_lb(1), params.uav_lb(2), ...
                           params.uav_ub(1) - params.uav_lb(1), params.uav_ub(2) - params.uav_lb(2)], ...
              'LineStyle', '--', 'EdgeColor', [0.45 0.65 0.85], 'LineWidth', 1.2, ...
              'FaceColor', [0.92, 0.96, 1.0]);

    % Mặt đất
    plot([-0.18, 1.88], [0, 0], 'k-', 'LineWidth', 1.6, 'DisplayName', 'Mặt đất (z=0)');

    % Vệ tinh S và người dùng
    plot(params.pos_S(1), params.pos_S(2), 'k*', 'MarkerSize', 10, 'LineWidth', 1.8, 'DisplayName', 'Vệ tinh S');
    plot([params.pos_D2(1), params.pos_D1(1)], [params.pos_D2(2), params.pos_D1(2)], ...
         'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'k', 'DisplayName', 'D_1, D_2');

    % UAV ban đầu và tối ưu
    plot(fixed_q(1), fixed_q(2), 'k^', 'MarkerSize', 8, 'LineWidth', 1.5, ...
         'MarkerFaceColor', 'w', 'DisplayName', 'UAV ban đầu U^{(0)}');
    plot(res_ao.optimal_q(1), res_ao.optimal_q(2), 'k^', 'MarkerSize', 9, ...
         'MarkerFaceColor', 'k', 'DisplayName', 'UAV tối ưu U^*');

    % Mũi tên dịch chuyển
    v_dir = res_ao.optimal_q - fixed_q;
    v_unit = v_dir / norm(v_dir);
    p_start = fixed_q + 0.03 * v_unit;
    p_vec = v_dir - 0.06 * v_unit;
    quiver(p_start(1), p_start(2), p_vec(1), p_vec(2), 0, 'k--', ...
           'LineWidth', 1.2, 'MaxHeadSize', 0.22);

    title('1. Bố trí không gian 2D (Oxz) & Vị trí UAV', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Tọa độ ngang chuẩn hóa x', 'FontSize', 10);
    ylabel('Độ cao chuẩn hóa z', 'FontSize', 10);
    xlim([-0.20, 1.95]);
    ylim([-0.15, 1.18]);
    legend('Location', 'northeast', 'FontSize', 7.5);

    %% --- ĐỒ THỊ 2: PHÂN BỔ CÔNG SUẤT NOMA THEO DẢI SNR ---
    subplot(2, 2, 2);
    hold on; box on; grid on;

    power_bar = bar(snr_dB_vec, [alpha1_opt_vec, alpha2_opt_vec], 'stacked');
    power_bar(1).FaceColor = c_red;
    power_bar(2).FaceColor = c_blue;

    a1_min = params.gamma_th1 / (1 + params.gamma_th1);
    yline(a1_min, 'k--', sprintf('Ngưỡng SIC \\alpha_{1,min} = %.3f', a1_min), ...
          'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left', 'FontSize', 8);

    title('2. Phân bổ công suất NOMA (a_1^*, a_2^*) theo SNR', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Tỉ số tín hiệu trên nhiễu \rho (dB)', 'FontSize', 10);
    ylabel('Hệ số công suất', 'FontSize', 10);
    ylim([0, 1.15]);
    legend({'a_1^* (Far-user D_1)', 'a_2^* (Near-user D_2)'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8.5);

    %% --- ĐỒ THỊ 3: XÁC SUẤT DỪNG CHI TIẾT THEO DẢI SNR ---
    subplot(2, 2, 3);
    hold on; box on; grid on;

    plot(snr_dB_vec, op_d1_opt_vec, '-s', 'Color', c_red, 'LineWidth', 1.8, ...
         'MarkerSize', 6, 'MarkerFaceColor', c_red, 'DisplayName', 'P_{out} D_1 (Far)');
    plot(snr_dB_vec, op_d2_opt_vec, '-^', 'Color', c_green, 'LineWidth', 1.8, ...
         'MarkerSize', 6, 'MarkerFaceColor', c_green, 'DisplayName', 'P_{out} D_2 (Near)');
    plot(snr_dB_vec, op_proposed, '-o', 'Color', c_purple, 'LineWidth', 2.0, ...
         'MarkerSize', 6, 'MarkerFaceColor', c_purple, 'DisplayName', 'P_{out} Toàn phần');

    set(gca, 'YScale', 'log');
    title('3. Xác suất dừng chi tiết qua dải SNR', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Tỉ số tín hiệu trên nhiễu \rho (dB)', 'FontSize', 10);
    ylabel('Xác suất dừng (Thang Log)', 'FontSize', 10);
    ylim([1e-4, 1.2]);
    legend('Location', 'southwest', 'FontSize', 8.5);

    %% --- ĐỒ THỊ 4: ĐƯỜNG CONG HỘI TỤ CỦA THUẬT TOÁN AO ---
    subplot(2, 2, 4);
    hold on; box on; grid on;

    iter_vec = 0:(length(res_ao.history_F) - 1);
    semilogy(iter_vec, res_ao.history_F, '-o', ...
             'Color', c_blue, 'LineWidth', 2.0, ...
             'MarkerSize', 6, 'MarkerFaceColor', c_blue);

    title(sprintf('4. Đường cong hội tụ AO (%d vòng lặp)', length(iter_vec) - 1), ...
          'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Số vòng lặp (Iteration k)', 'FontSize', 10);
    ylabel('Hàm mục tiêu F(k) (OP Toàn phần)', 'FontSize', 10);

    text(0, res_ao.history_F(1), sprintf('  Khởi tạo: %.4e', res_ao.history_F(1)), ...
         'FontSize', 8.5, 'Color', 'k');
    text(iter_vec(end), res_ao.history_F(end), sprintf('  Hội tụ: %.4e', res_ao.history_F(end)), ...
         'FontSize', 8.5, 'FontWeight', 'bold', 'Color', c_blue);

    sgtitle('MÔ HÌNH HÓA 2D (Oxz) VÀ TỐI ƯU HÓA LUÂN PHIÊN (AO) MẠNG SAGIN-NOMA', ...
            'FontSize', 12, 'FontWeight', 'bold');

    try
        saveas(fig, save_filename);
        fprintf('--> Đã lưu đồ thị tổng hợp: %s\n', save_filename);
    catch
    end
end
