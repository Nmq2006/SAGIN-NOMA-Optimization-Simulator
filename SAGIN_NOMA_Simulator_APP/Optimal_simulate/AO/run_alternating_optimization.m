function results = run_alternating_optimization(rho_val, params, init_q, init_a1)
    % =========================================================================
    % THUẬT TOÁN TỐI ƯU HÓA LUÂN PHIÊN (AO) - MÔ HÌNH 2D (Oxz) CHUẨN HÓA
    % Biến tối ưu: Hệ số công suất alpha1 và Tọa độ UAV q_U = [x_tilde, z_tilde]
    % =========================================================================

    % 1. Khởi tạo nghiệm ban đầu nếu chưa được cung cấp
    if nargin < 3 || isempty(init_q)
        q_k = [0.5, 0.4]; % Vị trí khởi tạo UAV [x, z]
    else
        q_k = init_q;
    end
    
    if nargin < 4 || isempty(init_a1)
        sic_min = params.gamma_th1 / (1.0 + params.gamma_th1);
        alpha1_k = (sic_min + 1.0) / 2.0;
    else
        alpha1_k = init_a1;
    end

    [f_current, op_d1_0, op_d2_0] = calculate_outage_probability(alpha1_k, q_k, rho_val, params);

    % Mảng lưu lịch sử hội tụ
    history_F      = f_current;
    history_q      = q_k;
    history_alpha  = alpha1_k;
    history_OP_D1  = op_d1_0;
    history_OP_D2  = op_d2_0;

    fprintf('--- BẮT ĐẦU AO VỚI SNR = %.1f dB ---\n', 10*log10(rho_val));
    fprintf('Khởi tạo: OP = %.6e, alpha1 = %.4f, q_U = [%.3f, %.3f]\n', ...
        f_current, alpha1_k, q_k(1), q_k(2));

    % 2. Vòng lặp luân phiên (Alternating Optimization Loop)
    for k = 1:params.max_iterations
        % Bước 1: Cố định vị trí UAV q_U^(k), tối ưu công suất alpha1^(k+1)
        [alpha1_next, ~] = solve_subproblem_power(q_k, rho_val, params);

        % Bước 2: Cố định công suất alpha1^(k+1), tối ưu tọa độ q_U^(k+1)
        [q_next, f_next] = solve_subproblem_position(alpha1_next, q_k, rho_val, params);
        [~, op_d1_k, op_d2_k] = calculate_outage_probability(alpha1_next, q_next, rho_val, params);

        % Lưu vết lịch sử
        history_F(end+1)      = f_next;
        history_q(end+1, :)   = q_next;
        history_alpha(end+1)  = alpha1_next;
        history_OP_D1(end+1)  = op_d1_k;
        history_OP_D2(end+1)  = op_d2_k;

        % Kiểm tra sai số theo tiêu chuẩn dừng (Công thức 26 & 27)
        delta_obj  = abs(f_next - f_current) / (abs(f_current) + 1e-9);
        step_q     = norm(q_next - q_k);
        step_alpha = abs(alpha1_next - alpha1_k);

        fprintf('Vòng %02d: OP = %.6e | d_OP = %.2e | step_q = %.4f\n', ...
            k, f_next, delta_obj, step_q);

        % Điều kiện dừng hội tụ
        if (delta_obj <= params.epsilon) && (step_q <= params.delta_q) && (step_alpha <= params.delta_a)
            fprintf('==> Thuật toán HỘI TỤ thành công tại vòng lặp thứ %d!\n', k);
            break;
        end

        % Cập nhật biến cho vòng lặp tiếp theo
        q_k = q_next;
        alpha1_k = alpha1_next;
        f_current = f_next;
    end

    % Đóng gói kết quả đầu ra
    results.optimal_alpha1 = alpha1_k;
    results.optimal_alpha2 = 1.0 - alpha1_k;
    results.optimal_q      = q_k;
    results.optimal_OP     = f_current;
    results.history_F      = history_F;
    results.history_q      = history_q;
    results.history_alpha  = history_alpha;
    results.history_OP_D1  = history_OP_D1;
    results.history_OP_D2  = history_OP_D2;
end
