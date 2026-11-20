function res = run_timeslot_ao(rho_val, params, Q_init, a1_init)
% RUN_TIMESLOT_AO: Thuật toán Tối ưu hóa Luân phiên (AO) trong mặt phẳng 2D (Oxz) theo Time-Slotting
%
% Inputs:
%   rho_val: Mức SNR phát tuyến tính
%   params: Cấu trúc tham số hệ thống
%   Q_init: Quỹ đạo ban đầu (N x 2) [tùy chọn], mỗi hàng là [x_n, z_n]
%   a1_init: Phân bổ công suất ban đầu (N x 1) [tùy chọn]
%
% Outputs:
%   res: Cấu trúc chứa toàn bộ kết quả tối ưu:
%        res.Q_opt: Quỹ đạo UAV tối ưu (N x 2) trong mặt phẳng Oxz
%        res.a1_opt: Hệ số công suất a1 tối ưu tại từng khe (N x 1)
%        res.a2_opt: Hệ số công suất a2 tối ưu tại từng khe (N x 1)
%        res.OP_avg_opt: Xác suất dừng trung bình tối ưu
%        res.OP_slots_opt: Xác suất dừng tại từng khe (N x 1)
%        res.OP_D1_slots: Xác suất dừng D1 tại từng khe (N x 1)
%        res.OP_D2_slots: Xác suất dừng D2 tại từng khe (N x 1)
%        res.history_F: Vector lịch sử giá trị hàm mục tiêu qua các vòng lặp AO
%        res.iterations: Số vòng lặp thực hiện

    N = params.N_slots;

    % Khởi tạo Quỹ đạo ban đầu nếu chưa truyền: Bay thẳng đều từ q_start đến q_end
    if nargin < 3 || isempty(Q_init)
        Q_curr = zeros(N, 2);
        for n = 1:N
            alpha = (n - 1) / (N - 1);
            Q_curr(n, :) = (1 - alpha) * params.q_start + alpha * params.q_end;
        end
    else
        Q_curr = Q_init;
    end

    % Khởi tạo Công suất ban đầu nếu chưa truyền
    if nargin < 4 || isempty(a1_init)
        a1_curr = 0.75 * ones(N, 1);
    else
        a1_curr = a1_init;
    end

    % Đánh giá hàm mục tiêu ban đầu
    [~, init_costs] = solve_subproblem_power(Q_curr, rho_val, params);
    cost_curr = mean(init_costs);

    history_F = zeros(params.max_iterations + 1, 1);
    history_F(1) = cost_curr;

    fprintf('========================================================================================\n');
    fprintf(' VÒNG LẶP AO |  F(k) (OP TB)  | SAI SỐ TƯƠNG ĐỐI | TRUNG BÌNH a1 | KHOẢNG CÁCH D1 | KHOẢNG CÁCH D2\n');
    fprintf('========================================================================================\n');
    d_D1_init = mean(vecnorm(Q_curr - params.pos_D1, 2, 2));
    d_D2_init = mean(vecnorm(Q_curr - params.pos_D2, 2, 2));
    fprintf('  Khởi tạo   |  %12.6e  |       ------     |    %7.4f    |    %7.4f     |    %7.4f\n', ...
            cost_curr, mean(a1_curr), d_D1_init, d_D2_init);

    converged = false;
    iter = 0;

    for k = 1:params.max_iterations
        iter = k;

        % --- BƯỚC 1: Tối ưu Phân bổ công suất khi cố định Quỹ đạo ---
        [a1_curr, ~] = solve_subproblem_power(Q_curr, rho_val, params);

        % --- BƯỚC 2: Tối ưu Quỹ đạo UAV khi cố định Công suất ---
        [Q_curr, cost_new] = solve_subproblem_trajectory(a1_curr, Q_curr, rho_val, params);

        % Đánh giá tiêu chuẩn dừng hội tụ
        rel_err = abs(cost_new - cost_curr) / (abs(cost_curr) + 1e-8);
        history_F(k + 1) = cost_new;

        d_D1_avg = mean(vecnorm(Q_curr - params.pos_D1, 2, 2));
        d_D2_avg = mean(vecnorm(Q_curr - params.pos_D2, 2, 2));

        fprintf('   Vòng %02d   |  %12.6e  |   %12.4e   |    %7.4f    |    %7.4f     |    %7.4f\n', ...
                k, cost_new, rel_err, mean(a1_curr), d_D1_avg, d_D2_avg);

        cost_curr = cost_new;

        if rel_err <= params.epsilon
            fprintf('----------------------------------------------------------------------------------------\n');
            fprintf('--> Thuật toán AO đã HỘI TỤ tại vòng lặp %d (Sai số tương đối: %.4e <= %.1e)\n', ...
                    k, rel_err, params.epsilon);
            converged = true;
            history_F = history_F(1:k + 1);
            break;
        end
    end

    if ~converged
        fprintf('----------------------------------------------------------------------------------------\n');
        fprintf('--> Đạt số vòng lặp tối đa (%d vòng).\n', params.max_iterations);
    end

    % Tính toán phân tích chi tiết xác suất dừng tại nghiệm tối ưu cuối cùng
    OP_slots    = zeros(N, 1);
    OP_D1_slots = zeros(N, 1);
    OP_D2_slots = zeros(N, 1);
    for n = 1:N
        [OP_slots(n), OP_D1_slots(n), OP_D2_slots(n)] = ...
            calculate_outage_slot(a1_curr(n), Q_curr(n, :), rho_val, params);
    end

    % Đóng gói kết quả
    res.Q_opt        = Q_curr;
    res.a1_opt       = a1_curr;
    res.a2_opt       = 1.0 - a1_curr;
    res.OP_avg_opt   = mean(OP_slots);
    res.OP_slots_opt = OP_slots;
    res.OP_D1_slots  = OP_D1_slots;
    res.OP_D2_slots  = OP_D2_slots;
    res.history_F    = history_F;
    res.iterations   = iter;
end
