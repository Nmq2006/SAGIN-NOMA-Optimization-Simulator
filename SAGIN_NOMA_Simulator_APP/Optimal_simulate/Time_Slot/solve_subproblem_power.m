function [a1_opt_slots, cost_slots] = solve_subproblem_power(Q_traj, rho_val, params)
% SOLVE_SUBPROBLEM_POWER: Tối ưu phân bổ công suất NOMA a1[n] tại mỗi khe thời gian n
%
% Khi chuỗi vị trí Q_traj = [q[1], ..., q[N]] đã cố định, bài toán tối ưu
% công suất NOMA trên toàn chu kỳ bay được phân rã thành N bài toán con độc lập.
%
% Inputs:
%   Q_traj: Ma trận tọa độ UAV (N x 2), hàng n là [x_n, y_n]
%   rho_val: Mức SNR phát tuyến tính
%   params: Cấu trúc tham số hệ thống
%
% Outputs:
%   a1_opt_slots: Vector (N x 1) công suất a1 tối ưu tại từng khe
%   cost_slots: Vector (N x 1) giá trị xác suất dừng OP_total tại từng khe

    N = params.N_slots;
    a1_opt_slots = zeros(N, 1);
    cost_slots   = zeros(N, 1);

    % Ngưỡng cận dưới khả thi cho a1 (để đảm bảo SIC feasibility)
    a1_min = params.gamma_th1 / (1.0 + params.gamma_th1);
    lb_a1  = a1_min + 1e-4;
    ub_a1  = 0.999;

    opts = optimset('Display', 'off', 'TolX', 1e-4);

    for n = 1:N
        q_n = Q_traj(n, :);

        % Hàm mục tiêu tại khe n theo biến alpha1
        slot_obj = @(a1) calculate_outage_slot(a1, q_n, rho_val, params);

        if exist('fminbnd', 'file') == 2 || exist('fminbnd', 'builtin') == 5
            [a1_best, min_cost] = fminbnd(slot_obj, lb_a1, ub_a1, opts);
        else
            % Fallback: Tìm kiếm lưới mịn nếu môi trường thiếu hàm fminbnd
            grid_a1 = linspace(lb_a1, ub_a1, 120);
            grid_vals = arrayfun(slot_obj, grid_a1);
            [min_cost, idx_best] = min(grid_vals);
            a1_best = grid_a1(idx_best);
        end

        a1_opt_slots(n) = a1_best;
        cost_slots(n)   = min_cost;
    end
end
