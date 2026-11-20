function [Q_opt, cost_total_opt] = solve_subproblem_trajectory(a1_slots, Q_init, rho_val, params)
% SOLVE_SUBPROBLEM_TRAJECTORY: Tối ưu hóa chuỗi vị trí quỹ đạo UAV trong mặt phẳng 2D (Oxz)
%
% Khi hệ số công suất a1_slots = [a1[1], ..., a1[N]] đã cố định,
% tối ưu hóa vị trí của UAV tại toàn bộ N khe thời gian trong mặt phẳng Oxz để cực tiểu hóa
% xác suất dừng trung bình, thỏa mãn:
%   1. Ranh giới hình học cố định vị thế giải mã SIC (User Ordering Constraints)
%   2. Ràng buộc vận tốc tối đa giữa 2 khe liên tiếp (V_max * dt)
%   3. Tọa độ điểm đầu (q_start) và điểm cuối (q_end)
%   4. Hành lang bay cho phép [X_min, X_max] x [Z_min, Z_max]
%
% Inputs:
%   a1_slots: Vector (N x 1) hệ số công suất tại từng khe
%   Q_init: Ma trận quỹ đạo khởi tạo (N x 2), mỗi hàng là [x_n, z_n]
%   rho_val: Mức SNR phát tuyến tính
%   params: Cấu trúc tham số hệ thống
%
% Outputs:
%   Q_opt: Ma trận quỹ đạo tối ưu (N x 2)
%   cost_total_opt: Giá trị hàm mục tiêu sau khi tối ưu

    N = params.N_slots;

    % Biến vector 1D: [x1; z1; x2; z2; ...; xN; zN] (Kích thước 2N x 1)
    q_init_vec = reshape(Q_init', 2 * N, 1);

    % Giới hạn hộp (Box bounds) [x, z] cho toàn bộ N khe dựa trên uav_lb, uav_ub
    lb = repmat([params.uav_lb(1); params.uav_lb(2)], N, 1);
    ub = repmat([params.uav_ub(1); params.uav_ub(2)], N, 1);

    % Cố định tọa độ điểm xuất phát (khe 1) và điểm đích (khe N)
    lb(1:2)       = params.q_start(:);
    ub(1:2)       = params.q_start(:);
    lb(end-1:end) = params.q_end(:);
    ub(end-1:end) = params.q_end(:);

    % Hàm mục tiêu tổng thể qua N khe thời gian
    obj_fun = @(q_vec) trajectory_objective(q_vec, a1_slots, rho_val, params);

    % Hàm ràng buộc phi tuyến (Ranh giới hình học SIC & Vận tốc bay)
    nonlcon = @(q_vec) check_geometric_constraints(q_vec, params);

    % Thực hiện giải bài toán tối ưu
    if exist('fmincon', 'file') == 2 || exist('fmincon', 'builtin') == 5
        opts = optimoptions('fmincon', ...
                            'Algorithm', 'sqp', ...
                            'Display', 'off', ...
                            'StepTolerance', 1e-4, ...
                            'OptimalityTolerance', 1e-4, ...
                            'ConstraintTolerance', 1e-4, ...
                            'MaxIterations', 250, ...
                            'MaxFunctionEvaluations', 6000);

        [q_opt_vec, ~] = fmincon(obj_fun, q_init_vec, [], [], [], [], lb, ub, nonlcon, opts);
    else
        % Fallback: Phương pháp hàm phạt (Penalty Function Method) nếu thiếu Optimization Toolbox
        penalty_weight = 1e5;
        penalized_obj = @(q_vec) penalty_objective(q_vec, obj_fun, nonlcon, lb, ub, penalty_weight);
        opts = optimset('Display', 'off', 'TolX', 1e-4, 'TolFun', 1e-4, 'MaxFunEvals', 5000);
        q_opt_vec = fminsearch(penalized_obj, q_init_vec, opts);
        % Chiếu lại vào miền ràng buộc hộp
        q_opt_vec = min(max(q_opt_vec, lb), ub);
    end

    % Tái tạo lại ma trận quỹ đạo (N x 2)
    Q_opt = reshape(q_opt_vec, 2, N)';
    cost_total_opt = trajectory_objective(q_opt_vec, a1_slots, rho_val, params);
end

% === HÀM NỘI BỘ TÍNH HÀM MỤC TIÊU Q_VEC ===
function total_cost = trajectory_objective(q_vec, a1_slots, rho_val, params)
    N = params.N_slots;
    Q = reshape(q_vec, 2, N)';
    sum_cost = 0;
    for n = 1:N
        [op_slot, ~, ~] = calculate_outage_slot(a1_slots(n), Q(n, :), rho_val, params);
        sum_cost = sum_cost + op_slot;
    end
    total_cost = sum_cost / N; % Trung bình cộng xác suất dừng qua N khe
end

% === HÀM PHẠT DỰ PHÒNG NẾU KHÔNG CÓ FMINCON ===
function cost = penalty_objective(q_vec, obj_fun, nonlcon, lb, ub, penalty_weight)
    base_cost = obj_fun(q_vec);
    [c, ~] = nonlcon(q_vec);

    % Vi phạm ràng buộc bất đẳng thức phi tuyến (c > 0)
    viol_c = sum(max(0, c).^2);

    % Vi phạm cận dưới và cận trên (lb, ub)
    viol_box = sum(max(0, lb - q_vec).^2) + sum(max(0, q_vec - ub).^2);

    cost = base_cost + penalty_weight * (viol_c + viol_box);
end
