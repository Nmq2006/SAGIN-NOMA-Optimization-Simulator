function [c, ceq] = check_geometric_constraints(q_vec, params)
% CHECK_GEOMETRIC_CONSTRAINTS: Kiểm tra ràng buộc hình học SIC và vận tốc bay trong mặt phẳng 2D (Oxz)
%
% Inputs:
%   q_vec: Vector 1D chứa toàn bộ tọa độ UAV [x1, z1, x2, z2, ..., xN, zN] (2N x 1)
%   params: Cấu trúc chứa các tham số hệ thống
%
% Outputs:
%   c: Vector bất đẳng thức phi tuyến c <= 0
%      - Nhóm 1: Ràng buộc vận tốc tối đa giữa 2 khe liên tiếp
%      - Nhóm 2: Ranh giới hình học cố định vị thế giải mã SIC (User Ordering)
%   ceq: Vector đẳng thức phi tuyến ceq == 0 (rỗng)

    N = params.N_slots;
    Q = reshape(q_vec, 2, N)'; % Kích thước N x 2: Mỗi hàng n là tọa độ [x_n, z_n]

    max_dist_step = params.V_max * params.dt; % Giới hạn dịch chuyển tối đa mỗi khe

    c_speed = zeros(N - 1, 1);
    c_geo   = zeros(N, 1);

    pos_D1 = params.pos_D1; % [x, z] của D1 (1.5, 0.0)
    pos_D2 = params.pos_D2; % [x, z] của D2 (1.0, 0.0)

    for n = 1:N
        q_n = Q(n, :); % [x_n, z_n]

        % 1. Ràng buộc ranh giới hình học (User Ordering Constraint)
        % Đảm bảo khoảng cách từ UAV đến D2 luôn gần hơn đến D1 ít nhất 1 khoảng margin:
        % ||q[n] - pos_D2|| - ||q[n] - pos_D1|| + margin <= 0
        dist_to_D2 = norm(q_n - pos_D2);
        dist_to_D1 = norm(q_n - pos_D1);
        c_geo(n) = dist_to_D2 - dist_to_D1 + params.SIC_margin;

        % 2. Ràng buộc vận tốc bay giữa 2 khe liên tiếp: ||q[n] - q[n-1]|| <= V_max * dt
        if n >= 2
            q_prev = Q(n - 1, :);
            step_dist = norm(q_n - q_prev);
            c_speed(n - 1) = step_dist - max_dist_step;
        end
    end

    c = [c_speed; c_geo];
    ceq = [];
end
