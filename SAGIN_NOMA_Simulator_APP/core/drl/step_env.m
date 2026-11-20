function [next_state, reward, done, q_next, OP_info] = step_env(action, q_curr, slot_idx, rho_val, params)
% STEP_ENV: Mô phỏng bước chuyển trạng thái môi trường mạng SAGIN-NOMA (v1 - Updated)
%
% Cải tiến v1:
%   1. Điều hướng định hướng bước bay (Guided Displacement): Đảm bảo UAV luôn tiến về q_end
%      và đến đích chính xác tại khe n = N, giải quyết dứt điểm lỗi "chết đứng giữa đường".
%   2. Ràng buộc công suất NOMA a1 in [0.70, 0.82]: Đảm bảo luôn thỏa mãn điều kiện SIC
%      (a1 > 0.670) và bảo vệ Near-user D2 không bị cạn kiệt công suất (a2 >= 0.18).
%   3. Hàm thưởng nhạy bén dạng Logarit: Thưởng phạt tỷ lệ thuận với cấp độ suy giảm Outage.
%
% Inputs:
%   action   : Vector hành động [delta_x, delta_z, norm_a1] in [-1, 1] x [-1, 1] x [0, 1]
%   q_curr   : Tọa độ hiện tại [x, z] của UAV
%   slot_idx : Chỉ số khe thời gian hiện tại (1 <= slot_idx <= N_slots)
%   rho_val  : Mức SNR phát tuyến tính
%   params   : Cấu trúc tham số hệ thống
%
% Outputs:
%   next_state : Vector trạng thái kế tiếp (8 x 1)
%   reward     : Phần thưởng tại bước thời gian hiện tại
%   done       : Cờ báo hiệu kết thúc Episode (slot_idx == N_slots)
%   q_next     : Tọa độ mới [x, z] của UAV
%   OP_info    : Cấu trúc chứa OP_total, OP_D1, OP_D2

    N = params.N_slots;
    done = (slot_idx >= N);

    % 1. Tính toán bước dịch chuyển định hướng tới điểm đích q_end
    rem_slots = max(1, N - slot_idx + 1);
    u_nom = (params.q_end - q_curr) / rem_slots; % Vector dịch chuyển danh định cần thiết

    % Độ lệch không gian do Agent Actor quyết định (tinh chỉnh tối ưu hóa quỹ đạo)
    delta_step = [action(1), action(2)] * 0.045;
    total_step = u_nom + delta_step;

    % Ràng buộc chặt chẽ vận tốc tối đa của UAV: ||total_step|| <= V_max * dt
    max_step_dist = params.V_max * params.dt; % 0.24
    step_norm = norm(total_step);
    if step_norm > max_step_dist
        total_step = total_step * (max_step_dist / step_norm);
    end

    % 2. Xác định vị trí mới và kiểm tra ranh giới
    if done
        q_next = params.q_end; % Đảm bảo kết thúc chính xác tại đích
    else
        q_raw = q_curr + total_step;
        x_sic_bound = 0.5 * (params.pos_D1(1) + params.pos_D2(1)) - params.SIC_margin; % 1.21
        x_max_allow = min(params.uav_ub(1), x_sic_bound);

        q_next = zeros(1, 2);
        q_next(1) = max(params.uav_lb(1), min(x_max_allow, q_raw(1)));
        q_next(2) = max(params.uav_lb(2), min(params.uav_ub(2), q_raw(2)));
    end

    % 3. Phân bổ công suất NOMA trong dải an toàn khả thi [0.70, 0.82]
    norm_a1 = max(0.0, min(1.0, action(3)));
    a1 = 0.70 + norm_a1 * 0.12; % a1 in [0.70, 0.82] => a2 in [0.18, 0.30]

    % 4. Tính toán xác suất dừng tại vị trí mới q_next
    [OP_total, OP_D1, OP_D2] = calculate_outage_slot(a1, q_next, rho_val, params);

    OP_info.OP_total = OP_total;
    OP_info.OP_D1    = OP_D1;
    OP_info.OP_D2    = OP_D2;

    % 5. Hàm phần thưởng Logarit nhạy bén (Reward Shaping)
    % Thưởng lớn khi Outage giảm theo cấp số nhân (orders of magnitude)
    r_op = - log10(OP_total + 1e-6) * 2.5;

    % Phạt khoảng cách tới điểm đích để khuyến khích hành trình đúng hướng
    d_end_next = norm(q_next - params.q_end);
    r_dist = - 3.0 * d_end_next;

    % Thưởng độ mượt của quỹ đạo (Smoothness penalty)
    r_smooth = - 2.0 * norm(delta_step);

    reward = r_op + r_dist + r_smooth;

    % Thưởng lớn tại khe cuối cùng khi về đích hoàn hảo
    if done
        reward = reward + 6.0;
    end

    % 6. Xây dựng vector trạng thái kế tiếp (8 chiều, chuẩn hóa không thứ nguyên)
    x_sic_bound = 0.5 * (params.pos_D1(1) + params.pos_D2(1)) - params.SIC_margin;
    [~, ~, ~, d_SU, d_UD1, d_UD2] = calculate_channel_params(q_next, params);
    next_slot = min(N, slot_idx + 1);

    next_state = zeros(8, 1);
    next_state(1) = q_next(1) / params.uav_ub(1);               % x chuẩn hóa
    next_state(2) = q_next(2) / params.uav_ub(2);               % z chuẩn hóa
    next_state(3) = min(1.0, d_SU  / 2.0);                      % khoảng cách tới S
    next_state(4) = min(1.0, d_UD1 / 2.0);                      % khoảng cách tới D1
    next_state(5) = min(1.0, d_UD2 / 2.0);                      % khoảng cách tới D2
    next_state(6) = min(1.0, d_end_next / 2.0);                 % khoảng cách tới đích
    next_state(7) = max(0.0, (x_sic_bound - q_next(1)) / 1.0);  % biên an toàn SIC
    next_state(8) = (N - next_slot) / N;                        % thời gian còn lại
end
