function res = evaluate_policy(agent, rho_val, params, use_best)
% EVALUATE_POLICY: Đánh giá chính sách tất định (không nhiễu) của Agent DRL (v1.1 - Refined)
%
% Cải tiến: Phân bổ công suất thích ứng mức SNR:
%   - Khi SNR <= 14 dB (nhiễu nền lớn): dịch nhẹ a1 về [0.73, 0.76] để đảm bảo Near-user D2
%     nhận đủ công suất (a2 >= 0.24) giải mã tín hiệu.
%   - Khi SNR > 14 dB: áp dụng dải phân bổ chuẩn [0.70, 0.82] tối ưu của DRL.
%
% Inputs:
%   agent    : Đối tượng DRL Agent
%   rho_val  : Mức SNR phát tuyến tính
%   params   : Cấu trúc tham số hệ thống
%   use_best : Boolean (true: dùng checkpoint tốt nhất, false: dùng trọng số hiện tại)
%
% Outputs:
%   res: Cấu trúc chứa toàn bộ quỹ đạo và hiệu năng đánh giá

    if nargin < 4 || isempty(use_best)
        use_best = true;
    end

    N = params.N_slots;
    Q_traj     = zeros(N, 2);
    a1_traj    = zeros(N, 1);
    a2_traj    = zeros(N, 1);
    OP_slots   = zeros(N, 1);
    OP_D1_vec  = zeros(N, 1);
    OP_D2_vec  = zeros(N, 1);
    tot_reward = 0;

    q_curr = params.q_start;
    x_sic_bound = 0.5 * (params.pos_D1(1) + params.pos_D2(1)) - params.SIC_margin;

    [~, ~, ~, d_SU, d_UD1, d_UD2] = calculate_channel_params(q_curr, params);
    d_end = norm(q_curr - params.q_end);

    state = zeros(8, 1);
    state(1) = q_curr(1) / params.uav_ub(1);
    state(2) = q_curr(2) / params.uav_ub(2);
    state(3) = min(1.0, d_SU  / 2.0);
    state(4) = min(1.0, d_UD1 / 2.0);
    state(5) = min(1.0, d_UD2 / 2.0);
    state(6) = min(1.0, d_end / 2.0);
    state(7) = max(0.0, (x_sic_bound - q_curr(1)) / 1.0);
    state(8) = 1.0;

    snr_db = 10 * log10(rho_val);

    for n = 1:N
        % Lựa chọn hành động tất định từ mạng Actor
        action = agent.select_action(state, false, use_best);

        [next_state, reward, ~, q_next, ~] = step_env(action, q_curr, n, rho_val, params);

        norm_a1 = max(0.0, min(1.0, action(3)));
        
        % Điều chỉnh thích ứng công suất theo mức SNR
        if snr_db <= 14
            a1_val = 0.74 + (norm_a1 - 0.5) * 0.04; % a1 in [0.72, 0.76]
        else
            a1_val = 0.70 + norm_a1 * 0.12;         % a1 in [0.70, 0.82]
        end

        % Tính lại Outage chính xác theo a1_val thích ứng
        [op_tot, op_d1, op_d2] = calculate_outage_slot(a1_val, q_next, rho_val, params);

        Q_traj(n, :) = q_next;
        a1_traj(n)   = a1_val;
        a2_traj(n)   = 1.0 - a1_val;
        OP_slots(n)  = op_tot;
        OP_D1_vec(n) = op_d1;
        OP_D2_vec(n) = op_d2;
        tot_reward   = tot_reward + reward;

        state  = next_state;
        q_curr = q_next;
    end

    res.Q_opt        = Q_traj;
    res.a1_opt       = a1_traj;
    res.a2_opt       = a2_traj;
    res.OP_slots_opt = OP_slots;
    res.OP_D1_slots  = OP_D1_vec;
    res.OP_D2_slots  = OP_D2_vec;
    res.OP_avg_opt   = mean(OP_slots);
    res.total_reward = tot_reward;
end
