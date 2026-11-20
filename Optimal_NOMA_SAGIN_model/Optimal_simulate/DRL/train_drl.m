function [agent, train_history, eval_res] = train_drl(rho_val, params, num_episodes)
% TRAIN_DRL: Huấn luyện Agent DRL với Warm-Start Experience Seeding (v1 - Updated)
%
% Cải tiến v1:
%   1. Warm-Start Buffer Seeding: Khởi tạo Replay Buffer bằng 25 Episode mẫu quỹ đạo
%      danh định để Actor và Critic có điểm tựa vững chắc từ Episode 1, triệt tiêu
%      hoàn toàn hiện tượng Agent bị bão hòa hoặc mò mẫm không lối thoát.
%   2. Đánh giá tất định chính xác và lưu Checkpoint dựa trên xác suất dừng nhỏ nhất.

    if nargin < 3 || isempty(num_episodes)
        num_episodes = 260;
    end

    state_dim  = 8;
    action_dim = 3;

    % Khởi tạo Agent DRL
    agent = drl_agent(state_dim, action_dim, params);
    x_sic_bound = 0.5 * (params.pos_D1(1) + params.pos_D2(1)) - params.SIC_margin;

    % -------------------------------------------------------------------------
    % BƯỚC 1: WARM-START BUFFER SEEDING (Mồi 30 Episode kinh nghiệm danh định)
    % -------------------------------------------------------------------------
    num_seed_episodes = 30;
    fprintf('--> Đang khởi tạo Replay Buffer với %d Episode kinh nghiệm danh định (Warm-Start)...\n', num_seed_episodes);
    for seed_ep = 1:num_seed_episodes
        q_curr = params.q_start;
        for n = 1:params.N_slots
            % Hành động danh định với nhiễu nhẹ quanh mức công suất cân bằng (a1 ~ 0.74 - 0.77)
            nom_a_norm = 0.40 + 0.20 * rand(); % norm_a1 in [0.4, 0.6] => a1 in [0.748, 0.772]
            action_seed = [0.15 * randn(), -0.15 * rand(), nom_a_norm];
            
            % Vector trạng thái hiện tại
            [~, ~, ~, d_SU, d_UD1, d_UD2] = calculate_channel_params(q_curr, params);
            d_end = norm(q_curr - params.q_end);
            s = zeros(state_dim, 1);
            s(1) = q_curr(1) / params.uav_ub(1);
            s(2) = q_curr(2) / params.uav_ub(2);
            s(3) = min(1.0, d_SU  / 2.0);
            s(4) = min(1.0, d_UD1 / 2.0);
            s(5) = min(1.0, d_UD2 / 2.0);
            s(6) = min(1.0, d_end / 2.0);
            s(7) = max(0.0, (x_sic_bound - q_curr(1)) / 1.0);
            s(8) = (params.N_slots - n + 1) / params.N_slots;
            
            [s_next, r, done, q_next, ~] = step_env(action_seed, q_curr, n, rho_val, params);
            agent.store_transition(s, action_seed(:), r, s_next, done);
            q_curr = q_next;
        end
    end
    fprintf('--> Khởi tạo Buffer hoàn tất (%d mẫu chuyển trạng thái)!\n\n', agent.buffer_count);

    % -------------------------------------------------------------------------
    % BƯỚC 2: TIẾN TRÌNH HUẤN LUYỆN CHÍNH
    % -------------------------------------------------------------------------
    ep_rewards = zeros(num_episodes, 1);
    ep_avg_ops = zeros(num_episodes, 1);

    fprintf('========================================================================================\n');
    fprintf('  TIẾN TRÌNH HUẤN LUYỆN DRL (ACTOR-CRITIC) TRÊN MÔ HÌNH 2D Oxz (SNR = %.1f dB)\n', 10*log10(rho_val));
    fprintf('========================================================================================\n');
    fprintf(' EPISODE | TỔNG PHẦN THƯỞNG | OP TRUNG BÌNH | TỌA ĐỘ CUỐI q_N  | NOISE STD | ĐÁNH GIÁ CHÍNH SÁCH\n');
    fprintf('----------------------------------------------------------------------------------------\n');

    % Đánh giá ban đầu từ Buffer Seeding
    init_eval = evaluate_policy(agent, rho_val, params, false);
    agent.save_checkpoint(init_eval.total_reward, init_eval.OP_avg_opt);

    for ep = 1:num_episodes
        q_curr = params.q_start;
        ep_reward = 0;
        ep_ops    = zeros(params.N_slots, 1);

        [~, ~, ~, d_SU, d_UD1, d_UD2] = calculate_channel_params(q_curr, params);
        d_end = norm(q_curr - params.q_end);

        state = zeros(state_dim, 1);
        state(1) = q_curr(1) / params.uav_ub(1);
        state(2) = q_curr(2) / params.uav_ub(2);
        state(3) = min(1.0, d_SU  / 2.0);
        state(4) = min(1.0, d_UD1 / 2.0);
        state(5) = min(1.0, d_UD2 / 2.0);
        state(6) = min(1.0, d_end / 2.0);
        state(7) = max(0.0, (x_sic_bound - q_curr(1)) / 1.0);
        state(8) = 1.0;

        for n = 1:params.N_slots
            action = agent.select_action(state, true, false);

            [next_state, reward, done, q_next, OP_info] = step_env(action, q_curr, n, rho_val, params);

            agent.store_transition(state, action, reward, next_state, done);

            % Huấn luyện 2 bước gradient mỗi slot để đẩy nhanh hội tụ
            agent.train_step();
            agent.train_step();

            ep_reward = ep_reward + reward;
            ep_ops(n) = OP_info.OP_total;

            state  = next_state;
            q_curr = q_next;
        end

        agent.decay_noise();

        ep_rewards(ep) = ep_reward;
        ep_avg_ops(ep) = mean(ep_ops);

        % Đánh giá tất định định kỳ
        is_eval_ep = (mod(ep, 10) == 0) || (ep == num_episodes);
        eval_tag = '';

        if is_eval_ep
            eval_out = evaluate_policy(agent, rho_val, params, false);
            agent.save_checkpoint(eval_out.total_reward, eval_out.OP_avg_opt);
            eval_tag = sprintf('Eval OP = %.4e (Best: %.4e)', eval_out.OP_avg_opt, agent.best_op);
        end

        if mod(ep, 20) == 0 || ep == 1 || ep == num_episodes
            fprintf(' Ep %4d | %16.3f | %13.5e | [%.2f, %.2f]     | %9.3f | %s\n', ...
                ep, ep_reward, ep_avg_ops(ep), q_curr(1), q_curr(2), agent.noise_std, eval_tag);
        end
    end
    fprintf('----------------------------------------------------------------------------------------\n');
    fprintf('Quá trình huấn luyện DRL hoàn tất thành công!\n\n');

    train_history.episodes   = (1:num_episodes)';
    train_history.rewards    = ep_rewards;
    train_history.avg_ops    = ep_avg_ops;

    % Kết quả đánh giá trên Checkpoint tối ưu nhất
    eval_res = evaluate_policy(agent, rho_val, params, true);
end
