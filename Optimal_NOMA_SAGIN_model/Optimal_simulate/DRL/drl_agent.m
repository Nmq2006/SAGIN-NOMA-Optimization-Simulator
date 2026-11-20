classdef drl_agent < handle
% DRL_AGENT: Lớp triển khai thuật toán Actor-Critic DRL (v1 - Updated)
% Triển khai thuần túy MATLAB với Adam Optimizer, Experience Replay Buffer,
% Soft Update và Warm-Start Guidance.

    properties
        state_dim       % Số chiều trạng thái (8)
        action_dim      % Số chiều hành động (3: delta_x, delta_z, norm_a1)
        
        % Siêu tham số DRL
        gamma           % Hệ số chiết khấu phần thưởng (0.95)
        tau             % Tốc độ cập nhật mềm mạng Target (0.01)
        lr_actor        % Tốc độ học của Actor
        lr_critic       % Tốc độ học của Critic
        batch_size      % Kích thước mini-batch (32)
        
        % Trọng số mạng Actor (64 - 64)
        W_a1, b_a1
        W_a2, b_a2
        W_a3, b_a3
        
        % Trọng số mạng Critic (64 - 64)
        W_c1, b_c1
        W_c2, b_c2
        W_c3, b_c3
        
        % Trọng số mạng Target
        W_a1_tar, b_a1_tar, W_a2_tar, b_a2_tar, W_a3_tar, b_a3_tar
        W_c1_tar, b_c1_tar, W_c2_tar, b_c2_tar, W_c3_tar, b_c3_tar
        
        % Bộ tối ưu hóa Adam
        m_Wa1, v_Wa1, m_ba1, v_ba1
        m_Wa2, v_Wa2, m_ba2, v_ba2
        m_Wa3, v_Wa3, m_ba3, v_ba3
        
        m_Wc1, v_Wc1, m_bc1, v_bc1
        m_Wc2, v_Wc2, m_bc2, v_bc2
        m_Wc3, v_Wc3, m_bc3, v_bc3
        adam_t
        
        % Replay Buffer
        buffer_capacity
        buffer_ptr
        buffer_count
        buf_s
        buf_a
        buf_r
        buf_s_next
        buf_done
        
        % Khám phá (Exploration Noise)
        noise_std
        noise_decay
        min_noise
        
        % Checkpoint lưu trọng số tối ưu
        best_W_a1, best_b_a1, best_W_a2, best_b_a2, best_W_a3, best_b_a3
        best_reward
        best_op
    end
    
    methods
        function obj = drl_agent(state_dim, action_dim, sys_params)
            obj.state_dim   = state_dim;
            obj.action_dim  = action_dim;
            
            obj.gamma       = 0.95;
            obj.tau         = 0.008;
            obj.lr_actor    = 0.0005;
            obj.lr_critic   = 0.0010;
            obj.batch_size  = 64;
            
            obj.noise_std   = 0.20;
            obj.noise_decay = 0.985;
            obj.min_noise   = 0.015;
            obj.best_reward = -inf;
            obj.best_op     = inf;
            obj.adam_t      = 0;
            
            h_dim = 64;
            
            % Khởi tạo trọng số Actor
            obj.W_a1 = randn(h_dim, state_dim) * sqrt(2 / state_dim);
            obj.b_a1 = zeros(h_dim, 1);
            obj.W_a2 = randn(h_dim, h_dim) * sqrt(2 / h_dim);
            obj.b_a2 = zeros(h_dim, 1);
            obj.W_a3 = randn(action_dim, h_dim) * 0.03; % Khởi tạo nhỏ để bám sát quỹ đạo danh định ban đầu
            obj.b_a3 = zeros(action_dim, 1);
            
            obj.W_a1_tar = obj.W_a1; obj.b_a1_tar = obj.b_a1;
            obj.W_a2_tar = obj.W_a2; obj.b_a2_tar = obj.b_a2;
            obj.W_a3_tar = obj.W_a3; obj.b_a3_tar = obj.b_a3;
            
            % Khởi tạo trọng số Critic
            in_c = state_dim + action_dim;
            obj.W_c1 = randn(h_dim, in_c) * sqrt(2 / in_c);
            obj.b_c1 = zeros(h_dim, 1);
            obj.W_c2 = randn(h_dim, h_dim) * sqrt(2 / h_dim);
            obj.b_c2 = zeros(h_dim, 1);
            obj.W_c3 = randn(1, h_dim) * 0.03;
            obj.b_c3 = 0;
            
            obj.W_c1_tar = obj.W_c1; obj.b_c1_tar = obj.b_c1;
            obj.W_c2_tar = obj.W_c2; obj.b_c2_tar = obj.b_c2;
            obj.W_c3_tar = obj.W_c3; obj.b_c3_tar = obj.b_c3;
            
            % Khởi tạo Adam
            obj.m_Wa1 = zeros(size(obj.W_a1)); obj.v_Wa1 = zeros(size(obj.W_a1));
            obj.m_ba1 = zeros(size(obj.b_a1)); obj.v_ba1 = zeros(size(obj.b_a1));
            obj.m_Wa2 = zeros(size(obj.W_a2)); obj.v_Wa2 = zeros(size(obj.W_a2));
            obj.m_ba2 = zeros(size(obj.b_a2)); obj.v_ba2 = zeros(size(obj.b_a2));
            obj.m_Wa3 = zeros(size(obj.W_a3)); obj.v_Wa3 = zeros(size(obj.W_a3));
            obj.m_ba3 = zeros(size(obj.b_a3)); obj.v_ba3 = zeros(size(obj.b_a3));
            
            obj.m_Wc1 = zeros(size(obj.W_c1)); obj.v_Wc1 = zeros(size(obj.W_c1));
            obj.m_bc1 = zeros(size(obj.b_c1)); obj.v_bc1 = zeros(size(obj.b_c1));
            obj.m_Wc2 = zeros(size(obj.W_c2)); obj.v_Wc2 = zeros(size(obj.W_c2));
            obj.m_bc2 = zeros(size(obj.b_c2)); obj.v_bc2 = zeros(size(obj.b_c2));
            obj.m_Wc3 = zeros(size(obj.W_c3)); obj.v_Wc3 = zeros(size(obj.W_c3));
            obj.m_bc3 = zeros(size(obj.b_c3)); obj.v_bc3 = zeros(size(obj.b_c3));
            
            % Replay Buffer
            obj.buffer_capacity = 6000;
            obj.buffer_ptr      = 1;
            obj.buffer_count    = 0;
            obj.buf_s           = zeros(state_dim, obj.buffer_capacity);
            obj.buf_a           = zeros(action_dim, obj.buffer_capacity);
            obj.buf_r           = zeros(1, obj.buffer_capacity);
            obj.buf_s_next      = zeros(state_dim, obj.buffer_capacity);
            obj.buf_done        = zeros(1, obj.buffer_capacity);
        end
        
        function action = select_action(obj, state, explore, use_best)
            if nargin < 3; explore  = false; end
            if nargin < 4; use_best = false; end
            
            if use_best && ~isempty(obj.best_W_a1)
                Wa1 = obj.best_W_a1; ba1 = obj.best_b_a1;
                Wa2 = obj.best_W_a2; ba2 = obj.best_b_a2;
                Wa3 = obj.best_W_a3; ba3 = obj.best_b_a3;
            else
                Wa1 = obj.W_a1; ba1 = obj.b_a1;
                Wa2 = obj.W_a2; ba2 = obj.b_a2;
                Wa3 = obj.W_a3; ba3 = obj.b_a3;
            end
            
            raw = obj.forward_actor_net(state, Wa1, ba1, Wa2, ba2, Wa3, ba3);
            
            % Ánh xạ kích hoạt vật lý:
            % raw(1) in [-1, 1] qua tanh: delta_x
            % raw(2) in [-1, 1] qua tanh: delta_z
            % raw(3) in [0, 1] qua sigmoid: norm_a1
            act1 = tanh(raw(1));
            act2 = tanh(raw(2));
            act3 = 1 / (1 + exp(-raw(3)));
            action = [act1; act2; act3];
            
            if explore
                noise = randn(3, 1) .* [0.20; 0.20; 0.15] * obj.noise_std;
                action = action + noise;
                action(1) = max(-1.0, min(1.0, action(1)));
                action(2) = max(-1.0, min(1.0, action(2)));
                action(3) = max(0.0,  min(1.0, action(3)));
            end
        end
        
        function raw_act = forward_actor_net(~, s, W1, b1, W2, b2, W3, b3)
            z1 = W1 * s + b1;
            h1 = max(z1, 0);
            z2 = W2 * h1 + b2;
            h2 = max(z2, 0);
            raw_act = W3 * h2 + b3;
        end
        
        function q_val = forward_critic_net(~, s, a, W1, b1, W2, b2, W3, b3)
            x  = [s; a];
            z1 = W1 * x + b1;
            h1 = max(z1, 0);
            z2 = W2 * h1 + b2;
            h2 = max(z2, 0);
            q_val = W3 * h2 + b3;
        end
        
        function store_transition(obj, s, a, r, s_next, done)
            idx = obj.buffer_ptr;
            obj.buf_s(:, idx)      = s;
            obj.buf_a(:, idx)      = a;
            obj.buf_r(idx)         = r;
            obj.buf_s_next(:, idx) = s_next;
            obj.buf_done(idx)      = done;
            
            obj.buffer_ptr = mod(obj.buffer_ptr, obj.buffer_capacity) + 1;
            if obj.buffer_count < obj.buffer_capacity
                obj.buffer_count = obj.buffer_count + 1;
            end
        end
        
        function [critic_loss, actor_loss] = train_step(obj)
            critic_loss = 0; actor_loss = 0;
            if obj.buffer_count < obj.batch_size
                return;
            end
            
            obj.adam_t = obj.adam_t + 1;
            idx = randperm(obj.buffer_count, obj.batch_size);
            
            S      = obj.buf_s(:, idx);
            A      = obj.buf_a(:, idx);
            R      = obj.buf_r(idx);
            S_next = obj.buf_s_next(:, idx);
            Done   = obj.buf_done(idx);
            
            % 1. Target Actions A_tar
            A_next = zeros(obj.action_dim, obj.batch_size);
            for b = 1:obj.batch_size
                raw_tar = obj.forward_actor_net(S_next(:, b), ...
                    obj.W_a1_tar, obj.b_a1_tar, obj.W_a2_tar, obj.b_a2_tar, obj.W_a3_tar, obj.b_a3_tar);
                s3 = 1 / (1 + exp(-raw_tar(3)));
                A_next(:, b) = [tanh(raw_tar(1)); tanh(raw_tar(2)); s3];
            end
            
            % 2. Target Q-values
            y_target = zeros(1, obj.batch_size);
            for b = 1:obj.batch_size
                q_tar = obj.forward_critic_net(S_next(:, b), A_next(:, b), ...
                    obj.W_c1_tar, obj.b_c1_tar, obj.W_c2_tar, obj.b_c2_tar, obj.W_c3_tar, obj.b_c3_tar);
                y_target(b) = R(b) + obj.gamma * q_tar * (1 - Done(b));
            end
            
            % 3. Cập nhật Critic
            X_c = [S; A];
            Z_c1 = obj.W_c1 * X_c + repmat(obj.b_c1, 1, obj.batch_size);
            H_c1 = max(Z_c1, 0);
            Z_c2 = obj.W_c2 * H_c1 + repmat(obj.b_c2, 1, obj.batch_size);
            H_c2 = max(Z_c2, 0);
            Q_pred = obj.W_c3 * H_c2 + obj.b_c3;
            
            err = Q_pred - y_target;
            critic_loss = mean(err.^2);
            
            d_out = 2 * err / obj.batch_size;
            dW_c3 = d_out * H_c2';
            db_c3 = sum(d_out);
            
            dH_c2 = obj.W_c3' * d_out;
            dZ_c2 = dH_c2 .* (Z_c2 > 0);
            dW_c2 = dZ_c2 * H_c1';
            db_c2 = sum(dZ_c2, 2);
            
            dH_c1 = obj.W_c2' * dZ_c2;
            dZ_c1 = dH_c1 .* (Z_c1 > 0);
            dW_c1 = dZ_c1 * X_c';
            db_c1 = sum(dZ_c1, 2);
            
            [obj.W_c3, obj.m_Wc3, obj.v_Wc3] = obj.adam_update(obj.W_c3, dW_c3, obj.m_Wc3, obj.v_Wc3, obj.lr_critic);
            [obj.b_c3, obj.m_bc3, obj.v_bc3] = obj.adam_update(obj.b_c3, db_c3, obj.m_bc3, obj.v_bc3, obj.lr_critic);
            [obj.W_c2, obj.m_Wc2, obj.v_Wc2] = obj.adam_update(obj.W_c2, dW_c2, obj.m_Wc2, obj.v_Wc2, obj.lr_critic);
            [obj.b_c2, obj.m_bc2, obj.v_bc2] = obj.adam_update(obj.b_c2, db_c2, obj.m_bc2, obj.v_bc2, obj.lr_critic);
            [obj.W_c1, obj.m_Wc1, obj.v_Wc1] = obj.adam_update(obj.W_c1, dW_c1, obj.m_Wc1, obj.v_Wc1, obj.lr_critic);
            [obj.b_c1, obj.m_bc1, obj.v_bc1] = obj.adam_update(obj.b_c1, db_c1, obj.m_bc1, obj.v_bc1, obj.lr_critic);
            
            % 4. Cập nhật Actor
            Z_a1 = obj.W_a1 * S + repmat(obj.b_a1, 1, obj.batch_size);
            H_a1 = max(Z_a1, 0);
            Z_a2 = obj.W_a2 * H_a1 + repmat(obj.b_a2, 1, obj.batch_size);
            H_a2 = max(Z_a2, 0);
            Raw_A = obj.W_a3 * H_a2 + repmat(obj.b_a3, 1, obj.batch_size);
            
            A_cur = zeros(obj.action_dim, obj.batch_size);
            dAct_dRaw = zeros(obj.action_dim, obj.batch_size);
            for b = 1:obj.batch_size
                t1 = tanh(Raw_A(1, b));
                t2 = tanh(Raw_A(2, b));
                s3 = 1 / (1 + exp(-Raw_A(3, b)));
                
                A_cur(:, b) = [t1; t2; s3];
                dAct_dRaw(1, b) = 1 - t1^2;
                dAct_dRaw(2, b) = 1 - t2^2;
                dAct_dRaw(3, b) = s3 * (1 - s3);
            end
            
            X_c_cur = [S; A_cur];
            Z_c1_cur = obj.W_c1 * X_c_cur + repmat(obj.b_c1, 1, obj.batch_size);
            H_c1_cur = max(Z_c1_cur, 0);
            Z_c2_cur = obj.W_c2 * H_c1_cur + repmat(obj.b_c2, 1, obj.batch_size);
            H_c2_cur = max(Z_c2_cur, 0);
            
            dH_c2_cur = repmat(obj.W_c3', 1, obj.batch_size);
            dZ_c2_cur = dH_c2_cur .* (Z_c2_cur > 0);
            dH_c1_cur = obj.W_c2' * dZ_c2_cur;
            dZ_c1_cur = dH_c1_cur .* (Z_c1_cur > 0);
            dX_c_cur  = obj.W_c1' * dZ_c1_cur;
            
            dQ_dA = dX_c_cur((obj.state_dim + 1):end, :);
            
            dLoss_dRaw = - (dQ_dA .* dAct_dRaw) / obj.batch_size;
            actor_loss = - mean(obj.W_c3 * H_c2_cur + obj.b_c3);
            
            dW_a3 = dLoss_dRaw * H_a2';
            db_a3 = sum(dLoss_dRaw, 2);
            
            dH_a2 = obj.W_a3' * dLoss_dRaw;
            dZ_a2 = dH_a2 .* (Z_a2 > 0);
            dW_a2 = dZ_a2 * H_a1';
            db_a2 = sum(dZ_a2, 2);
            
            dH_a1 = obj.W_a2' * dZ_a2;
            dZ_a1 = dH_a1 .* (Z_a1 > 0);
            dW_a1 = dZ_a1 * S';
            db_a1 = sum(dZ_a1, 2);
            
            [obj.W_a3, obj.m_Wa3, obj.v_Wa3] = obj.adam_update(obj.W_a3, dW_a3, obj.m_Wa3, obj.v_Wa3, obj.lr_actor);
            [obj.b_a3, obj.m_ba3, obj.v_ba3] = obj.adam_update(obj.b_a3, db_a3, obj.m_ba3, obj.v_ba3, obj.lr_actor);
            [obj.W_a2, obj.m_Wa2, obj.v_Wa2] = obj.adam_update(obj.W_a2, dW_a2, obj.m_Wa2, obj.v_Wa2, obj.lr_actor);
            [obj.b_a2, obj.m_ba2, obj.v_ba2] = obj.adam_update(obj.b_a2, db_a2, obj.m_ba2, obj.v_ba2, obj.lr_actor);
            [obj.W_a1, obj.m_Wa1, obj.v_Wa1] = obj.adam_update(obj.W_a1, dW_a1, obj.m_Wa1, obj.v_Wa1, obj.lr_actor);
            [obj.b_a1, obj.m_ba1, obj.v_ba1] = obj.adam_update(obj.b_a1, db_a1, obj.m_ba1, obj.v_ba1, obj.lr_actor);
            
            % 5. Cập nhật mềm Target Networks
            obj.W_c1_tar = obj.tau * obj.W_c1 + (1 - obj.tau) * obj.W_c1_tar;
            obj.b_c1_tar = obj.tau * obj.b_c1 + (1 - obj.tau) * obj.b_c1_tar;
            obj.W_c2_tar = obj.tau * obj.W_c2 + (1 - obj.tau) * obj.W_c2_tar;
            obj.b_c2_tar = obj.tau * obj.b_c2 + (1 - obj.tau) * obj.b_c2_tar;
            obj.W_c3_tar = obj.tau * obj.W_c3 + (1 - obj.tau) * obj.W_c3_tar;
            obj.b_c3_tar = obj.tau * obj.b_c3 + (1 - obj.tau) * obj.b_c3_tar;
            
            obj.W_a1_tar = obj.tau * obj.W_a1 + (1 - obj.tau) * obj.W_a1_tar;
            obj.b_a1_tar = obj.tau * obj.b_a1 + (1 - obj.tau) * obj.b_a1_tar;
            obj.W_a2_tar = obj.tau * obj.W_a2 + (1 - obj.tau) * obj.W_a2_tar;
            obj.b_a2_tar = obj.tau * obj.b_a2 + (1 - obj.tau) * obj.b_a2_tar;
            obj.W_a3_tar = obj.tau * obj.W_a3 + (1 - obj.tau) * obj.W_a3_tar;
            obj.b_a3_tar = obj.tau * obj.b_a3 + (1 - obj.tau) * obj.b_a3_tar;
        end
        
        function [param, m, v] = adam_update(obj, param, grad, m, v, lr)
            beta1 = 0.9; beta2 = 0.999; eps_val = 1e-8;
            gnorm = norm(grad(:));
            if gnorm > 5.0
                grad = grad * (5.0 / gnorm);
            end
            m = beta1 * m + (1 - beta1) * grad;
            v = beta2 * v + (1 - beta2) * (grad.^2);
            m_hat = m / (1 - beta1^obj.adam_t);
            v_hat = v / (1 - beta2^obj.adam_t);
            param = param - lr * m_hat ./ (sqrt(v_hat) + eps_val);
        end
        
        function decay_noise(obj)
            obj.noise_std = max(obj.min_noise, obj.noise_std * obj.noise_decay);
        end
        
        function save_checkpoint(obj, eval_reward, eval_op)
            % Tiêu chí lưu: Outage nhỏ hơn và phần thưởng tốt hơn
            if eval_op < obj.best_op || (eval_op <= obj.best_op * 1.02 && eval_reward > obj.best_reward)
                obj.best_op     = eval_op;
                obj.best_reward = eval_reward;
                obj.best_W_a1 = obj.W_a1; obj.best_b_a1 = obj.b_a1;
                obj.best_W_a2 = obj.W_a2; obj.best_b_a2 = obj.b_a2;
                obj.best_W_a3 = obj.W_a3; obj.best_b_a3 = obj.b_a3;
            end
        end
    end
end
