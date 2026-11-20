function res = run_ao_solver(params, progressCallback, cancelCheck)
% RUN_AO_SOLVER: Ham Engine thuc thi toi uu hoa luan phien (AO) cho mang SAGIN-NOMA
% Duoc thiet ke doc lap voi giao dien: ho tro Callback cap nhat tien trinh va Kiem tra Huy (Cancel).
%
% Inputs:
%   params           : Struct chua toan bo tham so he thong (tu get_default_params)
%   progressCallback : (Tuy chon) Function handle dang @(ratio, msg) de gui tien trinh ve UI/Console
%   cancelCheck      : (Tuy chon) Function handle dang @() tra ve true neu nguoi dung muon dung
%
% Output:
%   res: Struct chua ket qua toi uu (Q_opt, a1_opt, a2_opt, OP_slots, history_F, ...)

    % 0. Kiem tra tham so dau vao
    if nargin < 2, progressCallback = []; end
    if nargin < 3, cancelCheck = []; end

    N = params.N_slots;
    rho_val = 10^(params.rho_dB / 10); % Chuyen doi dB sang tuyen tinh

    % 1. Khoi tao quy dao ban dau Q^(0) (Bay thang deu tu q_start den q_end)
    Q_curr = zeros(N, 2);
    for n = 1:N
        alpha = (n - 1) / (N - 1);
        Q_curr(n, :) = (1 - alpha) * params.q_start + alpha * params.q_end;
    end

    % 2. Danh gia ham muc tieu ban dau F(0)
    [a1_curr, init_costs] = solve_subproblem_power(Q_curr, rho_val, params);
    cost_curr = mean(init_costs);

    history_F = zeros(params.max_iterations + 1, 1);
    history_F(1) = cost_curr;

    converged = false;
    actual_iter = 0;

    % 3. Vong lap Toi uu hoa Luan phien (Alternating Optimization)
    for k = 1:params.max_iterations
        actual_iter = k;

        % Kiem tra nguoi dung co bam CANCEL tren giao dien hay khong
        if ~isempty(cancelCheck) && cancelCheck()
            fprintf('>> [Engine] Nguoi dung da huy qua trinh toi uu tai vong %d.\n', k);
            break;
        end

        % Buoc A: Toi uu Phan bo cong suat khi co dinh Quy dao
        [a1_curr, ~] = solve_subproblem_power(Q_curr, rho_val, params);

        % Buoc B: Toi uu Quy dao UAV khi co dinh Cong suat
        [Q_curr, cost_new] = solve_subproblem_trajectory(a1_curr, Q_curr, rho_val, params);

        % Tinh sai so tuong doi va luu lich su hoi tu
        rel_err = abs(cost_new - cost_curr) / (abs(cost_curr) + 1e-8);
        history_F(k + 1) = cost_new;

        % Gui thong tin tien do ve UI / Console thong qua callback
        if ~isempty(progressCallback)
            ratio = k / params.max_iterations;
            msg = sprintf('Vong lap %d/%d | OP TB: %.3e | Sai so: %.2e', ...
                          k, params.max_iterations, cost_new, rel_err);
            progressCallback(ratio, msg);
        end

        cost_curr = cost_new;

        % Kiem tra dieu kien dung hoi tu
        if rel_err <= params.epsilon
            converged = true;
            history_F = history_F(1:(k + 1));
            break;
        end
    end

    if ~converged && actual_iter == params.max_iterations
        history_F = history_F(1:(actual_iter + 1));
    end

    % 4. Tinh toan xac suat dung chi tiet tai nghiem toi uu
    OP_slots    = zeros(N, 1);
    OP_D1_slots = zeros(N, 1);
    OP_D2_slots = zeros(N, 1);
    for n = 1:N
        [OP_slots(n), OP_D1_slots(n), OP_D2_slots(n)] = ...
            calculate_outage_slot(a1_curr(n), Q_curr(n, :), rho_val, params);
    end

    % 5. Dong goi toan bo ket qua chuan hoa dau ra
    res.Q_opt        = Q_curr;
    res.a1_opt       = a1_curr;
    res.a2_opt       = 1.0 - a1_curr;
    res.OP_avg_opt   = mean(OP_slots);
    res.OP_slots_opt = OP_slots;
    res.OP_D1_slots  = OP_D1_slots;
    res.OP_D2_slots  = OP_D2_slots;
    res.history_F    = history_F;
    res.iterations   = actual_iter;
    res.converged    = converged;
    res.params       = params;
end
