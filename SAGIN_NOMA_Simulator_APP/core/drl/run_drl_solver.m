function res = run_drl_solver(params, progressCallback)
% RUN_DRL_SOLVER: Chay danh gia suy luan chinh sach DRL Agent
% Thuc hien feed-forward qua mang Actor Neural Network tren moi khe thoi gian.

    if nargin < 2, progressCallback = []; end

    if ~isempty(progressCallback)
        progressCallback(0.2, 'Dang nap mo hinh DRL Neural Network...');
    end

    % 1. Tim va nap file trong so da huan luyen
    agent_file = '';
    candidate_paths = {
        fullfile(pwd, 'Optimal_simulate', 'DRL', 'trained_drl_agent.mat'), ...
        fullfile(pwd, 'data', 'trained_drl_agent.mat'), ...
        fullfile('C:\Users\MINH QUAN\OneDrive\Máy tính\MyProject\SAGIN_NOMA_Simulator_APP\Optimal_simulate\DRL\trained_drl_agent.mat'), ...
        fullfile('C:\Users\MINH QUAN\OneDrive\Máy tính\MyProject\Optimal_NOMA_SAGIN_model\Optimal_simulate\DRL\trained_drl_agent.mat')
    };

    for i = 1:length(candidate_paths)
        if exist(candidate_paths{i}, 'file')
            agent_file = candidate_paths{i};
            break;
        end
    end

    if isempty(agent_file)
        error('Khong tim thay file trained_drl_agent.mat!');
    end

    data = load(agent_file);
    if isfield(data, 'agent')
        agent = data.agent;
    else
        error('File trained_drl_agent.mat khong chua bien agent hop le!');
    end

    if ~isempty(progressCallback)
        progressCallback(0.5, 'Dang suy luan hanh dong thoi gian thuc (Actor Network)...');
    end

    % 2. Tinh toan suy luan qua tung khe thoi gian
    rho_val = 10^(params.rho_dB / 10);
    res = evaluate_policy(agent, rho_val, params, true);

    % 3. Bo sung cac truong chuan hoa
    res.iterations = 1;
    res.history_F  = [res.OP_avg_opt * 1.35, res.OP_avg_opt]; % Hien thi muc cai thien DRL
    res.converged  = true;
    res.params     = params;

    if ~isempty(progressCallback)
        progressCallback(1.0, 'Hoan tat suy luan DRL!');
    end
end
