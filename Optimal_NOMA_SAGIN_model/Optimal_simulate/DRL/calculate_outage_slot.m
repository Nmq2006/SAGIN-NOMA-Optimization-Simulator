function [OP_total, OP_D1, OP_D2] = calculate_outage_slot(alpha1, q_uav, rho_val, params)
% CALCULATE_OUTAGE_SLOT: Tính xác suất dừng tại 1 khe thời gian n trong mặt phẳng 2D (Oxz)
% q_uav = [x, z]: Tọa độ chuẩn hóa của UAV trong mặt phẳng Oxz
%
% Inputs:
%   alpha1: Hệ số phân bổ công suất cho D1 (Far-user) tại khe n (0 < alpha1 < 1)
%   q_uav: Tọa độ [x, z] của UAV tại khe n
%   rho_val: Mức SNR phát tuyến tính
%   params: Cấu trúc tham số hệ thống
%
% Outputs:
%   OP_total: Xác suất dừng toàn phần hệ thống tại khe n
%   OP_D1: Xác suất dừng của người dùng D1 tại khe n
%   OP_D2: Xác suất dừng của người dùng D2 tại khe n

    alpha2 = 1.0 - alpha1;

    % 1. Kiểm tra tính khả thi giải mã SIC: alpha1 / alpha2 > gamma_th1
    sic_min = params.gamma_th1 / (1.0 + params.gamma_th1);
    if (alpha1 <= sic_min) || (alpha1 >= 1.0) || (alpha2 <= 0)
        OP_total = 1.0;
        OP_D1    = 1.0;
        OP_D2    = 1.0;
        return;
    end

    % 2. Tính các ngưỡng độ lợi kênh yêu cầu Theta1, Theta2 (Công thức 12 & 13)
    denom1 = rho_val * (alpha1 - alpha2 * params.gamma_th1);
    if denom1 <= 0
        OP_total = 1.0; OP_D1 = 1.0; OP_D2 = 1.0;
        return;
    end
    Theta1 = params.gamma_th1 / denom1;
    Theta2 = max(Theta1, params.gamma_th2 / (rho_val * alpha2));

    % 3. Tính độ lợi kênh trung bình tại tọa độ [x, z]
    [lambda_SU, lambda_UD1, lambda_UD2] = calculate_channel_params(q_uav, params);

    % 4. Xác suất dừng cho D1 (Far-user)
    % Chặng 1: Vệ tinh S -> UAV qua cơ chế PRS chọn cực đại N_UAV nhánh
    a_SU  = sqrt(2 * params.K_SU);
    b_SU1 = sqrt(2 * (params.K_SU + 1) * Theta1 / lambda_SU);
    q_hop1_D1 = compute_marcumq(a_SU, b_SU1);
    prob_hop1_D1 = 1.0 - (1.0 - q_hop1_D1)^params.N_UAV;

    % Chặng 2: UAV -> D1
    a_UD  = sqrt(2 * params.K_UD);
    b_UD1 = sqrt(2 * (params.K_UD + 1) * Theta1 / lambda_UD1);
    prob_hop2_D1 = compute_marcumq(a_UD, b_UD1);

    OP_D1 = 1.0 - prob_hop1_D1 * prob_hop2_D1;

    % 5. Xác suất dừng cho D2 (Near-user)
    % Chặng 1: PRS
    b_SU2 = sqrt(2 * (params.K_SU + 1) * Theta2 / lambda_SU);
    q_hop1_D2 = compute_marcumq(a_SU, b_SU2);
    prob_hop1_D2 = 1.0 - (1.0 - q_hop1_D2)^params.N_UAV;

    % Chặng 2: UAV -> D2
    b_UD2 = sqrt(2 * (params.K_UD + 1) * Theta2 / lambda_UD2);
    prob_hop2_D2 = compute_marcumq(a_UD, b_UD2);

    OP_D2 = 1.0 - prob_hop1_D2 * prob_hop2_D2;

    % 6. Xác suất dừng toàn phần hệ thống (Có ít nhất một người dùng bị dừng)
    OP_total = 1.0 - (1.0 - OP_D1) * (1.0 - OP_D2);
    OP_total = max(min(OP_total, 1.0), 1e-9);
    OP_D1    = max(min(OP_D1, 1.0), 1e-9);
    OP_D2    = max(min(OP_D2, 1.0), 1e-9);
end
