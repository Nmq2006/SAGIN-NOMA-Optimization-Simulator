function [OP_total, OP_D1, OP_D2] = calculate_outage_probability(alpha1, q_uav, rho_val, params)
    % =========================================================================
    % TÍNH XÁC SUẤT DỪNG TOÀN PHẦN THEO MÔ HÌNH 2D (Oxz) CHUẨN HÓA KHÔNG THỨ NGUYÊN
    % q_uav = [x_tilde, z_tilde]: Tọa độ chuẩn hóa của UAV trong mặt phẳng Oxz
    %
    % Outputs:
    %   OP_total: Xác suất dừng toàn phần hệ thống
    %   OP_D1: Xác suất dừng của người dùng xa D1
    %   OP_D2: Xác suất dừng của người dùng gần D2
    % =========================================================================

    % Trích xuất tham số
    gamma_th1 = params.gamma_th1;
    gamma_th2 = params.gamma_th2;
    pos_S     = params.pos_S;     % [x, z] của vệ tinh S
    pos_D1    = params.pos_D1;    % [x, z] của người dùng xa D1
    pos_D2    = params.pos_D2;    % [x, z] của người dùng gần D2
    nu_SU     = params.nu_SU;     % Suy hao đường truyền chặng Vệ tinh - UAV
    nu_UD     = params.nu_UD;     % Suy hao đường truyền chặng UAV - Người dùng
    K_SU      = params.K_SU;      % Hệ số Rician chặng 1
    K_UD      = params.K_UD;      % Hệ số Rician chặng 2
    N_UAV     = params.N_UAV;     % Số lượng UAV trong cụm (PRS)

    alpha2 = 1.0 - alpha1;

    % 1. Kiểm tra ràng buộc khả thi giải mã SIC: alpha1 / alpha2 > gamma_th1
    sic_min = gamma_th1 / (1.0 + gamma_th1);
    if (alpha1 <= sic_min) || (alpha1 >= 1.0) || (alpha2 <= 0)
        OP_total = 1.0;
        OP_D1    = 1.0;
        OP_D2    = 1.0;
        return;
    end

    % 2. Tính ngưỡng độ lợi kênh yêu cầu Theta1, Theta2 (Công thức 12 & 13)
    denom = rho_val * (alpha1 - alpha2 * gamma_th1);
    if denom <= 0
        OP_total = 1.0;
        OP_D1    = 1.0;
        OP_D2    = 1.0;
        return;
    end
    Theta1 = gamma_th1 / denom;
    Theta2 = max(Theta1, gamma_th2 / (rho_val * alpha2));

    % 3. Khoảng cách và độ lợi kênh trung bình (sử dụng calculate_channel_params)
    [lambda_SU, lambda_UD1, lambda_UD2] = calculate_channel_params(q_uav, params);

    % 4. Xác suất dừng cho người dùng D1 (Công thức 17)
    % Chặng 1: PRS chọn nhánh cực đại trong số N UAV
    a_SU = sqrt(2 * K_SU);
    b_SU1 = sqrt(2 * (K_SU + 1) * Theta1 / lambda_SU);
    q_hop1_D1 = compute_marcumq(a_SU, b_SU1);
    prob_hop1_D1 = 1 - (1 - q_hop1_D1)^N_UAV;

    % Chặng 2: UAV -> D1
    a_UD = sqrt(2 * K_UD);
    b_UD1 = sqrt(2 * (K_UD + 1) * Theta1 / lambda_UD1);
    prob_hop2_D1 = compute_marcumq(a_UD, b_UD1);

    OP_D1 = 1 - prob_hop1_D1 * prob_hop2_D1;

    % 5. Xác suất dừng cho người dùng D2 (Công thức 17)
    % Chặng 1: PRS
    b_SU2 = sqrt(2 * (K_SU + 1) * Theta2 / lambda_SU);
    q_hop1_D2 = compute_marcumq(a_SU, b_SU2);
    prob_hop1_D2 = 1 - (1 - q_hop1_D2)^N_UAV;

    % Chặng 2: UAV -> D2
    b_UD2 = sqrt(2 * (K_UD + 1) * Theta2 / lambda_UD2);
    prob_hop2_D2 = compute_marcumq(a_UD, b_UD2);

    OP_D2 = 1 - prob_hop1_D2 * prob_hop2_D2;

    % 6. Xác suất dừng toàn phần hệ thống (Xác suất có ít nhất 1 người dùng dừng)
    OP_total = 1 - (1 - OP_D1) * (1 - OP_D2);
    OP_total = max(min(OP_total, 1.0), 1e-9);
    OP_D1    = max(min(OP_D1, 1.0), 1e-9);
    OP_D2    = max(min(OP_D2, 1.0), 1e-9);
end
