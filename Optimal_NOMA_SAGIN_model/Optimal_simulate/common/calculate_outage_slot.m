function [OP_total, OP_D1, OP_D2] = calculate_outage_slot(alpha1, q_uav, rho_val, params)
% CALCULATE_OUTAGE_SLOT Exact analytical outage probability for dual-hop SAGIN-NOMA.
%
% Syntax:
%   [OP_total, OP_D1, OP_D2] = calculate_outage_slot(alpha1, q_uav, rho_val, params)
%
% Description:
%   Computes the analytical outage probabilities for a given UAV position and
%   NOMA power allocation at transmit SNR rho_val.
%   Hop 1 incorporates Partial Relay Selection (PRS) across N_UAV candidate relays.
%   Hop 2 incorporates Successive Interference Cancellation (SIC) at Near-user D2.
%
% Mathematical Formulas:
%   - Decode thresholds:
%       Theta1 = gamma_th1 / [rho * (alpha1 - alpha2 * gamma_th1)]
%       Theta2 = max(Theta1, gamma_th2 / [rho * alpha2])
%   - Hop 1 Success (PRS over N_UAV relays under Rician fading):
%       P_succ_Hop1 = 1 - (1 - Q_1(a_SU, b_SU))^N_UAV
%   - Hop 2 Success:
%       P_succ_Hop2 = Q_1(a_UD, b_UD)
%   - Individual Outage:
%       OP_Di = 1 - P_succ_Hop1 * P_succ_Hop2
%   - Total System Outage (Union probability):
%       OP_total = 1 - (1 - OP_D1) * (1 - OP_D2)
%
% Inputs:
%   alpha1  - Power allocation coefficient for far-user D1, scalar in (0, 1)
%   q_uav   - 2D UAV coordinates [x, z], 1x2 vector
%   rho_val - Transmit SNR in linear scale (rho = 10^(SNR_dB / 10))
%   params  - System configuration struct
%
% Outputs:
%   OP_total - Total system outage probability
%   OP_D1    - Outage probability of Far-user D1
%   OP_D2    - Outage probability of Near-user D2
%
% Author: Nguyen Minh Quan
% Organization: Posts and Telecommunications Institute of Technology (PTIT)
% Date: November 2026

    alpha2 = 1.0 - alpha1;

    % 1. Verification of SIC feasibility condition: alpha1 / alpha2 > gamma_th1
    sic_min = params.gamma_th1 / (1.0 + params.gamma_th1);
    if (alpha1 <= sic_min) || (alpha1 >= 1.0) || (alpha2 <= 0)
        OP_total = 1.0;
        OP_D1    = 1.0;
        OP_D2    = 1.0;
        return;
    end

    % 2. Threshold evaluation for SINR compliance
    denom1 = rho_val * (alpha1 - alpha2 * params.gamma_th1);
    if denom1 <= 0
        OP_total = 1.0; 
        OP_D1    = 1.0; 
        OP_D2    = 1.0;
        return;
    end
    Theta1 = params.gamma_th1 / denom1;
    Theta2 = max(Theta1, params.gamma_th2 / (rho_val * alpha2));

    % 3. Average channel power gains
    [lambda_SU, lambda_UD1, lambda_UD2] = calculate_channel_params(q_uav, params);

    % 4. Outage Probability of Far-user D1
    % Hop 1: Satellite -> UAV cluster (PRS selection)
    a_SU  = sqrt(2 * params.K_SU);
    b_SU1 = sqrt(2 * (params.K_SU + 1) * Theta1 / lambda_SU);
    q_hop1_D1 = compute_marcumq(a_SU, b_SU1);
    prob_hop1_D1 = 1.0 - (1.0 - q_hop1_D1)^params.N_UAV;

    % Hop 2: UAV -> D1
    a_UD  = sqrt(2 * params.K_UD);
    b_UD1 = sqrt(2 * (params.K_UD + 1) * Theta1 / lambda_UD1);
    prob_hop2_D1 = compute_marcumq(a_UD, b_UD1);

    OP_D1 = 1.0 - prob_hop1_D1 * prob_hop2_D1;

    % 5. Outage Probability of Near-user D2 (SIC on D1 signal, then decode own signal)
    % Hop 1: PRS
    b_SU2 = sqrt(2 * (params.K_SU + 1) * Theta2 / lambda_SU);
    q_hop1_D2 = compute_marcumq(a_SU, b_SU2);
    prob_hop1_D2 = 1.0 - (1.0 - q_hop1_D2)^params.N_UAV;

    % Hop 2: UAV -> D2
    b_UD2 = sqrt(2 * (params.K_UD + 1) * Theta2 / lambda_UD2);
    prob_hop2_D2 = compute_marcumq(a_UD, b_UD2);

    OP_D2 = 1.0 - prob_hop1_D2 * prob_hop2_D2;

    % 6. System Total Outage Probability (Exact Union Probability)
    OP_total = 1.0 - (1.0 - OP_D1) * (1.0 - OP_D2);

    % Clamping for numerical stability
    OP_total = max(min(OP_total, 1.0), 1e-9);
    OP_D1    = max(min(OP_D1, 1.0), 1e-9);
    OP_D2    = max(min(OP_D2, 1.0), 1e-9);
end
