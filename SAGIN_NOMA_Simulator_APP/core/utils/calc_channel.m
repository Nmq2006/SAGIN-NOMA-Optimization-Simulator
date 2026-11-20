function [lambda_SU, lambda_UD1, lambda_UD2, d_SU, d_UD1, d_UD2] = calculate_channel_params(q_uav, params)
% CALCULATE_CHANNEL_PARAMS Compute Euclidean distances and average channel gains.
%
% Syntax:
%   [lambda_SU, lambda_UD1, lambda_UD2, d_SU, d_UD1, d_UD2] = calculate_channel_params(q_uav, params)
%
% Description:
%   Calculates 2D Euclidean distances from the UAV position to the LEO Satellite (S),
%   Far-user (D1), and Near-user (D2) in the normalized Oxz vertical plane.
%   Average channel power gains follow the pathloss model: \lambda = d^{-\nu}.
%
% Inputs:
%   q_uav  - Normalized 2D UAV coordinates [x, z], 1x2 or 2x1 vector
%   params - System configuration struct containing:
%            * pos_S: Satellite coordinates [x_S, z_S]
%            * pos_D1: Far user coordinates [x_D1, z_D1]
%            * pos_D2: Near user coordinates [x_D2, z_D2]
%            * nu_SU: Path loss exponent for Satellite-UAV feeder link
%            * nu_UD: Path loss exponent for UAV-User access links
%
% Outputs:
%   lambda_SU, lambda_UD1, lambda_UD2 - Average channel gains (linear scale)
%   d_SU, d_UD1, d_UD2               - Euclidean distances in normalized space
%
% Author: Nguyen Minh Quan
% Organization: Posts and Telecommunications Institute of Technology (PTIT)
% Date: November 2026

    % Euclidean distances in normalized 2D plane (Oxz)
    d_SU  = norm(params.pos_S  - q_uav);
    d_UD1 = norm(q_uav - params.pos_D1);
    d_UD2 = norm(q_uav - params.pos_D2);

    % Numerical safety floor to prevent division by zero
    d_SU  = max(d_SU,  1e-4);
    d_UD1 = max(d_UD1, 1e-4);
    d_UD2 = max(d_UD2, 1e-4);

    % Average channel power gains: lambda = d^(-nu)
    lambda_SU  = d_SU^(-params.nu_SU);
    lambda_UD1 = d_UD1^(-params.nu_UD);
    lambda_UD2 = d_UD2^(-params.nu_UD);
end
