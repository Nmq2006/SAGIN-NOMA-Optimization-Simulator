function [lambda_SU, lambda_UD1, lambda_UD2, d_SU, d_UD1, d_UD2] = calculate_channel_params(q_uav, params)
% CALCULATE_CHANNEL_PARAMS: Tính khoảng cách và độ lợi kênh trung bình trong mặt phẳng 2D (Oxz)
% q_uav = [x, z]: Tọa độ chuẩn hóa của UAV trong mặt phẳng Oxz (tương đồng với folder AO)
%
% Inputs:
%   q_uav: Tọa độ [x, z] của UAV tại khe thời gian hiện tại
%   params: Cấu trúc chứa các tham số hệ thống
%
% Outputs:
%   lambda_SU, lambda_UD1, lambda_UD2: Độ lợi kênh trung bình tương ứng
%   d_SU, d_UD1, d_UD2: Khoảng cách Euclidean trong mặt phẳng 2D Oxz

    % Khoảng cách Euclidean trong hệ tọa độ chuẩn hóa 2D (Oxz)
    d_SU  = norm(params.pos_S  - q_uav);
    d_UD1 = norm(q_uav - params.pos_D1);
    d_UD2 = norm(q_uav - params.pos_D2);

    % Đảm bảo khoảng cách không suy biến về 0
    d_SU  = max(d_SU,  1e-4);
    d_UD1 = max(d_UD1, 1e-4);
    d_UD2 = max(d_UD2, 1e-4);

    % Độ lợi kênh trung bình theo số mũ suy hao: lambda = d^(-nu)
    lambda_SU  = d_SU^(-params.nu_SU);
    lambda_UD1 = d_UD1^(-params.nu_UD);
    lambda_UD2 = d_UD2^(-params.nu_UD);
end
