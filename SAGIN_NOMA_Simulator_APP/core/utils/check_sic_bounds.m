function [c, ceq] = check_geometric_constraints(q_vec, params)
% CHECK_GEOMETRIC_CONSTRAINTS Evaluates kinematic and SIC user-ordering constraints.
%
% Syntax:
%   [c, ceq] = check_geometric_constraints(q_vec, params)
%
% Description:
%   Evaluates non-linear inequality constraints c(q) <= 0 for trajectory optimization:
%     1. Maximum velocity / displacement per time slot:
%        ||q[n] - q[n-1]|| <= V_max * dt
%     2. Large-scale geometric SIC user-ordering constraint:
%        ||q[n] - pos_D2|| - ||q[n] - pos_D1|| + margin <= 0
%        ensuring average channel gain g2 > g1 so that SIC decoding order holds.
%
% Inputs:
%   q_vec  - Flattened trajectory vector [x1; z1; x2; z2; ...; xN; zN], 2N x 1
%   params - System parameters struct
%
% Outputs:
%   c   - Vector of inequality constraint residuals (c <= 0 is feasible)
%   ceq - Vector of equality constraint residuals (empty for this problem)
%
% Author: Nguyen Minh Quan
% Organization: Posts and Telecommunications Institute of Technology (PTIT)
% Date: November 2026

    N = params.N_slots;
    Q = reshape(q_vec, 2, N)'; % Reshape to N x 2 matrix

    max_dist_step = params.V_max * params.dt; % Maximum allowable displacement per step

    c_speed = zeros(N - 1, 1);
    c_geo   = zeros(N, 1);

    pos_D1 = params.pos_D1;
    pos_D2 = params.pos_D2;

    for n = 1:N
        q_n = Q(n, :);

        % Geometric SIC user-ordering constraint (C4)
        dist_to_D2 = norm(q_n - pos_D2);
        dist_to_D1 = norm(q_n - pos_D1);
        c_geo(n) = dist_to_D2 - dist_to_D1 + params.SIC_margin;

        % Kinematic speed limit constraint (C3)
        if n >= 2
            q_prev = Q(n - 1, :);
            step_dist = norm(q_n - q_prev);
            c_speed(n - 1) = step_dist - max_dist_step;
        end
    end

    c = [c_speed; c_geo];
    ceq = [];
end
