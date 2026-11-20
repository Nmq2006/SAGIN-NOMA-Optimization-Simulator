function [opt_alpha1, min_val] = solve_subproblem_power(current_q, rho_val, params)
    sic_min = params.gamma_th1 / (1.0 + params.gamma_th1);
    lb_a = sic_min + 1e-4; % Ràng buộc dưới để thỏa mãn SIC
    ub_a = 0.9999;         % Ràng buộc trên
    obj_fun = @(a1) calculate_outage_probability(a1, current_q, rho_val, params);
    
    opts = optimset('Display', 'off', 'TolX', 1e-5);
    [opt_alpha1, min_val] = fminbnd(obj_fun, lb_a, ub_a, opts);
end