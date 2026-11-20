function q = compute_marcumq(a, b)
% COMPUTE_MARCUMQ First-order generalized Marcum-Q function Q_1(a, b).
%
% Syntax:
%   q = compute_marcumq(a, b)
%
% Description:
%   Evaluates the first-order Marcum-Q function defined as the Complementary
%   Cumulative Distribution Function (CCDF) of a Rician fading channel:
%       Q_1(a, b) = \int_{b}^{\infty} x \exp\left(-\frac{x^2 + a^2}{2}\right) I_0(a x) dx
%
%   If the MATLAB Communications Toolbox is available, the built-in 
%   'marcumq' routine is called. Otherwise, a high-precision numerical
%   adaptive Gauss-Kronrod quadrature or series expansion fallback is used.
%
% Inputs:
%   a - Non-negative scalar or array, non-centrality parameter \sqrt{2*K}
%   b - Non-negative scalar or array, normalized threshold
%
% Outputs:
%   q - Evaluated Marcum-Q value in [0, 1]
%
% References:
%   M. K. Simon and M.-S. Alouini, "Digital Communication over Fading Channels," 
%   2nd Edition, John Wiley & Sons, 2005.
%
% Author: Nguyen Minh Quan
% Organization: Posts and Telecommunications Institute of Technology (PTIT)
% Date: November 2026

    % Fast path: Built-in Communications Toolbox
    if exist('marcumq', 'file') == 2 || exist('marcumq', 'builtin') == 5
        q = marcumq(a, b, 1);
        return;
    end

    % Boundary conditions
    if b <= 0
        q = 1.0;
        return;
    end
    if a == 0
        q = exp(-b^2 / 2);
        return;
    end

    % Adaptive numerical integration fallback
    upper_lim = max(b + 8.0, 30.0);
    integrand = @(x) x .* exp(-(x.^2 + a.^2) / 2) .* besseli(0, a .* x);

    try
        q = integral(integrand, b, upper_lim, 'RelTol', 1e-6, 'AbsTol', 1e-8);
        q = max(0.0, min(1.0, q));
    catch
        % Series expansion fallback if numerical integral encounters singularity
        N_terms = 40;
        sum_val = 0;
        for k = 0:N_terms
            term = exp(-a^2 / 2) * ((a^2 / 2)^k / factorial(k)) * ...
                   gammainc(b^2 / 2, k + 1, 'upper');
            sum_val = sum_val + term;
        end
        q = max(0.0, min(1.0, sum_val));
    end
end
