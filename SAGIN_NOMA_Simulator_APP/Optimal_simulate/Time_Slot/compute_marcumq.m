function q = compute_marcumq(a, b)
% COMPUTE_MARCUMQ: Tính hàm Marcum-Q bậc 1: Q_1(a, b)
% Hỗ trợ tính toán chính xác cao, tự động tương thích khi không có Communications Toolbox
%
% Định nghĩa toán học:
%   Q_1(a, b) = \int_b^\infty x \exp(-(x^2 + a^2)/2) I_0(a x) dx

    % Nếu máy có sẵn hàm marcumq của MATLAB Communications Toolbox
    if exist('marcumq', 'file') == 2 || exist('marcumq', 'builtin') == 5
        q = marcumq(a, b, 1);
        return;
    end

    % Dự phòng: Tích phân số Gauss-Legendre / Simpson chính xác cao
    if b <= 0
        q = 1.0;
        return;
    end
    if a == 0
        q = exp(-b^2 / 2);
        return;
    end

    % Giới hạn trên tích phân (vượt qua giá trị này integrand xấp xỉ 0)
    upper_lim = max(b + 8.0, 30.0);
    integrand = @(x) x .* exp(-(x.^2 + a.^2) / 2) .* besseli(0, a .* x);

    try
        q = integral(integrand, b, upper_lim, 'RelTol', 1e-6, 'AbsTol', 1e-8);
        q = max(0.0, min(1.0, q));
    catch
        % Fallback xấp xỉ chuỗi nếu tích phân lỗi
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
