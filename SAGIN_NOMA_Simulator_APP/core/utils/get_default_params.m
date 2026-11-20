function params = get_default_params()
% GET_DEFAULT_PARAMS: Khoi tao va tra ve cau truc tham so he thong SAGIN-NOMA 2D (Oxz)
% Dung chung cho toan bo he thong: Script tinh toan, DRL va Giao dien App Designer.

    % 1. Toa do chuan hoa khong gian 2D (Oxz)
    params.pos_S      = [0.0, 1.0];     % Ve tinh nguon S tai dinh z = 1.0
    params.pos_D2     = [1.0, 0.0];     % Nguoi dung gan D2 tai x = 1.0
    params.pos_D1     = [1.5, 0.0];     % Nguoi dung xa D1 tai x = 1.5

    % 2. Hanh lang bay an toan cua UAV trong mat phang (Oxz): [xmin, zmin; xmax, zmax]
    params.uav_lb     = [0.2, 0.2];     % [Xmin, Zmin]
    params.uav_ub     = [1.8, 0.6];     % [Xmax, Zmax]

    % 3. Tham so kenh truyen va fading Rician
    params.nu_SU      = 2.0;            % So mu suy hao chang Ve tinh - UAV (LoS)
    params.nu_UD      = 2.5;            % So mu suy hao chang UAV - Nguoi dung mat dat
    params.K_SU       = 4;              % He so Rician chang 1 (~6 dB)
    params.K_UD       = 2;              % He so Rician chang 2 (~3 dB)
    params.N_UAV      = 4;              % So luong UAV trong cum (co che PRS)

    % 4. Yeu cau toc do truyen va nguong giai ma SINR
    params.R1         = 0.8;            % Target rate D1 (bps/Hz)
    params.R2         = 1.2;            % Target rate D2 (bps/Hz)
    params.gamma_th1  = 2^(2 * params.R1) - 1;
    params.gamma_th2  = 2^(2 * params.R2) - 1;

    % 5. Tham so Time-slotting & Khi dong hoc UAV
    params.T_total    = 20.0;           % Tong thoi gian bay khao sat (giay)
    params.N_slots    = 10;             % So luong khe thoi gian N
    params.dt         = params.T_total / params.N_slots;
    params.V_max      = 0.12;           % Van toc toi da cho phep (chuan hoa / s)
    params.q_start    = [0.35, 0.50];   % Toa do diem xuat phat cua UAV
    params.q_end      = [1.10, 0.30];   % Toa do diem dich cua UAV
    params.SIC_margin = 0.04;           % Bien do an toan giai ma hinh hoc SIC

    % 6. Cau hinh hoi tu thuat toan AO
    params.rho_dB         = 20;         % Muc SNR khao sat hoi tu (20 dB)
    params.max_iterations = 20;         % So vong lap toi da
    params.epsilon        = 1e-4;       % Nguong dung sai so tuong doi
end
