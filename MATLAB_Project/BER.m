% Project 1: Mo phong 16-QAM qua kenh AWGN va ve duong cong BER
clear; clc; close all;

% 1. Thong so he thong
M = 16;                     % 16-QAM
k = log2(M);                % So bit tren moi ky hieu (4 bits)
numBits = 2e5;              % Tong so bit phat (200,000 bits)
ebnoVec = 0:2:14;           % Dai gia tri Eb/No (dB)

% 2. Tao du lieu bit ngau nhien (so luong bit phai chia het cho k)
dataIn = randi([0 1], numBits, 1);

% 3. Dieu che 16-QAM truc tiep tu bit (Chuan hoa cong suat trung binh ve 1)
modData = qammod(dataIn, M, 'InputType', 'bit', 'UnitAveragePower', true);

berEst = zeros(size(ebnoVec));

% 4. Vong lap truyen qua kenh AWGN theo tung muc Eb/No
fprintf('Dang chay mo phong BER...\n');
for idx = 1:length(ebnoVec)
    % Chuyen doi Eb/No (dB) sang SNR (dB): SNR = Eb/No + 10*log10(k)
    snr = ebnoVec(idx) + 10*log10(k);
    
    % Truyen tin hieu qua kenh nhieu Gaussian trang (AWGN)
    rxSig = awgn(modData, snr, 'measured');
    
    % Giai dieu che truc tiep ra bit
    dataOut = qamdemod(rxSig, M, 'OutputType', 'bit', 'UnitAveragePower', true);
    
    % Tinh so bit loi va ty le loi bit BER
    [numErr, berEst(idx)] = biterr(dataIn, dataOut);
    fprintf('  Eb/No = %2d dB | So bit loi: %5d | BER = %.2e\n', ...
        ebnoVec(idx), numErr, berEst(idx));
end

% 5. Tinh BER ly thuyet theo cong thuc giai tich
berTheory = berawgn(ebnoVec, 'qam', M);

% 6. Ve do thi BER mo phong vs Ly thuyet
figure('Name', 'BER of 16-QAM over AWGN', 'Color', 'w');
semilogy(ebnoVec, berEst, 'bo-', 'LineWidth', 1.8, 'MarkerSize', 7, 'MarkerFaceColor', 'b'); 
hold on;
semilogy(ebnoVec, berTheory, 'r--', 'LineWidth', 2);
grid on;
axis([0 14 1e-5 1]); % Gioi han truc toa do de nhin ro
xlabel('E_b / N_0 (dB)', 'FontSize', 11); 
ylabel('Bit Error Rate (BER)', 'FontSize', 11);
title('Duong cong BER cho he thong 16-QAM qua kenh AWGN', 'FontSize', 12);
legend('BER Mo phong (Monte Carlo)', 'BER Ly thuyet', 'Location', 'southwest', 'FontSize', 11);