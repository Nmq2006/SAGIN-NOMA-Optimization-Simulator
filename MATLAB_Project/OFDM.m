% Project 3: He thong truyen dan OFDM co ban
clear; clc; close all;

% 1. Thong so OFDM
N_fft = 64;                 % So song mang con (FFT size)
N_cp = 16;                  % Chieu dai Cyclic Prefix
M = 4;                      % QPSK

% 2. Tao ky hieu du lieu trong mien tan so
dataBits = randi([0 1], N_fft * log2(M), 1);
modSym = qammod(dataBits, M, 'InputType', 'bit', 'UnitAveragePower', true);

% 3. Bo bien doi IFFT (Chuyen sang mien thoi gian)
txTime = ifft(modSym, N_fft);

% 4. Chen Cyclic Prefix (CP)
txSignal = [txTime(end - N_cp + 1 : end); txTime];

% 5. Kenh truyen da duong (Multipath Channel)
channelCoeff = [0.8, 0.4, 0.2]; % 3 duong truyen
fadedSignal = filter(channelCoeff, 1, txSignal);

% Them nhieu AWGN (SNR = 25 dB)
rxSignal = awgn(fadedSignal, 25, 'measured');

% 6. Phia Thu: Loai bo CP
rxNoCP = rxSignal(N_cp + 1 : N_cp + N_fft);

% 7. FFT (Chuyen ve lai mien tan so)
rxSym = fft(rxNoCP, N_fft);

% 8. Ve chom sao tin hieu (Constellation Diagram)
figure;
subplot(1, 2, 1);
plot(real(modSym), imag(modSym), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
grid on; axis([-2 2 -2 2]);
title('Chom sao tin hieu phat (QPSK)'); xlabel('In-Phase'); ylabel('Quadrature');

subplot(1, 2, 2);
plot(real(rxSym), imag(rxSym), 'b.', 'MarkerSize', 10);
grid on; axis([-2 2 -2 2]);
title('Tin hieu thu (bi anh huong boi Fading + Noise)'); xlabel('In-Phase'); ylabel('Quadrature');