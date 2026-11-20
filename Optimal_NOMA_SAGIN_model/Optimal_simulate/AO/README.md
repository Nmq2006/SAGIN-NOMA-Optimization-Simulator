# Tối ưu hóa Mạng SAGIN-NOMA bằng Thuật toán AO Tĩnh (Static AO Benchmark)

**Thư mục:** `Optimal_simulate/AO/`  
**Vai trò trong nghiên cứu:** Thiết lập **Nghiệm Tham chiếu Cơ sở Tĩnh (Static Baseline Benchmark)** tại một khe thời gian danh định (snapshot) nhằm kiểm chứng cơ sở toán học, sự hội tụ của giải thuật lặp luân phiên, và xác thực tính chính xác của công thức giải tích Marcum-Q so với mô phỏng thống kê Monte-Carlo.

---

## 1. Cấu trúc Mã nguồn

```
Optimal_simulate/AO/
├── main.m                          # Điều phối chính: Khởi tạo, giải AO tĩnh, kiểm chứng Monte-Carlo, xuất đồ thị
├── run_alternating_optimization.m  # Vòng lặp tối ưu hóa luân phiên giữa Tọa độ tĩnh q và Cặp công suất (a1, a2)
├── solve_subproblem_power.m        # Bài toán con 1: Tối ưu phân bổ công suất NOMA alpha1 (fminbnd)
├── solve_subproblem_position.m     # Bài toán con 2: Tối ưu tọa độ 2D Oxz của UAV (fmincon SQP)
├── calculate_outage_probability.m  # Hàm tính xác suất dừng toàn phần và thành phần (OP_total, OP_D1, OP_D2)
├── calculate_channel_params.m      # Tính cự ly Euclidean 2D và độ lợi kênh trung bình chặng S-U và U-D
├── compute_marcumq.m               # Hàm Marcum-Q bậc 1 độ chính xác cao (tương thích mọi phiên bản MATLAB)
├── plot_results.m                  # Hàm vẽ và lưu đồ thị tổng hợp 4 trong 1
└── README.md                       # Tài liệu hướng dẫn kỹ thuật này
```

---

## 2. Mô hình Tối ưu hóa & Thuật toán

### 2.1. Thiết lập Bài toán Tối ưu hóa Tĩnh (Snapshot)
$$\min_{\mathbf{q}_U, \, \alpha_1} \quad OP_{\text{total}}(\mathbf{q}_U, \alpha_1)$$
Thỏa mãn:
1. $x_U \in [0.2, 1.8], \quad z_U \in [0.2, 0.6]$
2. $\alpha_1 \ge \alpha_{1,\min} = \frac{\gamma_{th1}}{1 + \gamma_{th1}} \approx 0.670, \quad \alpha_1 + \alpha_2 = 1$

### 2.2. Kiểm chứng Monte-Carlo $10^5$ Mẫu
Tại mỗi mức SNR từ $10\text{ dB}$ đến $26\text{ dB}$, giải thuật sinh $10^5$ mẫu kênh truyền ngẫu nhiên độc lập:
* Chặng 1: Sinh $N_{\text{UAV}} = 3$ biến ngẫu nhiên Rician $K_{SU} = 5.0$, lấy cực đại theo cơ chế PRS.
* Chặng 2: Sinh 2 biến ngẫu nhiên Rician $K_{UD} = 3.0$ cho $D_1, D_2$, tính toán SINR thu và kiểm tra điều kiện giải mã thành công của NOMA.
* Kết quả Monte-Carlo (ký hiệu tròn `ko`) kiểm chứng khớp chặt chẽ với đường giải tích Marcum-Q trong sai số thống kê lý thuyết ($\approx 10\%$).

---

## 3. Danh mục Kết quả Xuất ra (Chuẩn IEEE)

Chạy file `main.m` sẽ tự động tạo và lưu các đồ thị chuẩn:
1. **`Figure1_Convergence.png`:** Đặc tính hội tụ đơn điệu qua 5 vòng lặp AO tại $\text{SNR} = 20\text{ dB}$ (từ $0.0701 \to 0.00533$, giảm $92.4\%$).
2. **`Figure2_Performance_vs_SNR.png`:** Đối chuẩn hiệu năng qua dải SNR ($10\text{ dB} \to 26\text{ dB}$) kèm các điểm kiểm chứng Monte-Carlo ($10^5$ mẫu) và 3 kịch bản đối chuẩn.
3. **`Figure3_Spatial_Trajectory_2D_Oxz.png`:** Bố trí không gian 2D $(Oxz)$, thể hiện vị trí xuất phát $U^{(0)}(0.5, 0.4)$ dịch chuyển về vị trí tối ưu $U^*(1.11, 0.20)$.
4. **`Figure4_TimeSlot_Power_Outage.png`:** Phân bổ công suất thích ứng theo SNR và xác suất dừng chi tiết từng người dùng.
5. **`Figure4_Power_Convergence.png`:** Tiến trình hội tụ của hệ số công suất và suy giảm xác suất dừng qua từng vòng lặp.
6. **`fig_AO_Optimization_Results.png`:** Đồ thị tổng hợp 4 trong 1.

---

## 4. Hướng dẫn Thực thi trên MATLAB

1. Mở **MATLAB** ($\ge$ R2020b).
2. Chuyển thư mục làm việc:
   ```matlab
   cd 'Optimal_simulate/AO'
   ```
3. Chạy lệnh:
   ```matlab
   main
   ```
4. Quá trình giải và mô phỏng Monte-Carlo hoàn tất sau khoảng $15\text{ giây}$.
