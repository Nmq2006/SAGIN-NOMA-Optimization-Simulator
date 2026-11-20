# Tối ưu hóa Mạng SAGIN-NOMA theo Khe Thời gian & Ranh giới Hình học (Time-Slotting AO)

**Thư mục:** `Optimal_simulate/Time_Slot/`  
**Vai trò trong nghiên cứu:** Thiết lập **Nghiệm Tham chiếu Chuẩn Số học (Numerical Benchmark)** qua $N=10$ khe thời gian rời rạc bằng thuật toán Tối ưu hóa Luân phiên (Alternating Optimization - AO) kết hợp Quy hoạch toàn phương tuần tự (SQP) và Tìm kiếm Lát cắt Vàng.

---

## 1. Cấu trúc Mã nguồn

```
Optimal_simulate/Time_Slot/
├── main.m                          # Điều phối chính: Khởi tạo, giải AO động, quét SNR, xuất 4 đồ thị IEEE
├── run_timeslot_ao.m               # Vòng lặp tối ưu hóa luân phiên liên hợp giữa Quỹ đạo và Công suất
├── solve_subproblem_power.m        # Bài toán con 1: Tối ưu phân bổ công suất NOMA a1[n] từng khe (fminbnd)
├── solve_subproblem_trajectory.m   # Bài toán con 2: Tối ưu chuỗi tọa độ 2D Oxz [x_n, z_n] qua 10 khe (SQP fmincon)
├── calculate_outage_slot.m         # Hàm tính xác suất dừng giải tích dạng đóng Marcum-Q tại khe n
├── calculate_channel_params.m      # Tính cự ly Euclidean 2D và độ lợi kênh trung bình theo mô hình suy hao
├── check_geometric_constraints.m   # Kiểm tra ràng buộc ranh giới hình học SIC (C4) và vận tốc bay (C3)
├── compute_marcumq.m               # Hàm Marcum-Q bậc 1 độ chính xác cao (tự động fallback nếu thiếu toolbox)
├── plot_results.m                  # Hàm vẽ và lưu đồ thị tổng hợp 4 trong 1
└── README.md                       # Tài liệu hướng dẫn kỹ thuật này
```

---

## 2. Mô hình Tối ưu hóa & Thuật toán

### 2.1. Thiết lập Bài toán Tối ưu hóa Rời rạc
$$\min_{\mathbf{Q}, \, \boldsymbol{\alpha}_1} \quad \bar{P}_{\text{out}} = \frac{1}{N} \sum_{n=1}^N OP_{\text{total}}[n]$$
Thỏa mãn các ràng buộc:
1. **Hành lang bay (C1):** $x \in [0.1, 1.3]$, $z \in [0.2, 0.6]$.
2. **Điểm đầu & Điểm cuối (C2):** $\mathbf{q}[1] = \mathbf{q}_{\text{start}} = (0.35, 0.50)$, $\mathbf{q}[N] = \mathbf{q}_{\text{end}} = (1.10, 0.30)$.
3. **Vận tốc tối đa (C3):** $\|\mathbf{q}[n] - \mathbf{q}[n-1]\| \le V_{\max} \cdot dt = 0.24$ đơn vị.
4. **Ranh giới an toàn SIC (C4):** $x[n] \le 1.21$ (bảo đảm độ lợi kênh trung bình $d_{UD2} < d_{UD1}$ không đảo lộn).
5. **Ngưỡng công suất giải mã (C5, C6):** $\alpha_1[n] \ge \alpha_{1,\min} = \frac{\gamma_{th1}}{1 + \gamma_{th1}} \approx 0.670$.

### 2.2. Tiến trình Giải Thuật toán AO
* **Khởi tạo:** Quỹ đạo bay thẳng đều từ $\mathbf{q}_{\text{start}}$ đến $\mathbf{q}_{\text{end}}$, công suất cố định $\alpha_1 = 0.75$.
* **Vòng lặp $k$:**
  1. Cố định $\mathbf{Q}^{(k-1)}$, giải tối ưu phân bổ công suất $\boldsymbol{\alpha}_1^{(k)}$ cho từng khe độc lập bằng `fminbnd`.
  2. Cố định $\boldsymbol{\alpha}_1^{(k)}$, giải tối ưu toàn bộ chuỗi tọa độ $\mathbf{Q}^{(k)}$ bằng giải thuật SQP (`fmincon`) dưới ràng buộc phi tuyến `check_geometric_constraints`.
  3. Kiểm tra điều kiện dừng: $|\bar{P}_{\text{out}}^{(k)} - \bar{P}_{\text{out}}^{(k-1)}| < \epsilon = 10^{-4}$ (thường hội tụ sau 4 vòng lặp).

---

## 3. Danh mục Kết quả Xuất ra (Chuẩn IEEE)

Chạy file `main.m` sẽ tự động tạo và lưu các đồ thị độ phân giải cao:
1. **`Figure1_Convergence.png`:** Tiến trình suy giảm đơn điệu của xác suất dừng trung bình qua 4 vòng lặp SQP tại $\text{SNR} = 20\text{ dB}$ (từ $0.0401 \to 0.0206$).
2. **`Figure2_Performance_vs_SNR.png`:** Đối chuẩn xác suất dừng qua dải SNR ($10\text{ dB} \to 26\text{ dB}$) giữa Đề xuất Joint AO và 3 kịch bản đối chuẩn (Chỉ tối ưu quỹ đạo, Chỉ tối ưu công suất, Không tối ưu).
3. **`Figure3_Spatial_Trajectory_2D_Oxz.png`:** Quỹ đạo bay tối ưu trong mặt phẳng 2D $(Oxz)$, thể hiện rõ việc UAV hạ độ cao vào vùng phủ sóng tốt và duy trì khoảng cách đệm an toàn trước ranh giới SIC $x \le 1.21$.
4. **`Figure4_TimeSlot_Power_Outage.png`:**
   * Subplot 1: Phân bổ công suất thích ứng $(\alpha_1[n], \alpha_2[n])$ từng khe, thỏa mãn ngưỡng an toàn SIC.
   * Subplot 2: Xác suất dừng chi tiết của $D_1$, $D_2$ và toàn phần hệ thống qua từng khe.

---

## 4. Hướng dẫn Thực thi trên MATLAB

1. Mở **MATLAB** (phiên bản $\ge$ R2020b, khuyến nghị R2024b).
2. Điều hướng Current Folder về thư mục:
   ```matlab
   cd 'Optimal_simulate/Time_Slot'
   ```
3. Chạy lệnh:
   ```matlab
   main
   ```
4. Toàn bộ tiến trình hội tụ và các đồ thị sẽ được xuất tự động sau khoảng $10\text{ giây}$.
