# Tối ưu hóa Mạng SAGIN-NOMA bằng Học Tăng Cường Sâu (DRL DDPG Actor-Critic)

**Thư mục:** `Optimal_simulate/DRL/`  
**Vai trò trong nghiên cứu:** Hiện thực hóa khả năng **điều khiển bay tự chủ và phân bổ công suất trực tuyến ở cấp độ mili-giây** bằng mạng nơ-ron Actor-Critic liên tục (Deep Deterministic Policy Gradient - DDPG), giải quyết triệt để điểm nghẽn độ trễ tính toán của phương pháp tối ưu số lặp AO.

---

## 1. Cấu trúc Mã nguồn

```
Optimal_simulate/DRL/
├── main.m                          # Điều phối chính: Nạp mô hình/Huấn luyện, quét SNR, xuất 5 đồ thị IEEE
├── drl_agent.m                     # Class triển khai mạng Actor-Critic, bộ tối ưu Adam và Replay Buffer
├── step_env.m                      # Mô phỏng môi trường MDP: cập nhật tọa độ, kiểm tra an toàn SIC, tính phần thưởng
├── train_drl.m                     # Vòng lặp huấn luyện mạng qua 260 episodes với Warm-Start Seeding
├── evaluate_policy.m               # Hàm đánh giá chính sách tất định (không nhiễu) qua các khe và dải SNR
├── calculate_outage_slot.m         # Hàm tính xác suất dừng giải tích dạng đóng Marcum-Q tại khe n
├── calculate_channel_params.m      # Tính cự ly Euclidean 2D và độ lợi kênh trung bình
├── check_geometric_constraints.m   # Kiểm tra ràng buộc ranh giới hình học SIC và vận tốc bay
├── compute_marcumq.m               # Hàm Marcum-Q bậc 1 độ chính xác cao
├── plot_results.m                  # Hàm vẽ và lưu đồ thị tổng hợp 4 trong 1
├── trained_drl_agent.mat           # Trọng số mô hình DDPG đã huấn luyện sẵn (nạp tức thì < 1 giây)
└── README.md                       # Tài liệu hướng dẫn kỹ thuật này
```

---

## 2. Thiết lập Markov Decision Process (MDP)

1. **Không gian Trạng thái ($\mathbf{s}_n \in \mathbb{R}^8$):**
   $$\mathbf{s}_n = \left[ \frac{x_n}{X_{\max}}, \; \frac{z_n}{Z_{\max}}, \; \min\left(1, \frac{d_{SU}}{2}\right), \; \min\left(1, \frac{d_{UD1}}{2}\right), \; \min\left(1, \frac{d_{UD2}}{2}\right), \; \min\left(1, \frac{d_{\text{end}}}{2}\right), \; \max\left(0, \frac{x_{\text{sic\_bound}} - x_n}{1.0}\right), \; \frac{N - n}{N} \right]^T$$
   Cung cấp thông tin chuẩn hóa về tọa độ hiện tại, cự ly tới các nút, khoảng cách tới đích đến, cự ly tới ranh giới an toàn SIC và thời gian còn lại.

2. **Không gian Hành động liên tục ($\mathbf{a}_n \in \mathbb{R}^3$):**
   $$\mathbf{a}_n = [v_n, \; \theta_n, \; \alpha_{1,n}]^T$$
   * $v_n \in [0, V_{\max}]$: Vận tốc bay của UAV trong khe thời gian ($V_{\max} = 0.12$).
   * $\theta_n \in [-\pi, \pi]$: Góc hướng bay trong mặt phẳng $(Oxz)$. Bước dịch chuyển được tính: $\Delta x = v \cdot dt \cdot \cos\theta$, $\Delta z = v \cdot dt \cdot \sin\theta$, tự động bảo đảm tuyệt đối ràng buộc vận tốc $\sqrt{\Delta x^2 + \Delta z^2} \le V_{\max} \cdot dt = 0.24$.
   * $\alpha_{1,n} \in [0.70, 0.83]$: Hệ số phân bổ công suất NOMA cho người dùng xa $D_1$.

3. **Hàm Phần thưởng Đa mục tiêu (Reward Function):**
   $$R_n = -\ln(OP_{\text{total}}[n] + 10^{-6}) + 4.0 \cdot (d_{\text{curr}} - d_{\text{next}}) - P_{\text{boundary}} + R_{\text{terminal}}$$
   * Thành phần logarit duy trì độ dốc gradient ổn định khi xác suất dừng giảm sâu.
   * Phần thưởng tiến trình khuyến khích UAV bay về phía tọa độ đích $q_{\text{end}}$.
   * Hàm phạt ngăn chặn xâm phạm hành lang bay hoặc ranh giới SIC $x > 1.21$.
   * Phần thưởng điểm cuối nếu tác tử cập đích chính xác tại bước $n = N$.

---

## 3. Danh mục Kết quả Xuất ra (Chuẩn IEEE)

Chạy file `main.m` sẽ tự động tạo và lưu 5 đồ thị chuẩn:
1. **`Figure1_Convergence.png`:** Đường cong hội tụ phần thưởng tích lũy (Reward) và tiến trình suy giảm xác suất dừng qua các Episode huấn luyện.
2. **`Figure2_Performance_vs_SNR.png`:** Đối chuẩn hiệu năng xác suất dừng qua dải SNR ($10\text{ dB} \to 26\text{ dB}$) giữa Đề xuất DRL DDPG, Nghiệm tham chiếu Joint AO và các kịch bản đối chuẩn.
3. **`Figure3_Spatial_Trajectory_2D_Oxz.png`:** Quỹ đạo bay tự chủ do mạng Actor điều hướng qua 10 bước bay, duy trì vùng đệm an toàn cách ranh giới SIC $x \le 1.21$ và hạ cánh chính xác tại $q_{\text{end}}$.
4. **`Figure4_TimeSlot_Power_Outage.png`:** Phân tích chi tiết phân bổ công suất NOMA và xác suất dừng tại từng bước bay do tác tử DRL quyết định.
5. **`fig_DRL_Optimization_Results.png`:** Đồ thị tổng hợp 4 trong 1 bao quát toàn bộ kết quả mô phỏng.

---

## 4. Hướng dẫn Thực thi trên MATLAB

1. Mở **MATLAB** ($\ge$ R2020b).
2. Chuyển thư mục làm việc:
   ```matlab
   cd 'Optimal_simulate/DRL'
   ```
3. Chạy file điều phối:
   ```matlab
   main
   ```
   * **Chế độ Mặc định (`RETRAIN = false`):** Nạp tức thì mô hình đã tối ưu từ `trained_drl_agent.mat` trong vòng $\approx 1\text{ giây}$ và xuất toàn bộ các đồ thị chuẩn IEEE.
   * **Chế độ Huấn luyện lại (`RETRAIN = true`):** Mở `main.m`, đặt `RETRAIN = true` tại dòng 105. Quá trình huấn luyện 260 episodes mất khoảng $5\text{ phút}$ trên CPU tiêu chuẩn (được cố định random seed 42 để bảo đảm tính tái lập).
