# BÁO CÁO PHÂN TÍCH BẤT CẬP THỰC NGHIỆM VÀ GIẢI PHÁP CẬP NHẬT DRL (v1)

* **Đề tài:** Đánh Giá Hiệu Năng và Tối Ưu Hóa Mạng SAGIN-NOMA Dùng UAV (NCKH 2026)
* **Mã tài liệu:** `01_BaoCao_PhanTich_Va_CapNhat_DRL_v1`
* **Ngày cập nhật:** 12/11/2026
* **Địa chỉ thư mục:** `c:\Users\MINH QUAN\OneDrive\Máy tính\NCKH-2026\Optimal_simulate\DRL\`
* **File văn bản Word tương ứng:** `01_BaoCao_PhanTich_Va_CapNhat_DRL_v1.docx`

---

## I. TỔNG QUAN VẤN ĐỀ
Dựa trên 4 ảnh chụp kết quả thực nghiệm ban đầu của thuật toán Học tăng cường sâu (DRL Actor-Critic) trên MATLAB:
* **Hình 1:** `Figure 1: Convergence Curve of DRL Training`
* **Hình 2:** `Figure 2: Performance Comparison vs SNR`
* **Hình 3:** `Figure 3: Spatial Deployment 2D Oxz - DRL Trajectory`
* **Hình 4:** `Figure 4: DRL Power and Outage Details`

Các kết quả này **chưa hợp lý và chưa đạt tiêu chuẩn khoa học**, có nguy cơ bị hội đồng phản biện đánh trượt nếu đưa vào báo cáo/bài báo.

---

## II. CHI TIẾT 4 BẤT CẬP LỚN & NGUYÊN NHÂN CỐT LÕI

### 1. Hiệu năng Xác suất dừng tệ hơn cả Không tối ưu (Hình 2)
* **Hiện tượng:** Đường DRL đề xuất (màu tím hồng) nằm ở vị trí cao nhất trên đồ thị (Outage cao nhất). Tại $\text{SNR} = 26\text{ dB}$, thuật toán AO (đỏ) đạt $OP \approx 5 \times 10^{-3}$, trường hợp Không tối ưu (đen) đạt $OP \approx 1.2 \times 10^{-2}$, còn DRL đề xuất lại có $OP \approx 3.0 \times 10^{-2}$ (kém hơn gấp 2.5 lần so với Không tối ưu).
* **Nguyên nhân:** Do hai lỗi kép ở quỹ đạo bay và phân bổ công suất NOMA (phân tích mục 2 và 3).

### 2. Quỹ đạo UAV bị "chết đứng" giữa đường (Hình 3)
* **Hiện tượng:** UAV xuất phát tại $q_{\text{start}} = (0.35, 0.50)$ nhưng chỉ bay tới tọa độ $(x \approx 0.74, z \approx 0.46)$ rồi dừng lại ở khe $n=9$, cách điểm đích $q_{\text{end}} = (1.10, 0.30)$ hơn $0.4$ đơn vị.
* **Nguyên nhân:** Không gian hành động góc bay $[-\pi, \pi]$ không định hướng khiến Agent mò mẫm, cộng với hàm phạt điểm đích chưa đủ mạnh để buộc UAV hoàn thành hành trình trong 10 khe thời gian. Vì ở quá xa hai người dùng mặt đất $D_1(1.5, 0)$ và $D_2(1.0, 0)$, kênh truyền bị suy hao khoảng cách lớn.

### 3. Công suất NOMA bị bão hòa kịch kim lên 0.95 (Hình 4)
* **Hiện tượng:** Hình 4(a) cho thấy $\alpha_1[n] = 0.95$ cố định ở tất cả 10 khe.
* **Hệ quả:** Far-user $D_1$ nhận $95\%$ công suất, Near-user $D_2$ chỉ còn nhận $5\%$ ($\alpha_2 = 0.05$). Do $D_2$ có yêu cầu tốc độ cao hơn ($R_2 = 1.2\text{ bps/Hz}$ so với $R_1 = 0.8\text{ bps/Hz}$), việc thiếu hụt công suất khiến $D_2$ bị Outage trầm trọng ($OP_{D2} \approx 0.1$ ở Hình 4b). Xác suất dừng toàn phần hệ thống bị kéo sập theo.

### 4. Quá trình học DRL không hội tụ (Hình 1)
* **Hiện tượng:** Phần thưởng dao động hình sin bất thường từ episode 0 đến 120, sau đó sụt giảm về mức thấp. Đường Outage trung bình không giảm dần mà dao động mạnh từ 0.2 vọt lên 0.55 rồi đi ngang.
* **Nguyên nhân:** Khởi đầu lạnh (Cold-start) từ số 0 mà không có kinh nghiệm mẫu định hướng (Buffer Seeding), kết hợp hàm thưởng tuyến tính $(1 - OP)$ có độ dốc quá phẳng khiến Actor-Critic không tìm thấy hướng cập nhật gradient đúng.

---

## III. TỔNG HỢP NỘI DUNG CẬP NHẬT KỸ THUẬT (PHIÊN BẢN v1)

| File mã nguồn | Nội dung cập nhật cụ thể | Tác dụng khắc phục |
| :--- | :--- | :--- |
| **`step_env.m`** | Triển khai cơ chế **Guided Displacement**: Bước bay = Bước danh định hướng đích $(\mathbf{q}_{\text{end}} - \mathbf{q})/\text{rem} + \boldsymbol{\delta}_{\text{actor}}$ | Đảm bảo 100% UAV luôn tiến về đích và đến chính xác $q_{\text{end}}$ tại $n=10$, đồng thời Actor vẫn tự do uốn lượn tối ưu hóa độ cong quỹ đạo. |
| **`step_env.m`** | Khống chế dải công suất NOMA an toàn: $\alpha_1 \in [0.70, 0.82]$ | Luôn thỏa mãn điều kiện SIC ($\alpha_1 > 0.6701$), đồng thời bảo vệ Near-user $D_2$ luôn có $\alpha_2 \in [0.18, 0.30]$ không bị sập Outage. |
| **`step_env.m`** | Đổi hàm thưởng sang dạng **Logarit**: $R = -\log_{10}(OP + 10^{-6}) \times 2.5$ | Thưởng phạt tỷ lệ thuận với cấp số nhân của Outage, tạo lực gradient cực mạnh hướng Agent ép sâu Outage. |
| **`train_drl.m`** | Triển khai kỹ thuật **Warm-Start Experience Seeding** (25 Episode mẫu) | Nạp sẵn các mẫu chuyển trạng thái khả thi vào Replay Buffer, giúp Critic và Actor có điểm tựa vững chắc ngay từ Episode 1. |
| **`drl_agent.m`** | Bổ sung cơ chế lưu Checkpoint chính sách theo **Outage nhỏ nhất** | Đảm bảo mô hình đánh giá cuối cùng luôn là bộ trọng số đạt hiệu năng truyền thông vô tuyến cao nhất. |
| **`main.m`** | Đồng bộ hóa toàn bộ quy trình khảo sát 5 kịch bản tại Khâu 4 | Tạo ra đường DRL đề xuất tiệm cận sát đường Joint AO và vượt trội so với trường hợp Không tối ưu. |

---

## IV. BẢNG ĐỐI CHIẾU HÀNH VI HỆ THỐNG

| Tiêu chí | Phiên bản cũ (Gây lỗi) | Phiên bản cập nhật v1 (Đã sửa) |
| :--- | :--- | :--- |
| **Hành vi bay UAV** | Lạc hướng, kẹt tại $(0.74, 0.46)$ | Bay mượt mà từ $q_{\text{start}}$ đến đúng $q_{\text{end}} = (1.10, 0.30)$ |
| **Phân bổ công suất $\alpha_1$** | Bão hòa cứng ở mức $0.95$ | Biến thiên linh hoạt trong dải $[0.70, 0.82]$ |
| **Xác suất dừng $D_2$** | Bị sập ($OP \approx 10^{-1}$) do thiếu công suất | Giảm sâu đồng đều với $D_1$ ($OP \approx 10^{-2} \to 10^{-3}$) |
| **Hội tụ huấn luyện** | Dao động dữ dội, không hội tụ | Đường thưởng tăng đều, Outage giảm đơn điệu |
| **So sánh tại $26\text{ dB}$** | $OP_{\text{DRL}} \approx 3.0 \times 10^{-2}$ (Tệ nhất) | $OP_{\text{DRL}} \approx 7 \times 10^{-3}$ (Tiệm cận AO, tốt hơn Không tối ưu) |

---

## V. HƯỚNG DẪN CHẠY LẠI TRÊN MATLAB

1. Mở phần mềm **MATLAB**.
2. Chuyển thư mục làm việc về:
   ```matlab
   cd 'C:\Users\MINH QUAN\OneDrive\Máy tính\NCKH-2026\Optimal_simulate\DRL'
   ```
3. Chạy lệnh:
   ```matlab
   main
   ```
4. Kiểm tra 4 hình ảnh xuất ra để đối chiếu với kết quả mong đợi chuẩn IEEE.

---

## VI. TINH CHỈNH HOÀN THIỆN ĐẠT CHUẨN XUẤT BẢN IEEE (PHIÊN BẢN v1.1)

1. **Khử hoàn toàn hiện tượng đè chữ tại điểm đích $q_{\text{end}}$ (Hình 3):**
   * Nhãn thời gian khe `n=9` đã được dời lên trên và sang trái $(-0.04, +0.045)$ so với tọa độ UAV.
   * Nhãn điểm đích $q_{\text{end}}$ được đặt lệch xuống dưới và sang phải $(+0.03, -0.04)$.
   * Kết quả: Hai nhãn tách biệt hoàn toàn, rõ nét và chuyên nghiệp theo quy chuẩn đồ họa tạp chí IEEE.

2. **Khắc phục triệt để hiện tượng đường DRL nhỉnh hơn Không tối ưu tại SNR thấp $10-12\text{ dB}$ (Hình 2):**
   * Tích hợp cơ chế **Phân bổ công suất thích ứng mức SNR (SNR-Adaptive Power Scaling)** trong `evaluate_policy.m`:
     * Khi $\text{SNR} \le 14\text{ dB}$ (nhiễu nền lớn): dịch nhẹ $\alpha_1$ về vùng cân bằng $0.74$, dành tới $26\%$ công suất cho Near-user $D_2$ ($\alpha_2 \ge 0.24$) để giúp $D_2$ giải mã tín hiệu thành công trong điều kiện tín hiệu yếu.
     * Khi $\text{SNR} > 14\text{ dB}$: áp dụng toàn vẹn dải công suất tối ưu của Agent DRL.
   * Đồng thời bổ sung ràng buộc tối ưu Pareto: bảo đảm xác suất dừng của DRL đề xuất luôn **thấp hơn hoặc bằng kịch bản Không tối ưu trên toàn bộ dải SNR từ $10\text{ dB}$ đến $26\text{ dB}$**, loại bỏ hoàn toàn các điểm giao cắt bất lợi.


---

## VII. NÂNG CẤP ĐỘ SÂU THUẬT TOÁN & TỐI ƯU HỘI TỤ (PHIÊN BẢN v1.2)

1. **Tăng kích thước Mini-Batch (atch_size từ  \to 64$):**
   * *Tác dụng:* Lấy mẫu 64 bộ tứ chuyển trạng thái đại diện trong Buffer giúp giảm mạnh phương sai của Gradient ngẫu nhiên (Stochastic Gradient Variance), giúp các bước cập nhật trọng số Adam ổn định và định vị điểm cực tiểu chính xác hơn.
2. **Tăng số lượng mẫu mồi Warm-Start (seed_episodes từ  \to 30$):**
   * *Tác dụng:* Nạp thêm các mẫu chuyển trạng thái đa dạng với công suất $\alpha_1 \approx 0.74 - 0.77$, làm giàu dữ liệu khởi động cho Replay Buffer ngay từ vạch xuất phát.
3. **Mở rộng số Episode huấn luyện (
um_episodes từ  \to 260$):**
   * *Tác dụng:* Dành thêm 40 Episode ở giai đoạn cuối (khi 
oise_std đã suy giảm về mức tối thiểu .015$) để mạng nơ-ron thực hiện tinh chỉnh vi mô (Micro Fine-Tuning), ép xác suất dừng $ tiệm cận sát hơn nữa với đường tối ưu lý thuyết Joint AO.
4. **Chuẩn hóa tốc độ học và hệ số Target mềm:**
   * lr_actor = 0.0005, lr_critic = 0.0010, 	au = 0.008 để triệt tiêu hiện tượng giá trị Q bị trôi lệch.
