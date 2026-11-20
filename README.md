# SAGIN-NOMA Optimization & Simulation Project

Hệ thống tối ưu hóa và mô phỏng mạng tích hợp Không gian - Vũ trụ - Mặt đất (**SAGIN**) sử dụng công nghệ Đa truy cập phi trực giao (**NOMA**).

Dự án bao gồm hai thành phần chính:
1. **Mô hình mô phỏng thuật toán tối ưu hóa (AO & DRL)**
2. **Ứng dụng mô phỏng trực quan (MATLAB App Designer)**

---

## Cấu trúc dự án

```
MyProject/
├── Optimal_NOMA_SAGIN_model/     # Mô hình toán học & thuật toán tối ưu hóa
│   ├── core/                     # Thuật toán cốt lõi (AO, DRL, Utils)
│   ├── assets/                   # Đồ thị và hình ảnh kết quả mô phỏng
│   ├── docs/                     # Báo cáo chuyên môn & tài liệu lý thuyết
│   ├── LECTURE NOTES/            # Báo cáo học thuật LaTeX (Bản tiếng Anh)
│   ├── LECTURE NOTES (Vie)/      # Báo cáo học thuật LaTeX (Bản tiếng Việt)
│   ├── run_all.m                 # Script chạy toàn bộ mô phỏng
│   └── test_engine.m             # Script kiểm thử hệ thống
│
├── SAGIN_NOMA_Simulator_APP/     # Ứng dụng mô phỏng trực quan
│   ├── SAGIN_NOMA_App.mlapp      # Giao diện chính MATLAB App Designer
│   ├── start_app.m               # Script khởi chạy ứng dụng
│   ├── core/                     # Động cơ tính toán của App
│   ├── data/                     # Dữ liệu mô hình DRL đã huấn luyện sẵn
│   └── exports/                  # Thư mục xuất dữ liệu & đồ thị
│
├── .gitignore                    # Cấu hình bỏ qua file tạm (MATLAB, LaTeX)
└── README.md                     # Tài liệu giới thiệu dự án
```

---

## Hướng dẫn cài đặt & Khởi chạy

### Yêu cầu hệ thống
- MATLAB R2022b trở lên.
- Toolboxes: *Optimization Toolbox*, *Deep Learning Toolbox* / *Reinforcement Learning Toolbox* (nếu chạy huấn luyện DRL).

### 1. Khởi chạy Mô phỏng (Simulation Core)
Mở MATLAB, chuyển đường dẫn vào thư mục `Optimal_NOMA_SAGIN_model` và chạy:
```matlab
run_all
```

### 2. Khởi chạy Ứng dụng (GUI App)
Mở MATLAB, chuyển đường dẫn vào thư mục `SAGIN_NOMA_Simulator_APP` và chạy:
```matlab
start_app
```
hoặc mở trực tiếp file `SAGIN_NOMA_App.mlapp`.
