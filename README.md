# Robot — Firmware V1.0

Cây thư mục khởi tạo theo bản thiết kế chốt ngày 26/11/2026.

## Trạng thái

Chỉ là skeleton thư mục, chưa có firmware C, project CubeMX/Keil, cấu hình ngoại vi, thư viện HAL/FreeRTOS, model MuJoCo hay kết quả build/test. Không nạp trực tiếp lên MCU.

## Quy trình phát triển

1. Giải nén Robot/ vào vị trí làm việc và mở thư mục Robot/ bằng VS Code (Open Folder). Codex làm việc trên cùng các file nguồn C.
2. Mở STM32CubeMX, chọn STM32F407VET6. Cấu hình clock, timer, PWM, encoder, GPIO, UART/DMA và FreeRTOS theo pinout đã xác minh.
3. Trong Project Manager, chọn MDK-ARM nếu muốn build bằng Keil. Đặt vị trí project sao cho các file CubeMX sinh nằm trong Firmware/ (kiểm tra đường dẫn thực tế trước khi Generate Code, tránh Firmware/Robot/ lồng ngoài ý muốn). Nếu chọn STM32CubeIDE làm toolchain thì sinh project CubeIDE, không phải project Keil.
4. Mở project .uvprojx do CubeMX sinh trong Keil để build/debug/flash; tiếp tục viết C trong VS Code trên cùng thư mục, không copy source giữa IDE. Thêm các file .c và include paths của module tự viết vào project Keil khi triển khai; thêm riêng startup/HAL/FreeRTOS theo cấu hình generator. Không đưa Platform/Simulation hay test PC vào build MCU.
5. Giữ code tự viết ở các module riêng; chỉ sửa file CubeMX sinh trong USER CODE sections nếu có thể. Sao lưu/commit trước khi generate lại và kiểm tra source groups/include paths sau khi generate.
6. Kiểm thử SIL: dùng native C compiler + CMake để build phần lõi thành DLL (Windows) hoặc SO (Linux); Python ctypes nối với MuJoCo qua Platform/Simulation. Không dùng binary ARM của Keil cho PC. Không compile BSP STM32 và FreeRTOS target vào build PC. HIL là bước bổ sung, chưa bắt buộc.

## Kiến trúc

- Firmware/BSP: ngoại vi MCU.
- Firmware/Device: driver thiết bị MDD3A, encoder, servo, limit switch.
- Firmware/Algorithm: PID, filter, interpolation độc lập phần cứng.
- Firmware/Controller: điều khiển khớp và quỹ đạo.
- Firmware/Application: state machine, homing, safety, robot, gripper.
- Firmware/Communication: protocol, frame, CRC, parser, command, telemetry.
- Firmware/Tasks: FreeRTOS và đồng bộ dữ liệu.
- Firmware/Platform: interface chung, adapter STM32 và simulation.
- Firmware/Config: tham số.
- Firmware/Tests: unit, integration, HostTools.
- Simulation: model MuJoCo, bridge C/Python, scenarios, analysis, kết quả.

Các thư mục Core/, Drivers/, Middlewares/ và project .ioc/.uvprojx chưa được tạo: để CubeMX sinh đúng theo cấu hình thực tế. Tên/vị trí file sinh phụ thuộc phiên bản và toolchain. Các file module .c/.h, CMakeLists.txt và model XML sẽ được tạo khi triển khai, không tạo placeholder rỗng.

Bản context thiết kế được lưu nguyên trạng tại Readme/Context/CONTEXT_CHATGPT_20261126_213034.md. README này bổ sung lựa chọn quy trình soạn thảo VS Code/Codex + CubeMX + Keil, không sửa giao thức UART hay kiến trúc đã chốt. Pinout, nguồn, encoder, thông số PID, timing và chính sách dừng an toàn vẫn cần kiểm chứng. Không dùng CAN/ROS 2/micro-ROS.

Các thư mục trống được giữ bằng directory entries trong ZIP. Nếu dùng Git, thêm .gitkeep vào những thư mục cần theo dõi cho tới khi có file thật.
