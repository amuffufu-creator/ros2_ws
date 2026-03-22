Project overview
1) Project Overview

Author: Sinh viên năm cuối ngành Điện – Điện tử (EEE), định hướng điều khiển/tự động hoá.
Goal: Chế tạo AGV 3 bánh (differential drive) dùng ESP32 + ROS 2 Jazzy để SLAM 2D và điều hướng tự động (Nav2).
Algorithms:
SLAM: slam_toolbox (hoặc tương đương) tạo bản đồ 2D từ LiDAR.
Localization: AMCL (khi dùng map có sẵn).
State Estimation: EKF (robot_localization) fuse wheel odom + IMU → /odometry/filtered + TF.

2) System Environment

Host OS: Windows (máy thật).
Guest OS: Ubuntu 24.04.x LTS chạy trên VMware Workstation 16.
ROS Version: ROS 2 Jazzy.
IDE/Tools:
VS Code
PlatformIO (ESP32 firmware)
colcon (build ROS2 workspace)
RViz2 (visualization)
Languages:
ESP32 firmware: C/C++ (Arduino framework trên PlatformIO)
ROS2 nodes: Python (hiện tại), có thể mở rộng C++ sau.

3) Hardware Components

Robot base: 3 bánh
2 bánh sau gắn động cơ (left/right) + encoder → differential drive
1 bánh trước caster (tự do), không điều khiển
MCU: ESP32 DevKit v1 (PlatformIO)
Sensors:
LiDAR: RPLIDAR A1M8 (đọc UART trên ESP32)
IMU: BNO055 (I2C, địa chỉ đang dùng 0x29)
Encoders: 2 encoder cho 2 bánh sau
Actuators:
2 DC motor + TB6612FNG
Power: tách nguồn motor/logic là lợi thế (nhiễu từ motor ảnh hưởng IMU là rủi ro cần lưu ý)

4) Communication Architecture (Wi-Fi + Hybrid TCP/UDP)

ESP32 ↔ PC (Ubuntu VM) qua Wi-Fi.
LiDAR data channel: TCP
ESP32 làm TCP server (port 20108)
PC chạy sllidar_ros2 ở chế độ TCP client để nhận stream scan
Control + Encoder + IMU: UDP
ESP32 lắng nghe lệnh điều khiển qua UDP port 8889 (PC → ESP32)
ESP32 gửi encoder ticks/odom raw về PC port 8888 (ESP32 → PC)
ESP32 gửi IMU raw về PC port 8890 (ESP32 → PC)
Data Flow (ESP32 → PC):
Encoder ticks (UDP) + IMU quaternion/gyro/accel (UDP)
LiDAR scan stream (TCP)
Data Flow (PC → ESP32):
Lệnh tốc độ (UDP): dạng s,vA,vB hoặc quy đổi từ /cmd_vel tùy node điều khiển
PC ROS2 responsibilities:
Publish /scan, /imu/data, /wheel/odom (hoặc tương đương)
EKF → publish /odometry/filtered + TF odom->base_link
SLAM/AMCL → TF map->odom
Nav2 tiêu thụ TF + topics để điều hướng

5) Rules / Constraints (for Assistant & Project)

Brevity: ưu tiên checklist + lệnh chạy + chẩn đoán theo bằng chứng.
ROS2 Jazzy compliance: tham số/launch theo Jazzy.
PlatformIO structure: firmware theo src/, platformio.ini, tránh code “Arduino IDE style” rời rạc.
TF rule (critical): mỗi transform chỉ có 1 publisher
Khi dùng EKF: udp_bridge_node không được broadcast odom->base_link.
VMware Networking caveats:
UDP có thể bị ảnh hưởng NAT/bridged mode, latency/jitter.
Debug theo hướng “tcpdump/ss/nc” trước khi đổ lỗi ROS.
----
trên đây là project overview
----------
Duoi day la project plan
Don vi deu duoc hien thi duoi dang millimet (mm)
Giai đoạn 1: Hardware Bringup và URDF (Xây dựng nền tảng)

Mục tiêu của giai đoạn này là khởi động giao tiếp giữa PC và ESP32, đảm bảo dữ liệu cảm biến được đưa lên ROS 2 chính xác.

1.1. Cấu hình URDF (agv.urdf)

Thiết lập base_link tại tâm trục hai bánh sau (điểm P).

Gắn LiDAR TF
Định nghĩa laser_frame với: <origin xyz="0.23 -0.01 0.22" rpy="0 0 0"/>

Gắn IMU TF
Định nghĩa imu_link với: <origin xyz="0.14 0.06 0.06" rpy="0 0 0"/>

1.2. Khởi chạy Bridge Nodes

Sử dụng mạng VMware ở chế độ Bridged Mode để PC (Ubuntu) và ESP32 nhận IP cùng dải.

Chạy udp_bridge_node để nhận dữ liệu encoder và gửi lệnh điều khiển qua UDP.
Cấu hình tham số:

* wheel_radius: 0.0636
* wheel_base: 0.285
* ticks_per_rev: 330
* publish_tf: False

Lưu ý: Đây là quy tắc bắt buộc khi dùng EKF để tránh xung đột TF.

Chạy imu_udp_bridge để nhận dữ liệu IMU BNO055 qua UDP port 8890.

Chạy node sllidar_ros2 ở chế độ TCP client, trỏ tới IP của ESP32 (port 20108).

1.3. Checklist kiểm tra (Giai đoạn 1)

* Chạy `ros2 topic hz /scan /imu/data /odom` để xác nhận tần số dữ liệu.
* Chạy lệnh `ros2 run tf2_tools view_frames`.
* Đảm bảo cây TF chỉ có:

  * base_link -> laser_frame
  * base_link -> imu_link
* Tuyệt đối không có odom ở bước này.

Giai đoạn 2: State Estimation (Cấu hình EKF)

Mục tiêu là kết hợp odometry từ bánh xe và dữ liệu IMU để tạo ra vị trí ước tính chính xác, giảm thiểu hiện tượng trượt bánh (drift).

2.1. Cấu hình ekf.yaml cho robot_localization

Input 1 (/odom)
Cấp dữ liệu:

* Vận tốc tuyến tính trục X (vx)
* Vận tốc góc quanh trục Z (wz)
  Từ bánh xe.

Input 2 (/imu/data)
Cấp dữ liệu:

* Góc xoay (yaw/orientation)
* Vận tốc góc quanh trục Z (wz)

2.2. Xuất bản TF và Topics

Cấu hình EKF để xuất bản:

* Topic `/odometry/filtered`
* TF `odom -> base_link`

2.3. Checklist kiểm tra (Giai đoạn 2)

* Mở RViz2, chọn Fixed Frame là `odom`.
* Hiển thị RobotModel.
* Đẩy xe bằng tay qua lại và quan sát mô hình 3D di chuyển mượt mà, phản hồi đúng góc xoay thực tế.

Giai đoạn 3: Mapping và SLAM 2D

Mục tiêu là điều khiển xe chạy quanh môi trường để vẽ bản đồ tĩnh dùng slam_toolbox.

3.1. Khởi chạy SLAM Toolbox

Sử dụng chế độ `async_slam_toolbox_node` (phù hợp với môi trường thực tế).

Đảm bảo node nhận đủ:

* TF `odom -> base_link` từ EKF
* Topic `/scan` từ LiDAR

Node này sẽ xuất bản TF `map -> odom`.

3.2. Quét bản đồ

* Chạy node teleop (`teleop_twist_keyboard` hoặc tay cầm) để điều khiển xe đi qua các ngóc ngách của phòng.
* Mở RViz2, thêm display Map (topic `/map`) để xem bản đồ đang hình thành.

3.3. Lưu bản đồ

Khi bản đồ đã hoàn thiện, chạy lệnh:

`ros2 run nav2_map_server map_saver_cli -f my_map`

Bạn sẽ thu được hai file:

* `my_map.yaml`
* `my_map.pgm`

Giai đoạn 4: Autonomous Navigation (Nav2 và AMCL)

Mục tiêu là định vị robot trong bản đồ đã lưu (AMCL) và tự động tính toán đường đi, tránh chướng ngại vật (Nav2).

4.1. Khai báo Robot Footprint trong Nav2

Mở file cấu hình thông số của Nav2 (thường là `nav2_params.yaml`).

Khai báo tọa độ bao phủ vào mục `footprint` ở cả:

* Global Costmap
* Local Costmap

Cấu hình footprint:
`[[0.30, 0.13], [0.30, -0.13], [-0.035, -0.13], [-0.035, 0.13]]`

4.2. Khởi chạy AMCL và Nav2 Stack

Khởi chạy Nav2 với bản đồ vừa lưu:

`ros2 launch nav2_bringup bringup_launch.py map:=my_map.yaml params_file:=nav2_params.yaml`

AMCL sẽ đảm nhận việc so khớp dữ liệu LiDAR với bản đồ tĩnh để xuất bản TF `map -> odom` (thay thế cho slam_toolbox).

4.3. Checklist kiểm tra (Giai đoạn 4)

* Trên RViz2, sử dụng công cụ "2D Pose Estimate" để báo cho hệ thống biết vị trí khởi điểm thực tế của AGV.
* Sử dụng công cụ "Nav2 Goal" để trỏ một điểm đích.
* Quan sát Local và Global costmap phình to quanh các vách tường (dựa theo kích thước footprint) và xe tự động tính toán `/cmd_vel` truyền xuống ESP32.
