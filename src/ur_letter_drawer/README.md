# UR3/UR3e draws K with MoveIt 2

Package này mô phỏng robot UR3e (mặc định) hoặc UR3 trên ROS 2 Humble. Node dùng
MoveIt 2 để đi tới đầu mỗi nét khi đầu công tác được nâng, hạ đầu công tác theo
Cartesian, rồi vẽ nét bằng Cartesian waypoints. Chữ K nằm trong mặt phẳng XY của
`base_link`, tại `z = 0.34 m`, kích thước khoảng `12 x 18 cm`.

## Build và chạy

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur_letter_drawer
source install/setup.bash
ros2 launch ur_letter_drawer draw_k_sim.launch.py
```

Đổi sang UR3 bằng lệnh:

```bash
ros2 launch ur_letter_drawer draw_k_sim.launch.py ur_type:=ur3
```

The writing plane height can be changed with `writing_plane_z:=0.34` (meters).

Launch mở Gazebo, MoveIt 2 và RViz. Node tự bắt đầu sau khi các thành phần mô
phỏng có thời gian khởi động. RViz hiển thị đường chữ K trên topic
`/letter_path`; robot chuyển động theo quỹ đạo MoveIt trong RViz/Gazebo.

## Waypoint và an toàn

Chữ gồm ba nét: trục dọc, chéo lên và chéo xuống. Giữa các nét, node nâng tool
4 cm rồi dùng MoveIt để lập kế hoạch tới đầu nét tiếp theo. Mỗi đoạn Cartesian
dùng bước lấy mẫu 2 mm, bật kiểm tra va chạm và bị từ chối nếu MoveIt chỉ tìm
được một phần quỹ đạo. Các đoạn di chuyển khác dùng planner OMPL qua
`MoveGroupInterface`; giới hạn khớp, collision scene và self-collision được
MoveIt kiểm tra trước khi thực thi. Tốc độ và gia tốc được giới hạn ở 5%.

Nếu kế hoạch không đạt toàn bộ đoạn hoặc controller thực thi lỗi, node dừng
việc vẽ và ghi nguyên nhân ra terminal. Khả năng tới được các waypoint cuối
cùng phụ thuộc vào trạng thái ban đầu, phiên bản URDF/MoveIt và môi trường mô
phỏng đang chạy.
