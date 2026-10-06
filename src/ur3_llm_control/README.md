# ur3_llm_control

ROS 2 package cho Bài thực hành 02 và 03. Package nhận câu lệnh tiếng Việt hoặc tiếng Anh, yêu cầu Ollama sinh một kế hoạch skill JSON, kiểm tra kế hoạch, rồi thực thi các chuyển động đã lập kế hoạch bằng MoveIt 2 trên UR3/UR3e trong Gazebo.

## MSSV và nhiệm vụ cá nhân

- Sinh viên: **Lục Văn Khoa**
- MSSV: **23020748**
- Hai chữ số cuối: **48**
- **P = 48 mod 6 = 0**
- Ánh xạ cá nhân: **Zone A = red_cube, Zone B = yellow_cube, Zone C = blue_cube**

Thông tin này nằm trong `config/student_config.yaml`. Lệnh “Arrange all objects according to my student ID” được planner diễn giải bằng ánh xạ trong file cấu hình.

## Hai cấu hình bài tập

| Launch argument | Nội dung |
| --- | --- |
| `assignment:=bai2` | UR3e, bàn, 3 block và 3 zone. Pose ban đầu đọc từ `config/scene_bai2.yaml`; không cần camera. |
| `assignment:=bai3` | UR3e, bàn, 5 block, 3 zone, camera nhìn từ trên xuống và các vị trí tạm trên bàn. Nhận dạng block và vị trí được cập nhật từ ảnh camera. |

Mô hình Gazebo của Bài 2 nằm trong `worlds/workcell_bai2.sdf`; Bài 3 nằm trong `worlds/workcell_bai3.sdf`.

## Luồng xử lý

```text
/task_command
    -> Ollama (chỉ tạo skill plan JSON)
    -> TaskValidator (schema, whitelist, object, zone, thứ tự và trạng thái chiếm chỗ)
    -> /validated_plan
    -> SkillExecutor (xác thực lại)
    -> pick / place / place_temp / home
    -> MoveIt 2 plan + execute
    -> UR3e + suction gripper trong Gazebo
```

LLM không tạo góc khớp, joint trajectory hay lệnh điều khiển mức thấp. Chỉ các skill trong whitelist mới có thể chạy. Kế hoạch sai skill, object, zone, schema hoặc thứ tự bị từ chối. Mỗi chuyển động được lập kế hoạch và thực thi qua MoveIt 2; mô hình UR cung cấp joint limit và MoveIt kiểm tra self-collision cùng collision objects trong PlanningScene.

Bài 2 nạp vị trí từ tham số scene tĩnh. Bài 3 nhận ảnh từ `/camera/image`, dò màu bằng OpenCV rồi chiếu pixel sang mặt bàn theo thông số camera trong `scene_bai3.yaml`. Node xuất trạng thái hiện thấy tại `/world_state`; planner và executor dùng trạng thái này để quyết định block nào đang ở đâu và zone nào đang trống. Nếu zone đích có vật, validator chèn các bước dọn vật đó sang một temporary slot trống trước khi đặt vật được yêu cầu.

Gripper mô phỏng là suction cup gắn trên robot. Gazebo tạo hoặc tháo detachable joint giữa suction cup và block khi skill gắp/thả chạy. Block vẫn là entity vật lý trong Gazebo; phần mềm không đặt pose block để giả lập gắp.

## Cài đặt

Môi trường tham chiếu là ROS 2 Humble, Gazebo Fortress, UR simulation, MoveIt 2 và ros_gz. Cài các dependency ROS còn thiếu bằng rosdep:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ur3_llm_control
source install/setup.bash
```

Planner dùng Ollama API và model mặc định `qwen2.5:0.5b`. Cài Ollama theo hướng dẫn của môi trường, sau đó tải model và chạy server trong một terminal:

```bash
ollama pull qwen2.5:0.5b
ollama serve
```

Nếu ROS chạy trong WSL và Ollama chạy ngoài WSL, truyền URL Ollama có thể truy cập từ WSL bằng tham số `ollama_url`.

## Chạy Bài 2

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch ur3_llm_control llm_robot.launch.py assignment:=bai2 ur_type:=ur3e
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub --once /task_command std_msgs/msg/String \
  "{data: 'Put the red cube in zone B'}"
```

Các câu lệnh khác có thể diễn đạt tự nhiên, ví dụ:

```bash
ros2 topic pub --once /task_command std_msgs/msg/String \
  "{data: 'Hãy lấy khối màu vàng và đặt vào vùng A'}"
```

Terminal planner hiển thị user command, LLM plan, kế hoạch sau validation và trạng thái từng skill. Có thể xem node/topic bằng `ros2 node list` và `ros2 topic list`.

## Chạy Bài 3 và demo zone bị chiếm

Khởi chạy Bài 3:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch ur3_llm_control llm_robot.launch.py assignment:=bai3 ur_type:=ur3e
```

Trong scene mẫu, Zone B đang có `blue_cube`. Gửi yêu cầu:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub --once /task_command std_msgs/msg/String \
  "{data: 'Put the red cube in Zone B'}"
```

Camera cung cấp trạng thái hiện tại. Kế hoạch hợp lệ sẽ chuyển block đang chiếm Zone B sang temporary slot trống, sau đó đặt `red_cube` vào Zone B và gọi `home()`. Có thể chạy demo nhiệm vụ cá nhân bằng:

```bash
ros2 topic pub --once /task_command std_msgs/msg/String \
  "{data: 'Arrange all objects according to my student ID'}"
```

Trong demo này, block ở Zone B/C phải được sắp xếp lại theo ánh xạ MSSV 23020748. Bài 3 không dùng các pose ban đầu trong YAML để thay camera: tọa độ x/y của block được phát hiện từ ảnh.

## Kiểm tra

Chạy kiểm tra validator, tình huống zone bị chiếm, màu camera và phép chiếu pixel:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
PYTHONPATH=src/ur3_llm_control python3 -m unittest discover \
  -s src/ur3_llm_control/test -v
```

Các tham số scene nằm trong `config/scene_bai2.yaml` và `config/scene_bai3.yaml`. Cấu hình MSSV nằm trong `config/student_config.yaml`. Có thể đổi robot bằng `ur_type:=ur3` hoặc `ur_type:=ur3e`, đổi model Ollama bằng `model:=...`, và URL API bằng `ollama_url:=...`.

Tốc độ MoveIt được scale thấp trong scene config. Nếu đổi kích thước bàn, tọa độ object/zone, camera hoặc vị trí robot, hãy cập nhật scene và kiểm tra lại vùng làm việc cùng collision scene trong Gazebo trước khi demo.
