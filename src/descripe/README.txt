# Delta 4DOF Workspace Setup

README này hướng dẫn tải workspace `delta_4dof`, build ROS 2 workspace và tạo shortcut chạy mô phỏng/GUI trên Ubuntu.

## 1. Tải workspace từ GitHub

```bash
cd ~
git clone https://github.com/ehe55555/delta_4dof.git
cd ~/delta_4dof
```

Nếu đã tải workspace trước đó và chỉ muốn cập nhật bản mới:

```bash
cd ~/delta_4dof
git pull
```

## 2. Cài các gói cần thiết

```bash
sudo apt update
sudo apt install -y \
  git \
  python3-colcon-common-extensions \
  python3-tk \
  python3-numpy \
  python3-scipy \
  python3-matplotlib \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  gz-harmonic \
  libgz-sim8-dev \
  libgz-plugin2-dev \
  libgz-transport13-dev \
  libgz-msgs10-dev
```

## 3. Build workspace

```bash
cd ~/delta_4dof
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Nếu cần build lại từ đầu:

```bash
cd ~/delta_4dof
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 4. Cấp quyền chạy script

```bash
cd ~/delta_4dof
chmod +x run_gazebo.sh
chmod +x run_delta_control.sh
```

## 5. Chạy workspace

Chạy mô phỏng Gazebo + bridge + GUI điều khiển:

```bash
cd ~/delta_4dof
./run_delta_control.sh
```

Chỉ chạy Gazebo để kiểm tra model:

```bash
cd ~/delta_4dof
./run_gazebo.sh
```

## 6. Tạo shortcut mở Delta Control

Tạo file shortcut ngoài Desktop:

```bash
cat > ~/Desktop/Delta_4DOF_Control.desktop <<'EOF'
[Desktop Entry]
Type=Application
Name=Delta 4DOF Control
Comment=Run Delta 4DOF Gazebo simulation and control GUI
Exec=bash -lc "cd ~/delta_4dof && ./run_delta_control.sh"
Icon=utilities-terminal
Terminal=true
Categories=Development;Robotics;
EOF

chmod +x ~/Desktop/Delta_4DOF_Control.desktop
gio set ~/Desktop/Delta_4DOF_Control.desktop metadata::trusted true 2>/dev/null || true
```

Sau đó ra Desktop, nếu Ubuntu hỏi quyền chạy thì chọn:

```text
Allow Launching
```

## 7. Tạo shortcut chỉ mở Gazebo

```bash
cat > ~/Desktop/Delta_4DOF_Gazebo.desktop <<'EOF'
[Desktop Entry]
Type=Application
Name=Delta 4DOF Gazebo
Comment=Run Delta 4DOF Gazebo world only
Exec=bash -lc "cd ~/delta_4dof && ./run_gazebo.sh"
Icon=utilities-terminal
Terminal=true
Categories=Development;Robotics;
EOF

chmod +x ~/Desktop/Delta_4DOF_Gazebo.desktop
gio set ~/Desktop/Delta_4DOF_Gazebo.desktop metadata::trusted true 2>/dev/null || true
```

## 8. Cập nhật code lên GitHub

Sau khi sửa code:

```bash
cd ~/delta_4dof
git status
git add .
git commit -m "Update workspace"
git push
```

## 9. Lỗi thường gặp

Nếu chạy không thấy package:

```bash
cd ~/delta_4dof
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg list | grep delta
```

Nếu sửa code Python nhưng chạy chưa cập nhật:

```bash
cd ~/delta_4dof
colcon build --symlink-install --packages-select delta_control
source install/setup.bash
./run_delta_control.sh
```

Nếu Gazebo không load plugin:

```bash
cd ~/delta_4dof
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
./run_delta_control.sh
```
