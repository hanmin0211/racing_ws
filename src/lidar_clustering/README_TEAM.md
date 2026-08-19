# lidar_clustering - 팀 전달용 소스 패키지

기준 환경
- Ubuntu 22.04
- ROS2 Humble
- Python 3.10
- SL LIDAR A1
- /scan : sensor_msgs/msg/LaserScan
- LiDAR baudrate: 256000

핵심 파이프라인
/scan
 -> ScanPreprocessor
 -> DBSCAN
 -> Hungarian data association / tracking
 -> obstacle feature extraction

동시에 원본 /scan
 -> Follow the Gap
 -> STRAIGHT / LEFT / RIGHT / STOP

차량 좌표계
- +X: 전방
- +Y: 왼쪽
- -Y: 오른쪽

팀원 PC에서 설치 예시

1) 소스를 워크스페이스에 복사
   ~/ros2_ws/src/lidar_clustering

2) 의존성 설치
   sudo apt update
   sudo apt install -y \
     python3-numpy \
     python3-scipy \
     python3-sklearn \
     python3-matplotlib

3) 빌드
   source /opt/ros/humble/setup.bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash

4) LiDAR 드라이버 실행
   ros2 launch sllidar_ros2 sllidar_a1_launch.py \
     serial_port:=/dev/ldlidar \
     serial_baudrate:=256000

   팀원 PC에서는 장치가 /dev/ttyUSB0 등으로 잡힐 수 있으므로 확인 필요.

5) LiDAR 처리 노드 실행
   ros2 run lidar_clustering cluster_plot_node

또는
   ros2 launch lidar_clustering lidar_clustering.launch.py

주의
- build/, install/, log/는 이 ZIP에 포함하지 않음.
- sllidar_ros2 드라이버는 별도 설치가 필요함.
- 실제 차량 통합 시 GPS/제어팀과 토픽 인터페이스를 별도로 합의해야 함.
