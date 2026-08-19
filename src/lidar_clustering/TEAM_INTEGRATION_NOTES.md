# GPS / 제어팀 통합 시 확인할 항목

1. ROS2 배포판
   echo $ROS_DISTRO

2. Ubuntu 버전
   lsb_release -a

3. GPS팀 Publish 토픽
   ros2 topic list
   ros2 topic info <topic>

4. 제어팀 Subscribe 토픽 및 메시지 타입
   ros2 topic info <topic> -v

5. 공통 좌표계
   +X = 차량 전방
   +Y = 차량 왼쪽
   -Y = 차량 오른쪽

6. 공통 단위
   거리: m
   속도: m/s 또는 km/h 중 하나로 통일
   조향각: rad 또는 degree 중 하나로 통일

현재 lidar_clustering 노드는 회피 결과를 로그/그래프로 표시한다.
통합 단계에서는 다음과 같은 ROS2 Publish 인터페이스 추가를 권장한다.

- /lidar/avoidance
- /lidar/obstacles

최종적으로 Decision/Planner 노드에서
GPS 경로 + LiDAR 장애물 정보를 융합하고,
제어팀이 요구하는 목표 속도/조향 명령으로 변환한다.
