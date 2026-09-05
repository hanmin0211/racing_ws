# RTK-GNSS 기반 저비용 자율주행 플랫폼의 실시간 측위 시스템 구현

**A Real-Time Localization System for a Low-Cost Autonomous Driving Platform Based on RTK-GNSS**

---

## 요약 (국문초록)

자율주행 차량의 경로 추종 성능은 측위 정확도에 직접적으로 의존한다. 본 연구는 저비용 소형 전동 차량을 자율주행 플랫폼으로 개조하고, RTK-GNSS(Real-Time Kinematic Global Navigation Satellite System)를 기반으로 한 실시간 측위 시스템을 구현한 사례를 다룬다. 국토지리정보원(NGII)의 VRS(Virtual Reference Station) 보정 서비스를 NTRIP 프로토콜로 수신하여 cm급 측위를 확보하였으며, 이 과정에서 기존 ROS 2 패키지가 지원하지 않는 GGA 상향 전송 문제를 해결하기 위해 전용 NTRIP 클라이언트를 구현하였다. 또한 자력계 오차로 지도 좌표계에 정렬되지 않는 IMU 방위각 문제를 해결하기 위해, 직진 구간의 GNSS 이동 방향(course)을 기준으로 방위각 오프셋을 추정하는 초기화 기법을 제안하였다. 측위 신호 두절 시 마지막 위치를 유지하는 대신 발행 자체를 중단하여 하위 제어기가 안전 정지하도록 설계함으로써 시스템 안전성을 확보하였다. 구현된 시스템은 총연장 190.4 m 시험 구간에서 자율 완주를 달성하였다.

**주제어:** RTK-GNSS, NTRIP, VRS, 자율주행, 측위, 방위각 초기화, ROS 2

---

## ABSTRACT

The path-tracking performance of an autonomous vehicle depends directly on its localization accuracy. This paper presents a real-time localization system based on RTK-GNSS (Real-Time Kinematic Global Navigation Satellite System) implemented on a low-cost autonomous driving platform converted from a small electric vehicle. Centimeter-level positioning was obtained by receiving Virtual Reference Station (VRS) corrections from the National Geographic Information Institute over the NTRIP protocol; because the NTRIP client bundled with the existing ROS 2 package does not transmit the NMEA GGA sentences that the VRS service requires to generate corrections for the rover's location, a dedicated client providing this capability was implemented so that the receiver converges to an RTK Fixed solution. To resolve the misalignment between the map coordinate frame and the yaw angle of the low-cost IMU, which suffers from magnetometer error, we propose an initialization method that estimates the yaw offset by treating the GNSS course over a straight-line segment as the true heading; although the absolute heading is unknown at initialization time, the change in heading is accurately measured by the IMU, so a closed-loop controller that maintains the initial yaw keeps the vehicle straight without knowing the absolute heading, which markedly improved initialization reliability over an open-loop approach in which steering-center offset curved the trajectory and repeatedly failed the straightness check. For safety, the system stops publishing localization output when sensor data is interrupted, rather than holding the last known pose, so that downstream modules detect the condition by timeout and bring the vehicle to a safe stop, and a curvature-constrained smoothing step removes path segments tighter than the vehicle's minimum turning radius while bounding the deviation from the recorded path. The implemented system achieved autonomous completion of a 190.4 m outdoor test course.

**Keywords:** RTK-GNSS, NTRIP, VRS, autonomous driving, localization, heading initialization, ROS 2

---

## 1. 서론

### 1.1 연구 배경

자율주행 시스템은 인지(perception), 측위(localization), 판단·제어(planning and control)의 세 축으로 구성되며, 이 중 측위는 차량이 지도상 어느 위치에 있는지를 결정하는 기반 기능이다. 측위 오차는 그대로 경로 추종 오차로 전이되므로, 차선 폭이 좁은 주행 환경에서는 수십 cm 수준의 오차도 차선 이탈로 이어질 수 있다.

측위 방식은 크게 GNSS 기반, LiDAR/카메라 기반 SLAM, 그리고 이들의 융합으로 구분된다. SLAM 기반 방식은 GNSS 음영 구간에서도 동작한다는 장점이 있으나 사전 지도 구축과 상당한 연산 자원을 요구한다. 반면 개활지 주행에서는 RTK-GNSS만으로 cm급 정확도를 얻을 수 있어, 저비용 플랫폼에서 실용적인 선택지가 된다.

본 연구는 유아용 전동 차량(HENES T870)을 개조한 저비용 자율주행 플랫폼을 대상으로, RTK-GNSS를 중심으로 한 실시간 측위 시스템을 구현하고 실차 주행으로 검증한 사례를 보고한다.

### 1.2 연구 목적 및 기여

본 연구의 기여는 다음과 같다.

1. **VRS 기반 RTK 보정 신호 수신 체계 구현**: 국내 NGII VRS 서비스는 이동체의 개략 위치를 GGA 문장으로 수신해야 해당 지점의 가상 기준국 보정 신호를 생성한다. 그러나 널리 쓰이는 ROS 2 NTRIP 클라이언트는 GGA 상향 전송을 지원하지 않는다. 본 연구는 이 기능을 포함한 전용 클라이언트를 구현하여 RTK Fixed 수렴을 확보하였다.

2. **GNSS course 기반 방위각 초기화 기법**: 저가형 IMU의 방위각은 자력계 오차로 지도 좌표계와 정렬되지 않는다. 본 연구는 직진 구간의 GNSS 이동 방향을 참값으로 간주하여 IMU 방위각과의 오프셋을 추정하고, 이 과정에서 방위각 **변화량**만으로 직진을 유지하는 폐루프 제어를 적용하여 초기화 신뢰성을 높였다.

3. **측위 신호 두절에 대한 안전 지향 설계**: 센서 두절 시 마지막 위치를 계속 발행하면 차량은 이동하는데 위치는 고정되어 하위 제어기가 이상을 감지하지 못한다. 본 연구는 발행 자체를 중단시켜 하위 제어기가 타임아웃으로 안전 정지하도록 설계하였다.

4. **차량 기구학 제약을 반영한 경로 후처리**: 기록된 주행 경로에 차량 최소 회전반경보다 급한 구간이 존재하면 측위와 제어가 정확해도 이탈이 발생한다. 본 연구는 원 경로로부터의 이탈량을 제한한 곡률 완화 기법을 적용하였다.

---

## 2. 관련 연구 및 이론적 배경

### 2.1 GNSS 측위 정확도

단독 측위(standalone)는 전리층·대류권 지연, 위성 궤도 오차 등으로 수 m 수준의 오차를 갖는다. DGPS(Differential GPS)는 기준국의 관측 오차를 이동국에 전달하여 m 이하로 개선하나, 여전히 차선 수준 주행에는 부족하다.

RTK 방식은 반송파 위상(carrier phase) 관측치의 미지정수(integer ambiguity)를 해결하여 cm급 정확도를 달성한다. 미지정수가 정수로 확정된 상태를 RTK Fixed, 실수 상태로 남은 경우를 RTK Float라 하며, 두 상태의 정확도 차이는 수 cm 대 수십 cm로 크다. 따라서 실주행에서는 Fixed 상태 유지 여부가 측위 품질을 좌우한다.

### 2.2 NTRIP과 VRS

NTRIP(Networked Transport of RTCM via Internet Protocol)은 RTCM 보정 신호를 인터넷으로 전송하는 표준 프로토콜이다. 단일 기준국(single-base) 방식은 기선(baseline) 길이가 길어질수록 정확도가 저하된다.

VRS는 다수 상시관측소의 관측치를 이용해 이동체 근처에 가상의 기준국을 생성하는 방식으로, 기선 길이 문제를 완화한다. 그러나 가상 기준국을 생성하려면 **캐스터가 이동체의 개략 위치를 알아야 하며**, 이는 NMEA GGA 문장을 이동체가 캐스터로 주기적으로 전송함으로써 이루어진다. 이 상향 전송이 없으면 보정 신호가 내려오지 않거나 부정확한 위치 기준으로 생성된다.

### 2.3 GNSS 기반 방위각 추정

단일 안테나 GNSS는 위치는 제공하나 차량의 방위각(heading)을 직접 제공하지 않는다. 방위각 획득 방법으로는 (a) 이중 안테나 GNSS(moving base), (b) IMU 자력계, (c) 이동 방향으로부터의 추정이 있다.

이중 안테나 방식은 정확하나 안테나 이격 거리와 추가 수신기가 필요하다. 자력계는 주변 금속·모터 자기장의 영향을 크게 받으며, 특히 모터가 근접한 소형 플랫폼에서는 오차가 크다. 본 연구는 (c) 방식을 채택하되, 초기화 시점에만 사용하고 이후에는 IMU 각속도 적분으로 방위각을 추종하는 하이브리드 구조를 적용하였다.

---

## 3. 시스템 구성

### 3.1 하드웨어 구성

| 구성 요소 | 사양 | 비고 |
|---|---|---|
| 차량 플랫폼 | HENES T870, 축거 0.785 m | 유아용 전동차 개조 |
| GNSS 수신기 | u-blox ZED-F9P | L1/L2 다중 대역, RTK 지원 |
| IMU | HandsFree A9 | 300 Hz |
| 제어 MCU | Arduino Mega 2560 | 하위 제어(구동·조향) |
| 상위 연산 | 노트북 PC (ROS 2 Humble) | 측위·경로계획·제어 |

차량 제원 중 축거는 자전거 모델(bicycle model) 기반 조향각 산출에 직접 사용되므로 실측하였다(0.785 m).

### 3.2 소프트웨어 아키텍처

시스템은 ROS 2 Humble 기반으로 구성되며, 측위 관련 데이터 흐름은 다음과 같다.

```
[NGII VRS 캐스터]
    ↑ GGA(위치 보고)     ↓ RTCM3(보정 신호)
[vrs_ntrip_client] ──/ntrip_client/rtcm──> [ublox_dgnss_node] ──USB──> [ZED-F9P]
                                                    │
                                                    ↓ /fix (NavSatFix)
[handsfree IMU] ──/imu──┐                          │
                        ↓                          ↓
              [heading_init] ──/heading/yaw_offset──┐
                                                    ↓
                                     [direct_localization_node]
                                                    ↓
                                    /odometry/filtered (map 프레임)
                                                    ↓
                              [경로 계획 · 추종 · 하위 제어]
```

각 노드의 역할은 다음 절에서 상술한다.

---

## 4. RTK 보정 신호 수신

### 4.1 GGA 상향 전송의 필요성

VRS 서비스는 이동체 위치 기준으로 가상 기준국을 생성하므로, 이동체가 자신의 개략 위치를 캐스터에 알려야 한다. 그러나 조사 결과 `ublox_dgnss` 패키지에 포함된 기본 NTRIP 클라이언트는 RTCM 하향 수신만 지원하고 GGA 상향 전송 기능이 없었다.

이에 본 연구는 GGA 전송을 포함한 NTRIP 클라이언트(`vrs_ntrip_client`)를 구현하였다. 동작 절차는 다음과 같다.

1. NGII 캐스터(RTS2.ngii.go.kr:2101)의 VRS 마운트포인트에 HTTP Basic 인증으로 접속
2. 이동체 위치를 NMEA GGA 문장으로 구성하여 주기적(기본 1 s)으로 캐스터에 전송
   - 초기에는 시험장 근방의 고정 좌표를 사용하고, `/fix`가 수신되면 실측 위치로 전환
3. 수신한 RTCM3 스트림을 프레임 단위로 분리하여 `rtcm_msgs/Message`로 발행
4. `ublox_dgnss_node`가 이를 USB를 통해 수신기에 주입하여 RTK Fixed로 수렴

GGA 문장은 NMEA 표준에 따라 구성하며, 체크섬은 문장 본문의 각 문자에 대한 XOR 연산으로 계산한다.

### 4.2 부팅 시 순환 의존 문제와 해결

초기 구현에서는 `/fix`가 수신되어야 GGA를 보낼 수 있고, GGA를 보내야 보정 신호가 내려와 정밀한 `/fix`를 얻는 순환 의존이 존재하였다. 본 연구는 시험장 근방의 고정 좌표를 파라미터로 두어 초기 GGA를 생성하도록 함으로써 이를 해소하였다. VRS는 수 km 범위에서 유효하므로 개략 좌표로도 초기 보정 신호 수신에 충분하다.

---

## 5. 좌표 변환 및 로컬 좌표계

### 5.1 WGS84에서 평면 좌표로의 변환

GNSS가 제공하는 위경도(WGS84, EPSG:4326)는 구면 좌표이므로 직접 거리·각도 연산에 사용하기 어렵다. 본 연구는 대상 지역에 해당하는 UTM Zone 52N(EPSG:32652)으로 투영하여 평면 좌표를 얻고, 여기에서 사전에 정의한 원점을 감산하여 지역 좌표계를 구성하였다.

$$
(x, y) = \mathcal{T}_{\text{UTM52N}}(\lambda, \phi) - (x_0, y_0)
$$

여기서 $\mathcal{T}$는 좌표 변환, $(\lambda, \phi)$는 경도·위도, $(x_0, y_0)$는 지역 원점이다. 본 시험장의 원점은 $(477800.0,\ 3964400.0)$이다. 원점은 시험 장소에 따라 갱신된다.

### 5.2 원점 일원화의 중요성

지역 원점은 경로 기록 노드와 측위 노드가 **완전히 동일한 값**을 사용해야 한다. 두 값이 다르면 오차가 아니라 전역 경로 전체가 평행이동하여 차량이 전혀 다른 위치를 추종하게 된다.

본 연구 초기에는 이 값이 다섯 곳에 개별적으로 하드코딩되어 있었고, 시험 장소 변경 시 일부만 갱신되어 지역 좌표가 $(78019,\ -127771)$로 산출되는 오류가 발생하였다. 이는 약 150 km에 해당하는 어긋남이다. 이후 원점을 단일 설정 파일(`site_origin.yaml`)로 통합하고 모든 노드가 공통 로더를 사용하도록 변경하여 재발을 방지하였다.

> **시사점**: 다중 노드 시스템에서 좌표계 기준값과 같은 전역 상수는 단일 정본(single source of truth)으로 관리되어야 하며, 이는 코드 품질 문제가 아니라 안전 문제이다.

---

## 6. 방위각 초기화

### 6.1 문제 정의

측위를 위해서는 위치뿐 아니라 방위각이 필요하다. 사용한 IMU의 방위각은 자력계 기반이어서 지도 좌표계(UTM 북방향 기준)와 정렬되어 있지 않으며, 그 오프셋은 세션마다 달라진다.

### 6.2 GNSS course 기반 오프셋 추정

차량이 직진할 때 GNSS 위치 변화의 방향은 차량의 실제 방위각과 일치한다. 이를 이용해 다음과 같이 오프셋을 산출한다.

$$
\text{course} = \operatorname{atan2}(\Delta n,\ \Delta e)
$$
$$
\psi_{\text{offset}} = \operatorname{normalize}(\text{course} - \psi_{\text{IMU}})
$$

여기서 $\Delta e, \Delta n$은 시작점 대비 동·북 방향 변위, $\psi_{\text{IMU}}$는 도달 시점의 IMU 방위각이다. 산출된 오프셋은 latched 토픽으로 발행되어, 초기화 노드가 종료된 이후에도 측위 노드가 이를 계속 참조한다.

기본 초기화 거리는 10 m로 설정하였다. 거리가 짧으면 GNSS 잡음 대비 변위 비율이 작아 방위각 추정 오차가 커지고, 지나치게 길면 시험 준비 시간이 증가한다.

### 6.3 직진 유지의 폐루프 제어

초기화의 전제는 "차량이 곧게 주행한다"는 것이다. 초기 시험에서는 사람이 차량을 밀거나 원격 조종으로 직진시켰으나, 실제 궤적의 방향 편차가 43°, 157°로 측정되어 직진성 검증에서 연속 거부되었다.

이에 차량이 자체적으로 직진을 유지하도록 하였다. 여기서 핵심은 다음과 같다.

> 방위각의 **절대값**은 아직 알 수 없으나(그것을 구하는 것이 초기화의 목적이다), 출발 시점 대비 **변화량**은 IMU로 정확히 알 수 있다. 따라서 "출발 시점의 방위각을 유지"하는 폐루프 제어는 절대 방위각을 몰라도 성립한다.

조향각 0°를 인가하는 개루프 방식은 조향 중립값이 조금만 어긋나도 궤적이 휘어 검증에 실패한다. 실제로 조향 중립 편차로 인해 우측으로 지속 편향되는 현상이 관측되었다. 폐루프 방식은 이러한 기구적 편차를 자동으로 보상한다.

또한 초기화 시 지령은 원격조종 채널로 전달하여, 명령 다중화기(command multiplexer)의 우선순위 체계(비상정지 > 원격조종 > 자율주행)를 그대로 활용하였다. 이로써 (a) 자율주행 로직의 정지 명령을 덮어쓸 수 있고, (b) 원격조종 채널의 타임아웃이 데드맨 스위치로 동작하며, (c) 비상정지는 최상위로 유지된다.

---

## 7. 측위 노드 설계

### 7.1 위치·방위각 융합

측위 노드(`direct_localization_node`)는 다음을 수행한다.

1. `/fix`(NavSatFix)를 UTM 52N으로 변환하고 지역 원점을 감산하여 $(x, y)$ 산출
2. IMU 방위각에 오프셋을 적용하여 지도 좌표계 방위각 $\psi$ 산출
3. 연속된 위치로부터 차체 전방 속도 추정
4. 이들을 `nav_msgs/Odometry`로 발행하고 `map → base_link` 좌표 변환을 방송

차체 전방 속도는 위치 미분값을 방위각 방향으로 투영하여 구한다.

$$
v_x = \dot{x}\cos\psi + \dot{y}\sin\psi
$$

RTK Fixed 상태에서 위치 정확도가 cm급이므로, 확장 칼만 필터(EKF) 기반 융합을 거치지 않고 직접 사용하는 구조를 채택하였다. 이는 구조를 단순화하여 디버깅 가능성을 높이는 실용적 선택이다.

### 7.2 센서 두절에 대한 안전 설계

초기 구현에서는 GNSS 신호가 끊겨도 마지막 위치를 계속 발행하였다. 이 경우 차량은 실제로 이동하지만 측위 출력은 고정되어, 하위 경로 계획 모듈이 이상을 감지하지 못한 채 경로를 계속 생성하는 위험한 상태가 된다.

본 연구는 이를 다음과 같이 변경하였다.

- `/fix`가 설정 시간(기본 1.0 s) 이상 수신되지 않으면 **측위 발행을 중단**한다.
- IMU에 대해서도 동일한 감시를 적용한다. 실제로 주행 중 IMU 프로세스는 살아 있으나 데이터 송신이 멈추는 사례가 관측되었다.
- 발행이 중단되면 하위 경로 계획·추종 모듈이 자체 타임아웃으로 안전 정지한다.

> **시사점**: 센서 두절 시 "마지막 값 유지"는 하위 모듈이 이상을 인지할 수 없게 만들어 오히려 위험하다. 데이터의 부재를 명시적으로 전파하는 편이 안전하다.

---

## 8. 경로 생성 및 기구학적 후처리

### 8.1 경로 기록과 재추출

주행 경로는 사람이 차량을 운전하며 GNSS 위치를 기록하는 방식(teaching)으로 생성하였다. 기록된 원 경로는 345점이며, 총연장은 190.4 m이다. 추종 알고리즘이 균일한 간격을 전제하므로, 선분 상 선형 보간으로 0.5 m 등간격 재추출을 수행하였다.

### 8.2 재추출 간격과 곡률 잡음

재추출 간격을 좁히면 경로 표현이 정밀해질 것으로 예상할 수 있으나, 실제로는 그렇지 않았다. 간격별 최소 곡률반경을 산출한 결과는 다음과 같다.

| 재추출 간격 | 점 수 | 최소 곡률반경 | 차량 한계(2.42 m) 미달 구간 |
|---|---|---|---|
| 0.3 m | 636 | 0.80 m | 25 |
| 0.4 m | 477 | 1.33 m | 9 |
| **0.5 m** | **382** | **1.73 m** | **2** |
| 0.8 m | 239 | 1.97 m | 2 |
| 1.0 m | 191 | 2.23 m | 1 |

원 기록이 345점인데 0.3 m 간격은 636점을 생성한다. 이는 원 데이터보다 조밀하여 존재하지 않는 정보를 보간으로 만들어내는 것이며, 그 결과 GNSS 기록 잡음(수 cm)이 짧은 구간에서 큰 곡률로 나타난다. 0.6 m 구간에서 수 cm의 횡방향 잡음은 곡률반경 1 m 미만으로 계산된다.

> **시사점**: 경로 재추출 간격은 원 데이터의 공간 해상도를 초과하지 않아야 하며, 잡음이 포함된 기록 데이터에서는 간격 축소가 곡률 잡음을 증폭시킨다.

### 8.3 곡률 제약을 반영한 경로 평활화

차량의 최대 조향각 18°와 축거 0.785 m로부터 최소 회전반경은 다음과 같다.

$$
R_{\min} = \frac{L}{\tan \delta_{\max}} = \frac{0.785}{\tan 18°} \approx 2.42\ \text{m}
$$

기록 경로에는 $R = 1.73$ m, $1.91$ m 구간이 존재하였으며, 이는 측위와 제어가 정확하더라도 차량이 기구학적으로 추종할 수 없는 구간이다.

이에 이탈량을 제한한 반복 평활화를 적용하였다. 각 점 $p_i$는 원 기록 $p_i^0$로 되돌리는 힘과 이웃 점들의 중점으로 당기는 힘의 균형으로 갱신된다.

$$
p_i \leftarrow p_i + \alpha (p_i^0 - p_i) + \beta (p_{i-1} + p_{i+1} - 2 p_i)
$$

$$
\text{if } \lVert p_i - p_i^0 \rVert > d_{\max}, \quad p_i \leftarrow p_i^0 + d_{\max} \frac{p_i - p_i^0}{\lVert p_i - p_i^0 \rVert}
$$

여기서 이탈량 제한 $d_{\max}$가 핵심이다. 제한 없이 평활화하면 곡선 구간이 안쪽으로 잘려(corner cutting) 오히려 주행 가능 영역을 벗어난다.

$\alpha = 0.10$, $\beta = 0.50$, $d_{\max} = 0.50$ m를 적용한 결과는 다음과 같다.

| 항목 | 평활화 전 | 평활화 후 |
|---|---|---|
| 점 수 | 383 | 378 |
| 최소 곡률반경 | 1.73 m | **2.68 m** |
| 한계 미달 구간 | 2 | **0** |
| 필요 최대 조향각 | 24.4° | **16.3°** |
| 원 경로 대비 이동량 | — | 최대 0.46 m, 평균 0.09 m |

평균 이동량이 0.09 m로 작아 대부분 구간은 원 경로를 유지하며, 기구학적으로 불가능한 소수 구간만 완화되었음을 확인할 수 있다.

---

## 9. 실험 및 결과

### 9.1 시험 환경

총연장 190.4 m의 실외 주행 구간에서 시험을 수행하였다. RTK 보정은 NGII VRS를 통해 수신하였다.

### 9.2 측위 정확도 검증

RTK 측위의 절대 정확도는 별도의 기준 장비 없이 직접 검증하기 어려우므로, 본 연구는 휠 엔코더 적산 거리와의 상호 비교로 정합성을 확인하였다. 직선 구간 주행에서 RTK 기준 이동 거리는 8.844 m, 엔코더 적산 거리는 8.664 m로 측정되었으며, 두 값의 비는 1.0208이었다. 동일 구간의 횡방향 편차는 0.24 m로 나타났다.

이 결과는 두 측정계가 2% 이내로 일치함을 보이며, 엔코더 환산 계수 보정에도 활용되었다.

### 9.3 자율 주행 시험

구현된 측위 시스템을 경로 추종 제어와 연동하여 자율 주행 시험을 수행하였다. 시험 구간 190 m를 자율 완주하였으며, 주행 속도는 약 3.6 km/h였다.

### 9.4 경로 평활화의 효과

평활화 적용 전후를 HIL(Hardware-in-the-Loop) 환경에서 비교하였다. HIL 환경은 실제 하위 제어기와 차량 구동계를 연결한 상태에서 차량 운동을 모사하여, 실외 시험 없이 제어 체인 전체를 검증하는 방식이다.

평활화 적용 후 주행에서 사용된 조향각은 −11°~+9° 범위였으며, 최대 조향각 18°에 대해 약 7°의 여유를 확보하였다. 조향 포화(saturation)는 발생하지 않았다. 평활화 이전에는 곡률 한계 초과 구간에서 조향이 포화된 상태로 경로를 이탈할 수밖에 없는 조건이었다.

---

## 10. 결론 및 향후 과제

### 10.1 결론

본 연구는 저비용 자율주행 플랫폼에 RTK-GNSS 기반 실시간 측위 시스템을 구현하고 실차 주행으로 검증하였다. 주요 결과는 다음과 같다.

1. VRS 기반 RTK 보정 수신을 위해 GGA 상향 전송을 포함한 NTRIP 클라이언트를 구현하여, 기존 ROS 2 패키지의 제약을 해소하였다.
2. GNSS 이동 방향을 이용한 방위각 초기화 기법을 구현하고, 방위각 변화량 기반 폐루프 직진 제어를 결합하여 초기화 신뢰성을 확보하였다.
3. 센서 두절 시 측위 발행을 중단하는 안전 지향 설계를 적용하여, 하위 제어기가 이상 상태를 명시적으로 인지하고 정지할 수 있도록 하였다.
4. 차량 기구학 제약을 반영한 경로 평활화를 통해 추종 불가능 구간을 제거하고 조향 여유를 확보하였다.

### 10.2 향후 과제

- **측위 정확도의 정량 평가**: 본 연구의 정확도 검증은 엔코더와의 상호 비교에 의존하였다. 기준점 측량 성과 또는 고정밀 기준 장비를 이용한 절대 정확도 평가가 필요하다.
- **RTK 상태별 성능 분석**: Fixed/Float 상태 전환이 경로 추종 오차에 미치는 영향을 정량 분석할 필요가 있다.
- **횡방향 편차의 원인 규명**: 직선 구간에서도 차선 이탈이 관측되었으며, 이는 방위각 오차 또는 RTK 상태 저하에 기인할 가능성이 있다. 주행 로그 기반의 횡편차 분석이 요구된다.
- **GNSS 음영 대응**: 현재는 신호 두절 시 안전 정지하는 방식이나, 추측항법(dead reckoning) 결합으로 단기 음영 구간 주행 유지가 가능하다.

---

## 참고문헌 (예시 — 실제 인용 문헌으로 교체 필요)

[1] RTCM Special Committee No. 104, "RTCM Standard 10403.3 for Differential GNSS Services," RTCM, 2016.

[2] RTCM Special Committee No. 104, "Networked Transport of RTCM via Internet Protocol (Ntrip) Version 2.0," RTCM, 2011.

[3] u-blox AG, "ZED-F9P Integration Manual," u-blox, 2022.

[4] 국토지리정보원, "위성기준점 실시간 데이터 서비스 이용 안내," 국토지리정보원.

[5] S. Macenski, T. Foote, B. Gerkey, C. Lalancette, and W. Woodall, "Robot Operating System 2: Design, architecture, and uses in the wild," *Science Robotics*, vol. 7, no. 66, 2022.

[6] R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm," Robotics Institute, Carnegie Mellon University, Tech. Rep. CMU-RI-TR-92-01, 1992.

---

## 작성 메모 (제출 전 확인 사항)

- **분량**: 위 구성은 학회 논문지 기준 약 7~8쪽에 해당한다. 학술대회 단문(4~6쪽)으로 줄일 경우 2장(관련 연구)과 8장(경로 후처리)을 축약하고, 6장·7장을 중심에 둘 것을 권한다.
- **그림 추가 권장**:
  - 그림 1. 시스템 구성도 (3.2절 데이터 흐름을 도식화)
  - 그림 2. 방위각 초기화 개념도 (GNSS course와 IMU yaw의 관계)
  - 그림 3. 경로 평활화 전후 비교 (급커브 구간 확대)
  - 그림 4. 곡률반경 분포 히스토그램 (평활화 전후)
- **보완 필요 데이터**:
  - RTK Fixed 유지율(주행 중 Fixed/Float/Single 비율) — 로그 수집 필요
  - 경로 추종 횡편차(cross-track error) 통계 — 로그 수집 필요
  - 방위각 초기화 반복 정밀도(동일 지점 반복 시 오프셋 편차)
- **표기**: 본문의 수치는 모두 실측값이며, 추정·가정값은 사용하지 않았다. 향후 실험으로 값이 갱신되면 해당 절을 함께 수정할 것.
