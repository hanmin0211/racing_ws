// =============================================================================
// henes_firmware.ino  —  HENES T870 브룬 자율주행 하위제어 펌웨어 (재구축)
// -----------------------------------------------------------------------------
// 설계 철학: "PID가 무엇을 하든, 안전가드가 스톨/과부하를 잘라 모터를 못 태운다."
//   [PID 계산] → PWM → [★안전가드] → analogWrite(모터)
//                          ↑ 스톨/엔드스톱/워치독 감지 시 강제 0
//
// 구성: 구동(전/후 DC모터, 엔코더 속도PID+FF+안티와인드업+소프트스타트),
//       조향(DC모터+포텐셔미터 A15, 위치PID, 캘리브 매핑), 소나 3개,
//       시리얼(serial_bridge 포맷) + 워치독.
//
// ⚠ 안전 수칙:
//   - 반드시 바퀴 들고(공중) 벤치 테스트부터. MAX_*_PWM 낮게 시작해 조금씩 올릴 것.
//   - 스톨 일부러 유발(바퀴/조향 막기)해서 가드가 실제로 컷하는지 확인 후 실주행.
// =============================================================================

#include <SPI.h>
#include <NewPing.h>

// ============================ 1. 핀 정의 (검증된 배선) =========================
#define MOTOR1_PWM 5    // 전륜 구동
#define MOTOR1_ENA 6
#define MOTOR1_ENB 7
#define MOTOR2_PWM 2    // 후륜 구동
#define MOTOR2_ENA 3
#define MOTOR2_ENB 4
#define MOTOR3_PWM 8    // 조향
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10

#define Steering_Sensor A15   // 조향 포텐셔미터
#define ENC1_ADD 22           // 엔코더 SPI CS
#define ENC2_ADD 23
#define SONAR_NUM 3
#define MAX_DISTANCE 200

// ============================ 2. 조향 캘리브 (실측 2026-08-13) =================
// 좌측끝=0, 우측끝=949, 최대타각 30° (좌+/우−).
// 직진(중앙): 459 → **424로 갱신** (2026-08-13 재실측). 바퀴를 손으로 물리적
// 직진에 맞춘 뒤 읽은 ADC가 424(±1)였다. 459는 실제 직진보다 우측으로 치우쳐
// 있어 "0° 명령인데 좌로 살짝 틀어져 보이는" 증상의 원인이었다.
// ※ 최대타각 20° 기준: 좌측 424counts/20° = 21.2/도, 우측 525counts/20° = 26.3/도로
//   비대칭이지만, steerAngleToADC가 좌/우를 각각 스케일하므로 각도는 정확하다.
//   (데드밴드 16counts = 좌 0.75° / 우 0.61° — 분해능은 충분)
#define STEER_CENTER    424
#define STEER_LEFT_MAX    0
#define STEER_RIGHT_MAX 949
// 최대 타각: 30°는 가정값이었고, 2026-08-13 실측 결과 **20°**. (가정대로 두면
// pure pursuit가 20° 명령해도 실제로는 13°만 꺾여 코너마다 밖으로 밀린다.)
#define STEER_MAX_ANGLE  20.0
// 엔드스톱 안전마진 (물리 끝값보다 안쪽에서 컷)
#define STEER_AD_MIN  (STEER_LEFT_MAX  + 10)   // 10
#define STEER_AD_MAX  (STEER_RIGHT_MAX - 10)   // 939

// 조향각(도, 좌+/우−) → 목표 ADC
// ★ 부호 정정(2026-08-15 실차 확인): 예전 매핑은 '+각도 → ADC 감소'였는데, 육안
//   확인 결과 그게 **실제로는 우회전**이었다. 즉 ADC가 낮은 쪽이 우, 높은 쪽이 좌.
//   (관찰 조건: 차량 뒤집힘 + 정면에서 관찰 → 반전이 두 번이라 상쇄되어 '눈에
//    보이는 오른쪽 = 차의 오른쪽'. 그 상태에서 +15° 명령에 우측으로 꺾였음.)
//   → +각도(좌)는 ADC 증가(949 방향), −각도(우)는 ADC 감소(0 방향)로 정정.
//
// ★ 단일 선형 스케일 채택(2026-08-15).
//   예전엔 '좌우 양 끝이 각각 정확히 20°'라고 가정해 방향별로 따로 스케일했는데
//   (좌 26.25 / 우 21.2 counts/도), 그 가정은 검증된 적이 없다. 최대타각 20°는
//   **한쪽만 실측**한 값이다. 포텐셔미터가 조향축에 직결이면 카운트/도는 전 구간
//   일정한 것이 물리적으로 옳으므로, 스케일을 하나로 통일한다.
//   보수적으로 작은 값(424counts/20° = 21.2)을 쓴다:
//     · 실제가 21.2면 → 정확
//     · 실제가 26.25면 → 명령보다 덜 꺾임(언더스티어) = 안전한 방향의 오차
//     (큰 값을 쓰면 반대로 오버스티어가 되어 위험하다)
//   실사용 18°면 ADC 42~806으로 양쪽 포화(0/949)에서 충분히 떨어진다.
//   ※ 반대쪽 최대타각을 실측하면 이 값만 고치면 된다.
#define STEER_COUNTS_PER_DEG  21.2

int steerAngleToADC(float ang) {
  if (ang >  STEER_MAX_ANGLE) ang =  STEER_MAX_ANGLE;
  if (ang < -STEER_MAX_ANGLE) ang = -STEER_MAX_ANGLE;
  float adc = STEER_CENTER + ang * STEER_COUNTS_PER_DEG;   // 좌+ → ADC↑
  return constrain((int)adc, STEER_LEFT_MAX, STEER_RIGHT_MAX);
}

// ADC → 실제 조향각[도]. 명령각이 아니라 '지금 바퀴가 실제로 몇 도인지'.
// 자율주행 중 조향 추종 오차를 감시하려면 이 값이 필요하다.
float steerADCToAngle(int adc) {
  return (adc - STEER_CENTER) / STEER_COUNTS_PER_DEG;
}

// ============================ 3. 안전 파라미터 ================================
// ★ 벤치 테스트는 이 값들을 낮게 시작해서 단계적으로 올린다.
// 구동 PWM 상한. 80은 근거 없는 보수값이었고, FF 실측 결과 1.5m/s에 PWM 71이
// 필요해 PID 여유를 더한 111 로 상향(ff_sweep 권장치). 스톨가드가 보호한다.
// ★ 2026-08-17 저녁: 190 으로 올렸더니 랩 종반에 아두이노가 꺼졌다.
// 모터 부하 시 공급전압이 4018mV → **3483mV** 로 535mV 떨어지는데,
// ATmega2560 브라운아웃 문턱(약 4.3V)보다 한참 아래라 리셋된다.
// 차량 배터리를 교체해도 전압이 그대로였으므로(4018→3987mV) 원인은
// 배터리가 아니라 **아두이노 전원 경로**다. 이건 실내에서 멀티미터로
// 5V 핀을 직접 재서 확정해야 한다.
//
// 그때까지는 완주 실적이 있는 111 로 되돌린다(전류 ↓ → 전압 강하 ↓).
// 전원 문제가 해결되면 190 으로 올릴 것 — FF 는 이미 검증됐다
// (0.5 명령에 PWM 158, 상한 미접촉).
// ★ 2026-08-18: 리셋의 진짜 원인은 USB 버스 충돌(ublox↔아두이노)이었고
// 버스 물리분리로 해결됐다. 전원/FF 문제가 아니었으므로 속도 상향이 가능하다.
// 단계적 상향: 111(완주 실적) → 150 → 190. 각 단계에서 VMIN·리셋을 확인한다.
// FF/PID 는 완주한 값 그대로 두고 이 값만 바꾼다(한 번에 한 변수).
#define MAX_DRIVE_PWM   210
// 조향 PWM 상한. 90은 벤치 초기 보수값이었는데, 실측 결과 '완전 정지 상태에서의
// breakaway(정지마찰 뜯기)'에 부족했다(움직이는 중엔 90으로 충분히 잘 감).
// 130으로 상향 — 스톨가드(PWM>30이 250ms간 무이동 시 컷 + 1.5s 쿨다운)가 보호한다.
#define MAX_STEER_PWM   130
// 부팅 후 이 시간 동안 조향 PWM 상한을 0→MAX 로 램프 (돌입 전류 방지).
// 아래 steering_pid_control 말미의 소프트스타트 주석 참고.
#define STEER_SOFT_MS   2500UL
#define STEER_SOFT_MIN  25       // 너무 낮으면 아예 안 움직이므로 하한

// 조향 위치제어 파라미터 (자율주행용).
// pure_pursuit가 20Hz로 내는 연속 목표각(ROS단에서 90°/s 슬루제한)을 매끄럽게 추종.
// ※ AC 디더 방식은 폐기: 50Hz 반전은 기어드 모터가 못 따라가 net 이동 없이 버즈(삐-)만
//   나고, 스티션 극복도 안 됨. 대신 '움직이면 최소 STEER_MIN_MOVE의 steady DC, 아니면 0'
//   방식(옛 검증본)으로 확실히 움직이게 하고, 히스테리시스로 중앙 헌팅/버즈를 막는다.
#define STEER_DEADBAND    16   // ADC(~1.0°). 이내로 도달하면 정지(무부하·무소음)
#define STEER_RESUME      24   // 정지상태에서 오차가 이 이상 벌어져야 재출발(히스테리시스)
#define STEER_MIN_MOVE    34   // 스티션 극복 최소 PWM. 이 이하 DC로는 실제로 안 움직임
                               //   → 움직일 땐 반드시 이 이상 실어 '확실히 이동 or 정지'
// ★ 정지마찰 탈출 부스트.
// 실측(2026-08-13): 큰 각도(300카운트↑)는 PWM이 130까지 올라가 잘 뜯기는데
// **작은 보정(60카운트≈2.8°)은 PWM 60으로 정지마찰을 못 이겨 아예 안 움직였다.**
// pure pursuit가 내는 명령 대부분이 이런 작은 보정이라, 그대로 두면 직진 유지가 안 된다.
// 대책: '명령이 있는데 실제로 안 움직이는' 동안만 PWM을 점진적으로 키우고, 움직이기
// 시작하면 즉시 0으로 리셋해 정상 PID로 복귀. 끝내 안 움직이면 스톨가드가 250ms에
// 컷하므로 모터 보호는 그대로 유지된다.
#define STEER_BOOST_STEP   6   // 사이클(10ms)당 증가량 → 약 160ms에 +96
#define STEER_MOVING_DELTA 2   // ADC가 이만큼 변하면 '움직이는 중'으로 판정

// 구동 스톨: PWM 높은데 안 움직임이 지속되면 컷
#define DRIVE_STALL_PWM    55     // 이 이상 PWM인데
#define DRIVE_STALL_SPEED  0.05   // 속도가 이 이하로 (m/s)
#define DRIVE_STALL_MS     300    // 이 시간 지속되면 스톨

// 조향 스톨: PWM 높은데 ADC가 목표로 안 감
#define STEER_STALL_PWM    30    // 이 이상 PWM인데 (실측: PWM 38~49에서도 스톨 발생.
                                 //   옛 50/60은 그 구간을 못 잡아 모터가 계속 전류만 먹었음)
#define STEER_STALL_DELTA  3      // ADC 변화가 이 이하이고 (카운트)
#define STEER_STALL_ERR    15     // 아직 오차가 이 이상이면
#define STEER_STALL_MS     250    // 이 시간 지속되면 스톨
// 스톨 컷 유지시간. 이 시간이 지나면 래치를 풀고 '재시도'한다.
// ★ 옛 로직은 '오차<15'가 되어야만 래치가 풀렸는데, 컷 상태(PWM 0)에선 움직일 수
//   없어 오차가 줄지 않아 → 조향이 영구 사망했다(주행 중이면 사고). 쿨다운 재시도로
//   일시적 걸림은 스스로 회복하고, 진짜 고장이면 컷/재시도를 반복하며 모터를 보호한다.
#define STEER_STALL_COOLDOWN_MS 1500

#define SERIAL_TIMEOUT_MS  500    // 명령 없으면 워치독 정지
#define CONTROL_DT_MS      10      // 제어주기 (100Hz)
#define TEL_INTERVAL_MS    50      // 텔레메트리 주기 (20Hz, serial_bridge STATUS_10ms)

// 소프트 스타트/브레이크 (가속도 제한, m/s^2)
const float ACCEL_LIMIT = 0.5;
const float BRAKE_LIMIT = 0.8;
const float WATCHDOG_BRAKE_LIMIT = 2.5;

// ============================ 4. 전역 상태 ===================================
float target_velocity = 0.0;     // ROS 목표 속도
float commanded_velocity = 0.0;  // 소프트스타트 반영된 PID 목표
float current_velocity = 0.0;
float target_steer_angle = 0.0;  // ROS 목표 조향각(도)

// ---- 개루프(open-loop) 모드 : FF/PID 식별 전용 ----
// `PWM:x` 명령을 받으면 속도 PID를 우회하고 지정 PWM을 그대로 인가한다.
// FF(정지마찰·속도비례 항)를 재식별하려면 '이 PWM에서 실제로 몇 m/s가 나오는가'를
// 측정해야 하는데, PID가 개입하면 그 관계가 가려지기 때문이다.
// ★ 안전가드(스톨 감지)·워치독은 개루프에서도 그대로 살아있다.
// `VEL:` 명령이 오면 즉시 폐루프로 복귀한다.
bool openloop_active = false;
int openloop_target_pwm = 0;
int openloop_pwm = 0;            // 레이트 제한이 적용된 실제 인가값
#define OPENLOOP_RATE     3      // 사이클(10ms)당 최대 변화 → 급가속 방지
// 식별용 상한 (MAX_DRIVE_PWM 과 별개).
// ★ 2026-08-17: 140 이면 지면에서 약 0.5 m/s 까지만 측정된다. 그 위를 쓰려면
//   외삽해야 하는데, 바로 그 외삽이 FF 를 10배 틀리게 만든 원인이었다.
//   **실제로 쓸 속도 범위 전체를 스윕이 덮어야 한다.**
//   모터는 24V 240W(16000rpm) 이고 현재 듀티는 43% 에 불과하다. 230 은
//   원차 설계 속도(약 1.4 m/s) 이내이며 255 대비 여유도 남긴다.
#define MAX_OPENLOOP_PWM 230

signed long encoder1count = 0, encoder2count = 0, prev_encoder1 = 0;
unsigned long prev_time = 0, last_rx_time = 0, last_tel_time = 0;
// ★ 부팅 직후에는 '워치독 작동' 상태로 시작한다 (true).
//
// false 로 두면 setup() 의 last_rx_time=millis() 때문에 **명령을 한 번도
// 받지 않았는데도 조향 PID 가 즉시 돌기 시작한다.** 목표각은 0(중앙)이라
// 바퀴가 중앙에서 벗어나 있으면 부팅하자마자 조향모터에 최대 PWM 이 걸린다.
//
// 2026-08-17 현장: 조향이 중앙에서 15° 벗어난 상태였고, 부팅 0.32초 뒤마다
// 정확히 리셋이 반복됐다(전류 급증 → 5V 강하 → 브라운아웃 → 재부팅 → 반복).
// serial_bridge 가 재연결할 때마다 DTR 로 또 리셋시켜 루프가 유지됐다.
//
// true 로 시작하면 첫 명령(VEL:/STEER:/PWM:)이 올 때까지 모터가 완전히
// 꺼져 있어 이 루프가 성립하지 않는다. 명령이 오면 handle_line() 이
// watchdog_tripped=false 로 풀어준다.
bool watchdog_tripped = true;

// 스톨 상태
unsigned long drive_stall_ms = 0, steer_stall_ms = 0, steer_cut_ms = 0;
bool drive_stalled = false, steer_stalled = false;
int prev_sensorValue = STEER_CENTER;

// 바퀴 파라미터
// ★ RTK 실측 보정(2026-08-15). encoder_calib 결과 보정계수 1.0208
//   (RTK 8.844m vs 엔코더 환산 8.664m, 3076counts, 직진편차 0.24m).
//   0.13 은 공칭 추정값이었고 실제 유효 구름반경이 조금 더 컸다.
//   ※ 재검증: encoder_calib 다시 돌려 보정계수가 1.00±0.02 면 통과.
const float wheel_radius = 0.1327;
const int counts_per_revolution = -290;
const float wheel_circumference = 2 * 3.14159 * wheel_radius;

// 구동 속도 PID
// ★ FF가 정확해졌으므로 PID는 작은 오차만 보정하면 된다. 이전 30/24 는 틀린 FF를
//   억지로 메우던 값이고, 정지마찰에 걸린 6초 동안 적분이 쌓였다가 터지는
//   **급발진의 원인**이었다(2026-08-16 현장 관측). 대폭 낮춘다.
// ★ PID 게인은 플랜트 기울기에 비례해야 한다. 기울기가 200 PWM/(m/s) 인데
// kp=8 이면 0.1m/s 오차에 0.8 PWM — P 제어가 사실상 없는 것과 같았다.
// FF 가 제대로 들어간 지금은 PID 가 작은 오차만 다듬으면 된다.
//   kp=50  → 0.1m/s 오차에 5 PWM
//   ki=30  → 0.1m/s 오차가 1초 지속되면 3 PWM 추가 (수 초 내 수렴)
// 완주 실적이 있는 값(2026-08-17 아침). FF 를 되돌렸으므로 게인도 함께 복귀.
float velocity_kp = 8.0, velocity_ki = 4.0, velocity_kd = 0.2;
float velocity_error = 0.0, velocity_error_old = 0.0, velocity_error_sum = 0.0;
int velocity_pwm_output = 0;
const float VELOCITY_DT = CONTROL_DT_MS / 1000.0;
// ★ FF 실측 식별(2026-08-16, ff_sweep 103샘플, 속도 0.06~1.15m/s, PWM 22~62,
//   피팅 잔차 RMS 4.3 PWM). 이전 35/60 은 근거 불명이었고 속도항이 3배 과다했다.
//   ※ 스윕을 한 방향에서만 했으므로 경사 편향 가능성이 있다. 반대 방향으로
//     한 번 더 재서 평균 내면 더 정확해진다.
// ★ 2026-08-17 재설정. 이전 값(40.5 / 20.7)은 **바퀴가 접지되지 않은 상태**의
// ff_sweep 에서 나온 것이라 속도항이 10배 작았다. 그 결과 FF 가 내야 할 몫을
// 적분항이 전부 메우게 되어 (a) 적분 클램프에 걸려 속도가 0.33m/s 에서 막히고
// (b) 적분이 포화돼 코너 감속이 수십 초 지연됐다.
//
// 지면 실측 2점으로 역산:
//     MAX_DRIVE_PWM 80  → 0.198 m/s
//     MAX_DRIVE_PWM 111 → 0.33  m/s
//   ⇒ 기울기 ≈ 200~240 PWM/(m/s), 절편 ≈ 30~42
// 보수적으로 200 / 35 를 쓴다. ±20% 틀려도 PID 가 흡수하는 범위다.
// 마른 노면에서 ff_sweep 을 다시 돌리면 이 값을 확정할 수 있다.
// ★ 2026-08-18: 어제 아침 190m 완주에 성공한 값으로 되돌린다.
// 200 은 지면 실측 2점으로 역산한 '더 정확한' 값이지만, 출발 시 돌입 PWM 을
// 43 → 61 로 40% 키워 전원 여유가 없는 이 차에서 보드 리셋을 유발했다.
// 정확도보다 **도는 것**이 우선이다. 전원 계통을 보강한 뒤 다시 올릴 것.
const float STATIC_FF = 40.5, VELOCITY_FF_GAIN = 20.7;

// 조향 위치 PID
float steering_kp = 1.0, steering_ki = 0.0, steering_kd = 0.2;
float steering_error = 0.0, steering_error_old = 0.0, steering_error_sum = 0.0;
int steering_pwm_output = 0;
int sensorValue = STEER_CENTER;

NewPing sonar[SONAR_NUM] = {
  NewPing(11, 11, MAX_DISTANCE),
  NewPing(12, 12, MAX_DISTANCE),
  NewPing(13, 13, MAX_DISTANCE)
};

// ============================ 5. 저수준 모터 제어 =============================
void front_motor_control(int pwm) {
  int lim = openloop_active ? MAX_OPENLOOP_PWM : MAX_DRIVE_PWM;
  pwm = constrain(pwm, -lim, lim);
  if (pwm > 0)      { digitalWrite(MOTOR1_ENA, HIGH); digitalWrite(MOTOR1_ENB, LOW);  analogWrite(MOTOR1_PWM, pwm); }
  else if (pwm < 0) { digitalWrite(MOTOR1_ENA, LOW);  digitalWrite(MOTOR1_ENB, HIGH); analogWrite(MOTOR1_PWM, -pwm); }
  else              { digitalWrite(MOTOR1_ENA, LOW);  digitalWrite(MOTOR1_ENB, LOW);  analogWrite(MOTOR1_PWM, 0); }
}
void rear_motor_control(int pwm) {
  int lim = openloop_active ? MAX_OPENLOOP_PWM : MAX_DRIVE_PWM;
  pwm = constrain(pwm, -lim, lim);
  if (pwm > 0)      { digitalWrite(MOTOR2_ENA, HIGH); digitalWrite(MOTOR2_ENB, LOW);  analogWrite(MOTOR2_PWM, pwm); }
  else if (pwm < 0) { digitalWrite(MOTOR2_ENA, LOW);  digitalWrite(MOTOR2_ENB, HIGH); analogWrite(MOTOR2_PWM, -pwm); }
  else              { digitalWrite(MOTOR2_ENA, LOW);  digitalWrite(MOTOR2_ENB, LOW);  analogWrite(MOTOR2_PWM, 0); }
}
// 조향 모터: ★엔드스톱 하드컷 (센서가 안전범위 밖이면 그 방향으로 더 안 밀음)
void steer_motor_control(int pwm) {
  pwm = constrain(pwm, -MAX_STEER_PWM, MAX_STEER_PWM);
  // pwm>0 = 우측(ADC↑ 방향)으로 민다고 가정. 실차에서 방향 반대면 부호만 뒤집기.
  bool at_right_limit = (sensorValue >= STEER_AD_MAX);
  bool at_left_limit  = (sensorValue <= STEER_AD_MIN);
  if ((pwm > 0 && at_right_limit) || (pwm < 0 && at_left_limit)) pwm = 0;  // 엔드스톱 컷
  if (pwm > 0)      { digitalWrite(MOTOR3_ENA, LOW);  digitalWrite(MOTOR3_ENB, HIGH); analogWrite(MOTOR3_PWM, pwm); }
  else if (pwm < 0) { digitalWrite(MOTOR3_ENA, HIGH); digitalWrite(MOTOR3_ENB, LOW);  analogWrite(MOTOR3_PWM, -pwm); }
  else              { digitalWrite(MOTOR3_ENA, LOW);  digitalWrite(MOTOR3_ENB, LOW);  analogWrite(MOTOR3_PWM, 0); }
}
void all_motors_off() { front_motor_control(0); rear_motor_control(0); steer_motor_control(0); }

// ============================ 5.5 공급전압(VCC) 모니터 =======================
// 내부 1.1V 밴드갭을 ADC로 읽어 역산하면 VCC를 알 수 있다(별도 배선 불필요).
// 모터 스톨 순간 전압이 얼마나 주저앉는지(sag) 보려고 매 제어주기 샘플링하고,
// 텔레메트리 주기 동안의 최솟값(VMIN)을 함께 보고한다.
//   - 정상: 4900~5100mV 부근에서 안정
//   - 4300mV 이하로 떨어지면 brownout 위험(ATmega2560 BOD 트립 영역)
int vcc_mv = 0;
int vcc_min_mv = 9999;

int readVccMv() {
  // MUX[5:0]=011110 → 1.1V 밴드갭 (ATmega2560). REFS0=1 → 기준전압 AVCC.
  ADCSRB &= ~_BV(MUX5);
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  // 기준/채널 전환 후 첫 변환은 버린다(정착). delay 대신 더미 변환 → 루프 영향 최소.
  ADCSRA |= _BV(ADSC); while (bit_is_set(ADCSRA, ADSC));
  ADCSRA |= _BV(ADSC); while (bit_is_set(ADCSRA, ADSC));
  uint8_t low = ADCL, high = ADCH;
  long raw = ((long)high << 8) | low;
  if (raw == 0) return 0;
  return (int)(1125300L / raw);   // 1.1V * 1023 * 1000 / raw
}

// ============================ 6. 엔코더 =====================================
void initEncoders() {
  pinMode(ENC1_ADD, OUTPUT); pinMode(ENC2_ADD, OUTPUT);
  digitalWrite(ENC1_ADD, HIGH); digitalWrite(ENC2_ADD, HIGH);
  SPI.begin();
  digitalWrite(ENC1_ADD, LOW); SPI.transfer(0x88); SPI.transfer(0x03); digitalWrite(ENC1_ADD, HIGH);
  digitalWrite(ENC2_ADD, LOW); SPI.transfer(0x88); SPI.transfer(0x03); digitalWrite(ENC2_ADD, HIGH);
}
long readEncoder(int no) {
  unsigned int c1, c2, c3, c4;
  digitalWrite(ENC1_ADD + no - 1, LOW);
  SPI.transfer(0x60);
  c1 = SPI.transfer(0x00); c2 = SPI.transfer(0x00); c3 = SPI.transfer(0x00); c4 = SPI.transfer(0x00);
  digitalWrite(ENC1_ADD + no - 1, HIGH);
  return ((long)c1 << 24) + ((long)c2 << 16) + ((long)c3 << 8) + (long)c4;
}
void clearEncoderCount(int no) {
  digitalWrite(ENC1_ADD + no - 1, LOW);
  SPI.transfer(0x98); SPI.transfer(0x00); SPI.transfer(0x00); SPI.transfer(0x00); SPI.transfer(0x00);
  digitalWrite(ENC1_ADD + no - 1, HIGH);
  delayMicroseconds(100);
  digitalWrite(ENC1_ADD + no - 1, LOW); SPI.transfer(0xE0); digitalWrite(ENC1_ADD + no - 1, HIGH);
}

// ============================ 7. 속도 계산 (10ms 이동평균) ====================
void calculate_velocity() {
  long d = encoder1count - prev_encoder1;
  float rev = (float)d / counts_per_revolution;
  float raw = (rev * wheel_circumference) / VELOCITY_DT;
  static float buf[5] = {0,0,0,0,0}; static int bi = 0;
  buf[bi] = raw; bi = (bi + 1) % 5;
  float s = 0; for (int i = 0; i < 5; i++) s += buf[i];
  current_velocity = s / 5.0;
  prev_encoder1 = encoder1count;
}

// ============================ 8. 소프트스타트 + 구동 속도 PID =================
void apply_acceleration_limit() {
  float limit = watchdog_tripped ? WATCHDOG_BRAKE_LIMIT
                : ((fabs(target_velocity) > fabs(commanded_velocity)) ? ACCEL_LIMIT : BRAKE_LIMIT);
  float md = limit * VELOCITY_DT;
  if (target_velocity > commanded_velocity) commanded_velocity = min(target_velocity, commanded_velocity + md);
  else if (target_velocity < commanded_velocity) commanded_velocity = max(target_velocity, commanded_velocity - md);
}
void velocity_pid_control() {
  // 개루프 모드: PID를 건너뛰고 지정 PWM을 레이트 제한만 걸어 인가한다.
  if (openloop_active) {
    int d = openloop_target_pwm - openloop_pwm;
    if (d >  OPENLOOP_RATE) d =  OPENLOOP_RATE;
    if (d < -OPENLOOP_RATE) d = -OPENLOOP_RATE;
    openloop_pwm += d;
    velocity_pwm_output = constrain(openloop_pwm,
                                    -MAX_OPENLOOP_PWM, MAX_OPENLOOP_PWM);
    velocity_error_sum = 0.0;
    velocity_error_old = 0.0;
    commanded_velocity = 0.0;
    return;
  }
  velocity_error = commanded_velocity - current_velocity;
  float ed = (velocity_error - velocity_error_old) / VELOCITY_DT;
  float ff = 0.0;
  if (commanded_velocity > 0.02)  ff =  STATIC_FF + VELOCITY_FF_GAIN * commanded_velocity;
  else if (commanded_velocity < -0.02) ff = -STATIC_FF + VELOCITY_FF_GAIN * commanded_velocity;
  // 조건부 안티와인드업
  float ts = velocity_error_sum + velocity_error * VELOCITY_DT;
  // ★ 적분 클램프. ki 가 4→30 으로 커졌으므로 같은 ±25 를 쓰면 적분 권한이
  // 750 PWM 이 되어 조금만 어긋나도 포화한다. FF 가 맞는 지금은 적분이
  // ±90 PWM 정도만 다듬으면 충분하다 (경사·배터리 새그 보정분).
  ts = constrain(ts, -25.0, 25.0);   // ki=4 복귀에 맞춰 원래 클램프로
  float tout = ff + velocity_kp * velocity_error + velocity_ki * ts + velocity_kd * ed;
  bool sat_p = (tout > MAX_DRIVE_PWM && velocity_error > 0);
  bool sat_n = (tout < -MAX_DRIVE_PWM && velocity_error < 0);
  if (!sat_p && !sat_n) velocity_error_sum = ts;
  float out = ff + velocity_kp * velocity_error + velocity_ki * velocity_error_sum + velocity_kd * ed;
  velocity_pwm_output = constrain((int)out, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);
  // 정지 상태
  if (fabs(target_velocity) < 0.05 && fabs(commanded_velocity) < 0.05) {
    velocity_pwm_output = 0; velocity_error_sum = 0.0;
  }
  velocity_error_old = velocity_error;
}

// ============================ 9. 조향 위치 PID ================================
void steering_pid_control() {
  // 워치독(comms 끊김) 시: 조향을 중앙으로 몰아붙이지 않고 정지(모터 off).
  // 저PWM 스톨로 모터 태우는 것 방지. comms 복구되면 정상 추종 자동 재개.
  if (watchdog_tripped) {
    steering_pwm_output = 0;
    steering_error_old = 0;
    return;
  }
  int target_adc = steerAngleToADC(target_steer_angle);
  steering_error = target_adc - sensorValue;   // +면 ADC 올려야, −면 내려야
  float ed = steering_error - steering_error_old;
  // 히스테리시스 데드밴드: 목표 도달(±STEER_DEADBAND)하면 멈추고, 다시
  // ±STEER_RESUME 넘어야 재출발 → 중앙 근처를 넘나들며 삐- 하는 리밋사이클/버즈 방지.
  static bool settled = true;
  int ae = abs(steering_error);
  if (settled) { if (ae > STEER_RESUME)  settled = false; }
  else         { if (ae <= STEER_DEADBAND) settled = true;  }

  static int breakaway_boost = 0;
  int pwm;
  if (settled) {
    pwm = 0;                    // 목표 도달 → 정지(무부하·무소음)
    breakaway_boost = 0;
  } else {
    pwm = (int)(steering_kp * steering_error + steering_kd * ed);
    // 스티션 극복: 움직일 땐 최소 이 PWM은 실어줘야 실제로 돈다(steady DC).
    // 이 이하로는 '버즈만 나고 안 움직임'이라 아예 최소값으로 끌어올린다.
    if (abs(pwm) < STEER_MIN_MOVE)
      pwm = (steering_error > 0) ? STEER_MIN_MOVE : -STEER_MIN_MOVE;

    // 정지마찰 탈출 부스트: 명령했는데 ADC가 안 변하면(=안 움직임) PWM을 점점
    // 키운다. 움직이기 시작하면 즉시 0으로 리셋 → 과도한 힘이 남지 않는다.
    // prev_sensorValue는 safety_guard가 매 사이클 끝에 갱신하므로 여기선 직전값.
    bool moving = abs(sensorValue - prev_sensorValue) >= STEER_MOVING_DELTA;
    if (moving) {
      breakaway_boost = 0;
    } else {
      int room = MAX_STEER_PWM - abs(pwm);
      if (room < 0) room = 0;
      breakaway_boost = min(breakaway_boost + STEER_BOOST_STEP, room);
    }
    pwm += (pwm > 0) ? breakaway_boost : -breakaway_boost;
  }
  // ★ 부팅 직후 조향 소프트스타트.
  //
  // 부팅 시 목표각은 0(중앙)이다. 바퀴가 중앙에서 크게 벗어나 있으면 첫 사이클부터
  // 최대 PWM(130)이 걸리고, 그 전류 급증에 보드가 리셋된다. 리셋되면 다시 부팅해
  // 또 슬램 → **바퀴가 중앙에 닿기 전에 영원히 리셋을 반복하는 교착**이 된다.
  // 2026-08-18 현장: 모터가 한 번 돈 뒤부터 300초간 41회 끊김, 모터를 멈춰도
  // 회복되지 않았다. 손으로 바퀴를 중앙에 맞춰야만 풀렸다.
  //
  // 부팅 후 STEER_SOFT_MS 동안 상한을 0→MAX 로 선형 증가시켜 돌입 전류를 없앤다.
  // 조향 속도만 잠깐 느려질 뿐 추종 성능에는 영향이 없다(2초 후 정상).
  int lim = MAX_STEER_PWM;
  unsigned long up = millis();
  if (up < STEER_SOFT_MS) {
    lim = (int)((long)MAX_STEER_PWM * up / STEER_SOFT_MS);
    if (lim < STEER_SOFT_MIN) lim = STEER_SOFT_MIN;
  }
  steering_pwm_output = constrain(pwm, -lim, lim);
  steering_error_old = steering_error;
}

// ============================ 10. ★ 안전가드 (스톨 감지) ======================
void safety_guard() {
  // --- 구동 스톨: PWM 높은데 안 움직이고 목표는 있음 ---
  if (abs(velocity_pwm_output) > DRIVE_STALL_PWM &&
      fabs(current_velocity) < DRIVE_STALL_SPEED &&
      (fabs(target_velocity) > 0.05 || openloop_target_pwm != 0)) {
    drive_stall_ms += CONTROL_DT_MS;
    if (drive_stall_ms >= DRIVE_STALL_MS) drive_stalled = true;
  } else {
    drive_stall_ms = 0;
  }
  if (drive_stalled) {
    velocity_pwm_output = 0;             // 컷
    velocity_error_sum = 0;
    // 목표가 0(폐루프) 또는 개루프 지령 0이면 해제
    if (fabs(target_velocity) < 0.05 && openloop_target_pwm == 0)
      drive_stalled = false;
  }

  // --- 조향 스톨: PWM 높은데 ADC 안 변하고 오차 큼 ---
  if (abs(steering_pwm_output) > STEER_STALL_PWM &&
      abs(sensorValue - prev_sensorValue) < STEER_STALL_DELTA &&
      abs(steering_error) > STEER_STALL_ERR) {
    steer_stall_ms += CONTROL_DT_MS;
    if (steer_stall_ms >= STEER_STALL_MS) steer_stalled = true;
  } else {
    steer_stall_ms = 0;
  }
  if (steer_stalled) {
    steering_pwm_output = 0;             // 컷 (모터 보호)
    steering_error_sum = 0;
    steer_cut_ms += CONTROL_DT_MS;
    // 해제 조건 ①목표 근처로 오면(외부에서 밀렸거나 목표가 바뀜) 즉시
    //          ②쿨다운 경과 시 재시도 (영구 사망 방지)
    if (abs(steering_error) < STEER_STALL_ERR ||
        steer_cut_ms >= STEER_STALL_COOLDOWN_MS) {
      steer_stalled = false;
      steer_cut_ms = 0;
      steer_stall_ms = 0;
    }
  } else {
    steer_cut_ms = 0;
  }
  prev_sensorValue = sensorValue;
}

// ============================ 11. 시리얼 파서 + 워치독 ========================
void parseCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  int vi = line.indexOf("VEL:");
  if (vi >= 0) {
    int st = vi + 4, en = line.indexOf(",", st);
    float v = (en >= 0 ? line.substring(st, en) : line.substring(st)).toFloat();
    target_velocity = constrain(v, -3.0, 3.0);
    if (openloop_active) {           // VEL 명령 → 즉시 폐루프 복귀
      openloop_active = false;
      openloop_target_pwm = 0;
      openloop_pwm = 0;
    }
  }
  // 개루프 식별 명령: "PWM:<-140..140>" — 속도 PID를 우회한다.
  int pi = line.indexOf("PWM:");
  if (pi >= 0) {
    int st = pi + 4, en = line.indexOf(",", st);
    int v = (en >= 0 ? line.substring(st, en) : line.substring(st)).toInt();
    openloop_active = true;
    openloop_target_pwm = constrain(v, -MAX_OPENLOOP_PWM, MAX_OPENLOOP_PWM);
    target_velocity = 0.0;
    commanded_velocity = 0.0;
  }
  int si = line.indexOf("STEER:");
  if (si >= 0) {
    int st = si + 6, en = line.indexOf(",", st);
    float s = (en >= 0 ? line.substring(st, en) : line.substring(st)).toFloat();
    target_steer_angle = constrain(s, -STEER_MAX_ANGLE, STEER_MAX_ANGLE);
  }
  last_rx_time = millis();
  watchdog_tripped = false;
}
void check_watchdog() {
  if (millis() - last_rx_time > SERIAL_TIMEOUT_MS) {
    if (!watchdog_tripped) Serial.println("WATCHDOG: serial timeout - stop");
    watchdog_tripped = true;
    target_velocity = 0.0;
    openloop_active = false;      // 개루프도 즉시 해제
    openloop_target_pwm = 0;
    openloop_pwm = 0;
    // 조향은 중앙으로 몰지 않고 정지(freeze)한다. comms 끊긴 뒤 중앙 복귀를
    // 시도하다 breakaway 스티션에 걸려 저PWM 스톨 → 모터 발열/소손하는 것을 방지.
    // (steering_pid_control이 watchdog_tripped를 보고 PWM 0 출력)
  }
}

// ============================ 12. 텔레메트리 (serial_bridge 포맷) =============
void send_telemetry() {
  unsigned long now = millis();
  if (now - last_tel_time < TEL_INTERVAL_MS) return;
  last_tel_time = now;
  const char* slope = "FLAT";
  Serial.print("STATUS_10ms: ENC1="); Serial.print(encoder1count);
  Serial.print(" VEL="); Serial.print(current_velocity, 3);
  Serial.print(" TARGET="); Serial.print(target_velocity, 3);
  Serial.print(" PWM="); Serial.print(velocity_pwm_output);
  Serial.print(" SLOPE="); Serial.print(slope);
  Serial.print(" MODE="); Serial.print(openloop_active ? "OPENLOOP" : "PID");
  // 소나 ping_cm()은 에코 대기(최대 ~11ms/개) 블로킹이라 3개를 한 번에 핑하면
  // 100Hz 제어루프가 밀린다. 매 텔레메트리(50ms)마다 1개씩만 순번으로 핑하고
  // 나머지는 직전 캐시값을 낸다(각 소나 ~150ms/6.7Hz 갱신). 완전 비블로킹이
  // 필요하면 NewPing 타이머(ping_timer) 방식으로 후속 고도화.
  static float sonar_m[SONAR_NUM] = {0, 0, 0};
  static uint8_t si = 0;
  sonar_m[si] = sonar[si].ping_cm() / 100.0;
  si = (si + 1) % SONAR_NUM;
  Serial.print(" SONAR1="); Serial.print(sonar_m[0], 2);
  Serial.print(" SONAR2="); Serial.print(sonar_m[1], 2);
  Serial.print(" SONAR3="); Serial.println(sonar_m[2], 2);
  // 조향 진단 (ADC 현재/목표, PWM, 목표각)
  Serial.print("STEER: ADC="); Serial.print(sensorValue);
  Serial.print(" TGT="); Serial.print(steerAngleToADC(target_steer_angle));
  Serial.print(" PWM="); Serial.print(steering_pwm_output);
  Serial.print(" ANG="); Serial.print(target_steer_angle, 1);
  // ANGACT = ADC로 환산한 '실제' 조향각. ANG(명령각)과의 차이가 추종 오차다.
  Serial.print(" ANGACT="); Serial.print(steerADCToAngle(sensorValue), 1);
  // 공급전압: 현재값과 이 구간 최솟값(스톨 sag 확인용). 보고 후 최솟값 리셋.
  Serial.print(" VCC="); Serial.print(vcc_mv);
  Serial.print(" VMIN="); Serial.println(vcc_min_mv);
  vcc_min_mv = 9999;
  // 스톨 진단
  if (drive_stalled || steer_stalled) {
    Serial.print("STALL: drive="); Serial.print(drive_stalled);
    Serial.print(" steer="); Serial.println(steer_stalled);
  }
}

// ============================ 13. setup ======================================
void setup() {
  Serial.begin(57600);

  // ---- PWM 주파수 상향 (모터 '삐-' 소음 제거) ----
  // analogWrite 기본 주파수가 490Hz라 가청대역에서 모터 코일이 울린다(삐 소리).
  // ★ 구동과 조향의 드라이버 특성이 달라 **타이머별로 주파수를 다르게** 준다.
  //   Timer3 = 핀 2,3,5 → 구동(MOTOR1_PWM=5, MOTOR2_PWM=2)
  //   Timer4 = 핀 6,7,8 → 조향(MOTOR3_PWM=8)
  //   Timer0(핀 4,13)은 millis()/delay()용 — 절대 건드리지 않는다.
  //
  // 실측(2026-08-13):
  //   구동: 490Hz만 정상. 3.9kHz·31kHz에선 **회전 자체가 안 됨**(PWM만 오르고
  //         속도 0 → 스톨가드 컷). 드라이버 게이트 스위칭이 느려서 고주파에선
  //         온타임 대부분이 전환손실로 날아가는 것으로 보인다.
  //   조향: 3.9kHz에서 정상 동작 확인(ADC 684카운트 이동). 490Hz의 가청 소음이
  //         줄어든다.
  // → 구동은 기본값(0x03=/64, 488Hz) 유지, 조향만 0x02(=/8, 3.9kHz)로 올린다.
  TCCR3B = (TCCR3B & 0b11111000) | 0x03;   // 구동 488Hz (드라이버 한계)
  TCCR4B = (TCCR4B & 0b11111000) | 0x02;   // 조향 3.9kHz (소음 저감)

  // (핀13 하트비트 제거: 핀13은 SONAR3 트리거/에코와 충돌 → 소나 정합성 우선)
  int mp[] = {MOTOR1_PWM,MOTOR1_ENA,MOTOR1_ENB,MOTOR2_PWM,MOTOR2_ENA,MOTOR2_ENB,MOTOR3_PWM,MOTOR3_ENA,MOTOR3_ENB};
  for (int i = 0; i < 9; i++) pinMode(mp[i], OUTPUT);
  all_motors_off();                 // 부팅 시 안전 상태
  initEncoders(); clearEncoderCount(1); clearEncoderCount(2);
  sensorValue = analogRead(Steering_Sensor);
  prev_sensorValue = sensorValue;
  prev_time = last_rx_time = last_tel_time = millis();
  Serial.println("HENES firmware ready: drive PID + steer PID + STALL guard + watchdog");
  Serial.print("  STEER cal: center="); Serial.print(STEER_CENTER);
  Serial.print(" L="); Serial.print(STEER_LEFT_MAX);
  Serial.print(" R="); Serial.print(STEER_RIGHT_MAX);
  Serial.print("  MAX_PWM drive="); Serial.print(MAX_DRIVE_PWM);
  Serial.print(" steer="); Serial.println(MAX_STEER_PWM);
}

// ============================ 14. loop (100Hz 제어) ===========================
void loop() {
  // 시리얼 수신
  while (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    parseCommand(cmd);
  }
  check_watchdog();

  unsigned long now = millis();
  if (now - prev_time >= CONTROL_DT_MS) {
    prev_time = now;

    // 센서 갱신
    encoder1count = readEncoder(1);
    encoder2count = readEncoder(2);
    sensorValue = analogRead(Steering_Sensor);
    // 공급전압 감시 (스톨 순간 sag를 놓치지 않도록 매 주기 샘플 + 최솟값 유지)
    vcc_mv = readVccMv();
    if (vcc_mv > 0 && vcc_mv < vcc_min_mv) vcc_min_mv = vcc_mv;
    calculate_velocity();

    // 제어 계산
    apply_acceleration_limit();
    velocity_pid_control();
    steering_pid_control();

    // ★ 안전가드 (PID 결과를 덮어씀)
    safety_guard();

    // 모터 출력
    front_motor_control(velocity_pwm_output);
    rear_motor_control(velocity_pwm_output);
    steer_motor_control(steering_pwm_output);
  }

  send_telemetry();
}
