/* 전원
- ESC로부터 VIN, GND 받음
- ESP32는 크리스털이 있어서 usb 포트 연결 유뮤에 상관없이 안정적인 pwm이 나옴
- 33iot나 33ble는 usb 포트 연결하면 pwm이 바뀌는 문제 발생 (usb start of frame freq 기반으로 보정함)
*/

/* Xbee연결
-  Xbee    Ard ESP32
-  DIN  <->  D3
-  DOUT <->  D2
*/

/* 서보연결
- ESC: D5
- Steer: D4
*/

// Servo 라이브러리를 사용하여 ESC 모터와 서보 모터를 제어하는 객체 생성

#include <ESP32Servo.h>

int esc_us_stop = 1530;
int esc_us_fwd_strt = 1555;
int esc_us_slow_max = 1565;
int esc_us_fast_max = 1588;

int esc_us_rev_strt = 1510;
int esc_us_rev_max = 1490;
int esc_us_max = esc_us_stop;

int steer_us_center = 1477;
int steer_us_small_gap = 150;
int steer_us_large_gap = 300;



// int esc_us_stop = 1500;
// int esc_us_fwd_strt = 1500;
// int esc_us_slow_max = 1540;
// int esc_us_fast_max = 1550;
// int esc_us_rev_strt = 1500;
// int esc_us_rev_max = 1465;
// int esc_us_max = esc_us_stop;


// int steer_us_center = 1493;
// int steer_us_small_gap = 150;
// int steer_us_large_gap = 300;
int steer_us_small_R = steer_us_center - steer_us_small_gap;
int steer_us_small_L = steer_us_center + steer_us_small_gap;
int steer_us_large_R = steer_us_center - steer_us_large_gap;
int steer_us_large_L = steer_us_center + steer_us_large_gap;
int steer_us = steer_us_center;  // 서보 모터 각도

// int steer_us_R = steer_us_small_R;
// int steer_us_L = steer_us_small_L; 
int steer_us_R = steer_us_large_R;
int steer_us_L = steer_us_large_L;

int btn_val = 0;
// 상: 1 (정지), 하: 2 (후진), 좌: 3 (천천히), 우: 4 (빨리)
// 토글을 내리면 메뉴얼, 올리면 오토



// 시리얼 통신 속도 (보드레이트)
const long baud = 57600;


ESP32PWM esc_pwm;
ESP32PWM steer_pwm;
int period_us = 20000;  // 20ms
int freq = pow(10, 6) / period_us; // 50Hz
int resolution = 14;
double max_duty = pow(2, resolution);
double us_2_duty_scale = max_duty / period_us;


// 모터의 핀 번호
int esc_pin = 4;
int steer_pin = 5;

// xbee 통신 핀번호
int rx_1_pin = 2;
int tx_1_pin = 3;

int joy_sp_init = 512;
bool joy_first_time = false;
int prev_x = 0;
int prev_y = 0;
int cnt = 0;

// 조이스틱 데이터의 유니온 (다양한 데이터 형식을 함께 사용)
union JoyUnion {
  struct {
    char header[3];  // 헤더
    int axis[2];      // 조이스틱 좌표
    int btn;          // 버튼 상태
  } joy_data;
  char buffer[sizeof(joy_data)];  // 버퍼
};



// 조이스틱 데이터의 유니온 배열
union JoyUnion joy_union[3];  // 0: zigbee, 1: orin
unsigned int joy_struct_size_m1 = sizeof(JoyUnion) - 1;  // 조이스틱 데이터 크기 - 1

bool rcv_strt[2] = {false, false};   // 데이터 수신 시작 여부 배열
int data_cnt[2] = {0 ,0};  // 데이터 카운터 배열


int esc_us = esc_us_stop;  // ESC 모터 속도

int control_init[2][2] = {{512, 512}, {steer_us_center, esc_us_stop}};
int mode_id = 0;  // 0: zigbee, 1: orin (로봇 제어 모드)

unsigned long start_time = 0;
int Tms = 20;  // 주기적인 통신 주기

int speed_mode = 0; // 0: stop, 1: slow, 2: fast, 3: reverse

int rcv_msg_success_cnt = 0;
int prev_rcv_msg_success_cnt = 0;
int disconnect_cnt = 0;

void setup() {


  Serial.begin(baud);
  Serial1.begin(baud, SERIAL_8N1, rx_1_pin, tx_1_pin);
  
  for (int i = 0; i < 2; i++) {   
    joy_union[i].joy_data.axis[0] = control_init[i][0];
    joy_union[i].joy_data.axis[1] = control_init[i][1];
  }

  // 아두이노 -> PC 메시지용, pwm을 전송함
  joy_union[2].joy_data.header[0] = 'A';
  joy_union[2].joy_data.header[1] = 'R';
  joy_union[2].joy_data.header[2] = 'D';
  joy_union[2].joy_data.axis[0] = control_init[1][0];
  joy_union[2].joy_data.axis[1] = control_init[1][1];
  joy_union[2].joy_data.btn = 0;

  esc_pwm.attachPin(esc_pin, freq, resolution);  // ESC 모터 서보 객체에 연결  
  steer_pwm.attachPin(steer_pin, freq, resolution);  // steer 모터 서보 객체에 연결  
}

void loop() {
  rcvComm();  // zigbee와 jetson orin 간의 통신 함수 호출
  

  // 주기적으로 아두이노 데이터 전송
  if (millis() - start_time > Tms) {
    start_time = millis();

    // 아두이노에서 pc로 전송
    Serial.write(joy_union[2].buffer, sizeof(joy_union[2].buffer));

    // 디버깅용
    // Serial.println((String)joy_union[2].joy_data.header[0] + joy_union[2].joy_data.header[1] + joy_union[2].joy_data.header[2] +
    // + "," + joy_union[2].joy_data.axis[0]+ "," + joy_union[2].joy_data.axis[1]);

    if (prev_rcv_msg_success_cnt < rcv_msg_success_cnt) {
      disconnect_cnt = 0;
    } else {
      disconnect_cnt++;
      if (disconnect_cnt > 100) {
        for (int i = 0; i < 2; i++) {
          joy_union[i].joy_data.axis[0] = control_init[i][0];
          if (joy_union[i].joy_data.axis[1] > control_init[i][1]) {
            joy_union[i].joy_data.axis[1] -= 2;
            joy_union[i].joy_data.axis[1] = max(joy_union[i].joy_data.axis[1], control_init[i][1]);
          }          
        }
      }
    }
    prev_rcv_msg_success_cnt = rcv_msg_success_cnt;
  }  

  // 로봇 제어 모드에 따라 속도 및 각도 값 설정
  // st = map(joy_union[mode_id].joy_data.axis[0], 0, 1023, 1200, 1800);    
  if (mode_id == 0) {
    
    esc_us = esc_us_stop;
    steer_us_R = steer_us_large_R;
    steer_us_L = steer_us_large_L;
    esc_us_max = esc_us_fast_max;

    int v_adc = joy_union[mode_id].joy_data.axis[1];    
    if (speed_mode == 1 || speed_mode == 2) {
      if (speed_mode == 1) {  // slow      
        steer_us_R = steer_us_small_R;
        steer_us_L = steer_us_small_L;
        esc_us_max = esc_us_slow_max;
      }
      esc_us = map(v_adc, joy_sp_init, 1023, esc_us_fwd_strt, esc_us_max);      
      esc_us = constrain(esc_us, esc_us_fwd_strt, esc_us_max );      
    } else if (speed_mode == 3) { // reverse
      esc_us = map(v_adc, 0, joy_sp_init, esc_us_rev_max, esc_us_rev_strt);
      esc_us = constrain(esc_us, esc_us_rev_max, esc_us_rev_strt);            
    }
    steer_us = map(joy_union[mode_id].joy_data.axis[0], 0, 1023, steer_us_R, steer_us_L);
  } else if (mode_id == 1) {
    steer_us = joy_union[mode_id].joy_data.axis[0];
    esc_us = joy_union[mode_id].joy_data.axis[1];
  }
  steer_us = constrain(steer_us, steer_us_large_R, steer_us_large_L);

  // 아두이노 -> PC
  joy_union[2].joy_data.axis[0] = steer_us;
  joy_union[2].joy_data.axis[1] = esc_us;
  joy_union[2].joy_data.btn = btn_val;
  

  
  // Serial.println((String)"Btn = " + btn_val + " Steer = " + steer_us + " Throttle = " + esc_us);
  // Serial.println((String)"Btn = " + btn_val + " Steer = " + steer_us + " Throttle = " + esc_us);  
  
  esc_pwm.write(us_2_duty_scale * esc_us);
  steer_pwm.write(us_2_duty_scale * steer_us);
  
}

// zigbee와 jetson orin 간의 통신을 처리하는 함수
void rcvComm() {  
  for (int i = 0; i < 2; i++) {
    bool rcv_msg = false;  // 문자를 수신했는지 여부
    char c = '\0';  // 수신된 문자
    if (i == 0 && Serial1.available()) {
      c = Serial1.read();  // Zigbee 모듈로부터 문자 수신
      // Serial.println(c);
      rcv_msg = true;      
    } else if (i == 1 && Serial.available()) {
      c = Serial.read();  // 임베디드 컴퓨터로부터 문자 수신
      rcv_msg = true;
    }
    if (!rcv_msg) {
      continue;  // 문자를 수신하지 않은 경우 다음 반복으로 이동
    }    
    // 데이터 수신을 시작하는 지점 식별 ('J', 'O', 'Y' 순서로 수신됨)
    if (c == 'J' && rcv_strt[i] == false) {
      rcv_strt[i] = true;
      joy_union[i].buffer[data_cnt[i]] = c;
      data_cnt[i] = 1;   
    } else if (c == 'O' && data_cnt[i] == 1) {
      joy_union[i].buffer[data_cnt[i]] = c;
      data_cnt[i]++;      
    } else if (c == 'Y' && data_cnt[i] == 2) {
      joy_union[i].buffer[data_cnt[i]] = c;
      data_cnt[i]++;      
    } else {
      // 데이터 수신 중
      if (rcv_strt[i] && data_cnt[i] > 2) {
        joy_union[i].buffer[data_cnt[i]] = c;
        data_cnt[i]++;
        // 조이스틱 데이터 수신 완료
        if (data_cnt[i] == joy_struct_size_m1) {          
          rcv_msg_success_cnt++;
          
          // 수신된 데이터를 joy_union[i].joy_data에 복사
          memcpy(&joy_union[i].joy_data, joy_union[i].buffer, sizeof(joy_union[i].joy_data));
          rcv_strt[i] = false;        
          data_cnt[i] = 0; 

          
          // 버튼값은 조이스틱에서만 읽어옴
          if (i == 0) {        
            int rcv_raw_btn = joy_union[0].joy_data.btn;
            int default_val = 0;
            if (rcv_raw_btn < 6) { // 토글이 아래로 되어있으면 수동모드이며 버튼이 안눌리면 5, 그 외의 버튼은 1,2,3,4로 나옴 (up:1, dn:2, lft:3, rgt:4)
              mode_id = 0;  
              if (joy_first_time) {
                joy_sp_init = joy_union[0].joy_data.axis[1];
                joy_first_time = false;
              }
            } else {
              mode_id = 1;
              default_val = 10;
            }
            int tmp_btn_val = rcv_raw_btn - default_val;

            // Serial.println((String)rcv_raw_btn + "," + default_val);
            if (tmp_btn_val < 5) {
              btn_val = tmp_btn_val;
              speed_mode = 0;
              switch (btn_val) {
                case 1:         
                  break;
                case 2:        
                  speed_mode = 3;
                  break;
                case 3:        
                  speed_mode = 1;
                  break;                
                case 4:                          
                  speed_mode = 2;
                  break;
                default:
                  break;      
              }
            }
          }
        }
      } else {
        rcv_strt[i] = false;        
        data_cnt[i] = 0;        
      }      
    }
  }  
}
 
