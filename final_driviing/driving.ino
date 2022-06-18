#include <Servo.h>
Servo servo;

const int SERVO1_PIN = 9;      // 서보모터1 연결핀
const int IR_R = A3;  //  적외선센서 우측 핀
const int IR_L = A4;  // 적외선센서 좌측 핀

const int M1_PWM = 5;   // DC모터1 PWM 핀 왼
const int M1_DIR1 = 7;   // DC모터1 DIR1 핀
const int M1_DIR2 = 8;   // DC모터 1 DIR2 핀

const int M2_PWM = 6;   // DC모터2 PWM 핀
const int M2_DIR1 = 11;   // DC모터2 DIR1 핀
const int M2_DIR2 = 12;   // DC모터2 DIR2 핀

const int FC_TRIG  = 13;   // 전방 초음파 센서 TRIG 핀
const int FC_ECHO = 10;  // 전방 초음파 센서 ECHO 핀
const int L_TRIG = A2;  // 좌측 초음파 센서 TRIG 핀
const int L_ECHO = A1;  // 좌측 초음파 센서 ECHO 핀
const int R_TRIG = 2;   // 우측 초음파 센서 TRIG 핀
const int R_ECHO = A5;  // 우측 초음파 센서 ECHO 핀

const int MAX_DISTANCE = 2000; // 초음파 센서의 최대 감지거리

float center;
float left;
float right;

int state = 0;

// 자동차 튜닝 파라미터 =====================================================================
int detect_ir = 27; // 검출선이 흰색과 검정색 비교


int punch_pwm = 200; // 정지 마찰력 극복 출력 (0 ~ 255)
int punch_time = 50; // 정지 마찰력 극복 시간 (단위 msec)
int stop_time = 300; // 전진후진 전환 시간 (단위 msec)

int max_ai_pwm = 130; // 자율주행 모터 최대 출력 (0 ~ 255)
int min_ai_pwm = 70; // 자율주행 모터 최소 출력 (0 ~ 255)

int angle_offset = 0; // 서보 모터 중앙각 오프셋 (단위: 도)
int angle_limit = 55; // 서보 모터 회전 제한 각 (단위: 도)

int center_detect = 200; // 전방 감지 거리 (단위: mm)
int center_start = 160; // 전방 출발 거리 (단위: mm)
int center_stop = 70; // 전방 멈춤 거리 (단위: mm)

int side_detect = 100; // 좌우 감지 거리 (단위: mm)


float cur_steering;
float cur_speed;
float compute_steering;
float compute_speed;

float max_pwm;
float min_pwm;

// 초음파 거리측정
float GetDistance(int trig, int echo)
{
    digitalWrite(trig, LOW);
    delayMicroseconds(4);
    digitalWrite(trig, HIGH);
    delayMicroseconds(20);
    digitalWrite(trig, LOW);

    unsigned long duration = pulseIn(echo, HIGH, 5000);
    if (duration == 0)
        return MAX_DISTANCE;
    else
        return duration * 0.17;     // 음속 340m/s
}

bool ir_sensing(int pin) {
    if(analogRead(pin)>detect_ir)
      return false;
    else
      return true;
}

// 앞바퀴 조향
void SetSteering(float steering)
{
    cur_steering = constrain(steering, -1, 1);// constrain -1~ 1 값으로 제한

    float angle = cur_steering * angle_limit;
    int servoAngle = angle + 90;
    servoAngle += angle_offset;

    servoAngle = constrain(servoAngle, 0, 180);
    servo.write(servoAngle);
    delay(20);
}


// 뒷바퀴 모터회전
void SetSpeed(float speed)
{
    speed = constrain(speed, -1, 1);

    if ((cur_speed * speed < 0) // 움직이는 중 반대 방향 명령이거나
            || (cur_speed != 0 && speed == 0)) // 움직이다가 정지라면
    {
        cur_speed = 0;
        digitalWrite(M1_PWM, HIGH);
        digitalWrite(M1_DIR1, LOW);
        digitalWrite(M1_DIR2, LOW);

        digitalWrite(M2_PWM, HIGH);
        digitalWrite(M2_DIR1, LOW);
        digitalWrite(M2_DIR2, LOW);

        if (stop_time > 0)
            delay(stop_time);
    }

    if (cur_speed == 0 && speed != 0) // 정지상태에서 출발이라면
    {
        if (punch_time > 0)
        {
            if (speed > 0)
            {
                analogWrite(M1_PWM, punch_pwm);
                digitalWrite(M1_DIR1, HIGH);
                digitalWrite(M1_DIR2, LOW);

                analogWrite(M2_PWM, punch_pwm);
                digitalWrite(M2_DIR1, HIGH);
                digitalWrite(M2_DIR2, LOW);
            }
            else if (speed < 0)
            {
                analogWrite(M1_PWM, punch_pwm);
                digitalWrite(M1_DIR1, LOW);
                digitalWrite(M1_DIR2, HIGH);

                analogWrite(M2_PWM, punch_pwm);
                digitalWrite(M2_DIR1, LOW);
                digitalWrite(M2_DIR2, HIGH);
            }
            delay(punch_time);
        }
    }

    if (speed != 0) // 명령이 정지가 아니라면
    {
        int pwm = abs(speed) * (max_pwm - min_pwm) + min_pwm;           // 0 ~ 255로 변환

        if (speed  > 0)
        {
            analogWrite(M1_PWM, pwm);
            digitalWrite(M1_DIR1, HIGH);
            digitalWrite(M1_DIR2, LOW);

            analogWrite(M2_PWM, pwm);
            digitalWrite(M2_DIR1, HIGH);
            digitalWrite(M2_DIR2, LOW);
        }
        else if (speed < 0)
        {
            analogWrite(M1_PWM, pwm);
            digitalWrite(M1_DIR1, LOW);
            digitalWrite(M1_DIR2, HIGH);

            analogWrite(M2_PWM, pwm);
            digitalWrite(M2_DIR1, LOW);
            digitalWrite(M2_DIR2, HIGH);
        }
    }
    cur_speed = speed;
}


////////////////////////////////////////////////
// 각 state별로 함수로 분리                       //
// 각 함수에 필요한 센서 파라미터 추가할 것!           //
// state가 추가되면 함수도 추가할 것!               //
///////////////////////////////////////////////


// 센서의 값에 따라서
// state값 리턴
// ex) 적외선 left, right 다 false면 직진이니까 직진에 해당하는 state 반환
int SetState(bool ir_left, bool ir_right, float uw_left, float uw_right, float uw_front){
    return 0;
}

// 직진에 해당하게끔 스티어링이랑 속도 조정
void Straight(){

}

// 우회전
void RightTurn(){

}

// 좌회전
void LeftTurn(){

}

// 오른쪽 장애물
void RightObstacle(){

}

// 왼쪽 장애물
void LeftObstacle(){

}


// 전방 장애물
void FrontObstacle(){

}

void driving() {

    // 한 번의 루프마다 각각 센서값 받아오기
    float uw_center = GetDistance(FC_TRIG, FC_ECHO);
    float uw_left = GetDistance(L_TRIG, L_ECHO);
    float uw_right = GetDistance(R_TRIG, R_ECHO);
    bool ir_left = ir_sensing(IR_L);
    bool ir_right = ir_sensing(IR_R);

    // 받아온 센서값을 바탕으로 이번 루프의 state결정
    state = SetState(ir_left, ir_right, uw_left, uw_right, uw_center);

    // case별로 분기 추가하기!
    // case 별로 상수 DEFINE 해서 숫자 없애기!
    switch (state)
    {
    case 0:
        Straight();
        break;
    case 1:
        LeftTurn();
        break;
    case 2:
        RightTurn();
        break;
    }


}

void straight() { // 기본주행    
    if (ir_sensing(IR_R) == false && ir_sensing(IR_L) == false ) {  //차선이 검출되지 않을 경우 직진
        compute_steering = 0;
        compute_speed = 0.5;
//        Serial.print("straight\n");
    }

  else if (ir_sensing(IR_R) == true && ir_sensing(IR_L) == false) { // 오른쪽 차선이 검출된 경우
        compute_steering = -1;
        compute_speed = 0.1;
//        Serial.print("left\n");
    }

  else if (ir_sensing(IR_L) == true && ir_sensing(IR_R) == false ) { //왼쪽 차선이 검출된 경우 
        compute_steering = 1;
        compute_speed = 0.1;
//        Serial.print("right\n");
    }
}

void setup() {

    Serial.begin(115200);
    servo.attach(SERVO1_PIN); //서보모터 초기화

    pinMode(IR_R, INPUT);
    pinMode(IR_L, INPUT);

    pinMode(M1_PWM, OUTPUT);
    pinMode(M1_DIR1, OUTPUT);
    pinMode(M1_DIR2, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M2_DIR1, OUTPUT);
    pinMode(M2_DIR2, OUTPUT);
    
    pinMode(FC_TRIG, OUTPUT);
    pinMode(FC_ECHO, INPUT);
  
    pinMode(L_TRIG, OUTPUT);
    pinMode(L_ECHO, INPUT);
  
    pinMode(R_TRIG, OUTPUT);
    pinMode(R_ECHO, INPUT);

    max_pwm = max_ai_pwm;
    min_pwm = min_ai_pwm;

    SetSteering(0);
    SetSpeed(0);
}

void loop() {
    driving();
}
