#include <Servo.h>
#include <ArduinoSTL.h>
#include <deque>

using namespace std;

const int FPS = 60;

deque<float> front_check;
deque<float> left_check;
deque<float> right_check;

// float SIDE_PARLKING_FRONT[60] = {300,300,300,300,300,300
//                             ,300,300,300,300,300,300
//                             ,300,300,300,300,300,300
//                             ,300,300,300,300,300,300
//                             ,300,300,300,300,300,300
//                             ,300,300,300,300,300,300
//                             ,300,300,300,300,300,300
//                             ,300,300,300,300,300,300
//                             ,300,300,300,300,300,300
//                             ,300,300,300,300,300,300
// };

Servo servo;

// 노이즈 방지를 위한 센서값 평균을 측정하기 위한 변수와 배열

deque<float> front_queue;
deque<float> left_queue;
deque<float> right_queue;
int steering_degree = 0;


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

const int MAX_DISTANCE = 300; // 초음파 센서의 최대 감지거리


// 센서값 전역변수
float uw_front;
float uw_left;
float uw_right;
bool ir_right;
bool ir_left;


int state = 0;

// 자동차 튜닝 파라미터 =====================================================================
int detect_ir = 28; // 검출선이 흰색과 검정색 비교


int punch_pwm = 200; // 정지 마찰력 극복 출력 (0 ~ 255)
int punch_time = 50; // 정지 마찰력 극복 시간 (단위 msec)
int stop_time = 300; // 전진후진 전환 시간 (단위 msec)

int max_ai_pwm = 130; // 자율주행 모터 최대 출력 (0 ~ 255)
int min_ai_pwm = 70; // 자율주행 모터 최소 출력 (0 ~ 255)

int angle_offset = 0; // 서보 모터 중앙각 오프셋 (단위: 도)
int angle_limit = 55; // 서보 모터 회전 제한 각 (단위: 도)

int front_detect = 200; // 전방 감지 거리 (단위: mm)
int front_start = 160; // 전방 출발 거리 (단위: mm)
int front_stop = 70; // 전방 멈춤 거리 (단위: mm)

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

float QueueSum(deque<float> Q){
    float sum = 0;
    for(float num : Q){
        sum += num;
    }
    return sum;
}

// 이전 5프레임의 초음파센서값을 평균내서 반환한다
float GetFrontDistance(float current){
    if(front_queue.size() < 3){
        front_queue.push_back(current);
        return QueueSum(front_queue) / front_queue.size();
    }
    front_queue.push_back(current);
    front_queue.pop_front();
    return QueueSum(front_queue) / 3;
}

float GetLeftDistance(float current){
    if(left_queue.size() < 3){
        left_queue.push_back(current);
        return QueueSum(left_queue) / left_queue.size();
    }
    left_queue.push_back(current);
    left_queue.pop_front();
    return QueueSum(left_queue) / 3;
}

float GetRightDistance(float current){
    if(right_queue.size() < 3){
        right_queue.push_back(current);
        return QueueSum(right_queue) / right_queue.size();
    }
    right_queue.push_back(current);
    right_queue.pop_front();
    return QueueSum(right_queue) / 3;
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


//센서값 설정
void SetSensor(){

    float current_front = GetDistance(FC_TRIG, FC_ECHO);
    float current_left = GetDistance(FC_TRIG, FC_ECHO);
    float current_right = GetDistance(FC_TRIG, FC_ECHO);    
    uw_front = GetFrontDistance(current_front);
    uw_left = GetLeftDistance(current_left);
    uw_right = GetRightDistance(current_right);
    ir_left = ir_sensing(IR_L);
    ir_right = ir_sensing(IR_R);


    // front_check.push_back(current_front);
    // front_check.pop_front();
    // left_check.push_back(current_left);
    // left_check.pop_front();
    // right_check.push_back(current_right);
    // right_check.pop_front();

    // Serial.print("front: ");
    // Serial.print(GetSUMSQ(0));
    // Serial.print("  left: ");
    // Serial.print(GetSUMSQ(1));
    // Serial.print("  front: ");
    // Serial.println(GetSUMSQ(2));
    

    // // 디버깅용 프린트
    // // Serial.print("left: ");
    // // Serial.print(uw_left);
    // // Serial.print("  right: ");
    // // Serial.print(uw_right);
    
    // Serial.print("  front: ");
    // Serial.println(GetDistance(FC_TRIG, FC_ECHO));
    
}

// 센서의 값에 따라서
// state값 리턴
// ex) 적외선 left, right 다 false면 직진이니까 직진에 해당하는 state 반환


//////////////////////////////////////////////////////////////
// uw 센서값 범위                                              //
// left, right 도로 정중앙 기준 양옆 차선까지는 약 65~90정도 측정됨    //
// front 블럭 정중앙 기준 앞 블록까지 약 50정도 측정                 //
//////////////////////////////////////////////////////////////

void SetState(){
    state = 0;


    // 정지선
    if(ir_left == true && ir_right == true){
        state = 4;
    }
    else if(uw_front < 270){
        state = 3;
    }
    //직진(차선 검출 X)
    else if(ir_left==false && ir_right==false){
        state=0;
    }
    //좌회전(오른쪽 차선 검출)
    else if(ir_left==false && ir_right==true){
        state=1;
    }
    //우회전(왼쪽 차선 검출)
    else if(ir_left==true && ir_right==false){
        state=2;
    }


}

// 직진에 해당하게끔 스티어링이랑 속도 조정
void Straight(){
    if(steering_degree < 0){
        steering_degree++;
    }
    else if(steering_degree > 0){
        steering_degree--;
    }
    compute_steering = steering_degree / 5.0;
    compute_speed = 1;
}

// 좌회전
void LeftTurn(){
    steering_degree = constrain(steering_degree-1, -5, 5);
    compute_steering = steering_degree / 5.0;
    compute_speed = 0.7;
}

// 우회전
void RightTurn(){
    steering_degree = constrain(steering_degree+1, -5, 5);
    compute_steering = steering_degree / 5.0;
    compute_speed = 0.7;
}

// 정지선

void StopLine(){
    compute_steering = 0;
    compute_speed = 0;
    SetSpeed(compute_speed);
    SetSteering(compute_steering);
    delay(3000);
    compute_steering = 0;
    compute_speed = 1;
    SetSpeed(compute_speed);
    SetSteering(compute_steering);
    // delay(200);
}


// 오른쪽 장애물
void RightObstacle(){

}

// 왼쪽 장애물
void LeftObstacle(){

}

// float abs_num(float num){
//     if(num > 0){
//       return num;
//     }
//     else{
//       return (-1) * num;
//     }
// }

// float GetSUMSQ(int flag){
//     int SUMSQ = 0;
//     if(flag == 0){  // front
//         for(int i = 0; i < FPS; i++){
//             SUMSQ += abs_num(front_check[i] - SIDE_PARLKING_FRONT[i]);
//         }
//     }
//     else if(flag == 1){ // left
//         for(int i = 0; i < FPS; i++){
//             SUMSQ += abs_num(left_check[i] - SIDE_PARLKING_FRONT[i]);
//         }
//     }
//     else{   // right
//         for(int i = 0; i < FPS; i++){
//             SUMSQ += abs_num(right_check[i] - SIDE_PARLKING_FRONT[i]);
//         }
//     }
//     return SUMSQ;
// }


// 전방 장애물
// 일단 왼쪽으로 회전
void FrontObstacle(){
    while(GetRightDistance(GetDistance(R_TRIG, R_ECHO)) > 70 && !ir_sensing(IR_L) && !ir_sensing(IR_R)){
        SetSpeed(0.4);
        SetSteering(-1);
    }
    compute_speed = 0.6;
    compute_steering = 0;
}




// void SideParking(){
//     SetSteering(0.7);
//     SetSpeed(0.5);
//     delay(1000);
//     SetSteering(-1);
//     SetSpeed(0.5);
//     delay(500);

//     SetSteering(0);
//     while(GetFrontDistance(GetDistance(FC_TRIG, FC_ECHO)) > 45){
//         continue;
//     }
//     SetSpeed(-1);
//     while(GetFrontDistance(GetDistance(FC_TRIG, FC_ECHO)) < 200){
//         continue;
//     }

//     SetSpeed(0.5);
//     SetSteering(-1);
//     delay(1000);
//     SetSpeed(0.5);
//     SetSteering(1);
//     delay(1000);
//     SetSteering(0);

// }

void driving() {

    // SideParking();

    // 한 번의 루프마다 각각 센서값 설정
    SetSensor();

    //받아온 센서값을 바탕으로 이번 루프의 state결정
    SetState();

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
    case 3:
        FrontObstacle();
        break;
    case 4:
        StopLine();
    }

    SetSpeed(compute_speed);
    SetSteering(compute_steering);
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

//    for(int i = 0; i < FPS; i++){
//        front_check.push_back(MAX_DISTANCE);
//        left_check.push_back(MAX_DISTANCE);
//        right_check.push_back(MAX_DISTANCE);
//    }
}

void loop() {
    driving();
}
