#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

// RS232 통신 포트 설정 (TX3 = 14, RX3 = 15)
#define MOTOR_SERIAL Serial3  

// 모터 속도 범위
#define MAX_SPEED 255  
#define MIN_SPEED 0     

// ROS에서 `cmd_vel`을 받으면 실행되는 함수
void cmdVelCallback(const geometry_msgs::Twist& msg) {
    float linear = msg.linear.x;  // 전진/후진 속도
    float angular = msg.angular.z; // 좌우 회전 속도

    // 왼쪽, 오른쪽 모터 속도 계산
    int left_speed = constrain((linear - angular) * MAX_SPEED, -MAX_SPEED, MAX_SPEED);
    int right_speed = constrain((linear + angular) * MAX_SPEED, -MAX_SPEED, MAX_SPEED);

    // 방향 설정 (0: 후진, 1: 정지, 2: 전진)
    int left_dir = (left_speed > 0) ? 2 : (left_speed < 0) ? 0 : 1;
    int right_dir = (right_speed > 0) ? 2 : (right_speed < 0) ? 0 : 1;

    // RS232 명령어 생성
    char command[50];
    sprintf(command, "L,%d,%d R,%d,%d\n", left_dir, abs(left_speed), right_dir, abs(right_speed));

    // 모터 드라이버로 전송
    MOTOR_SERIAL.println(command);
}

// ROS 구독 설정
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmdVelCallback);

void setup() {
    nh.initNode();
    nh.subscribe(sub);
    MOTOR_SERIAL.begin(115200);
}

void loop() {
    nh.spinOnce();
    delay(10);
}
