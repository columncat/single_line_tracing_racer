#include <ros.h>
#include <std_msgs/String.h>
#include <MotorDriver.h>
#include <Servo.h>

#define SERVO_PIN 8 // 3 or 8
#define TRIG_A 3
#define ECHO_A 2
#define TRIG_B 12
#define ECHO_B 11
#define MOTORTYPE YF_PMR3

uint8_t SerialDebug = 1;      // unused in code, but neccesary for MotorDriver.h
uint8_t servoNeutral = 88;    // Change this value for your settings
uint16_t stop_distance = 620; // Change this value for your settings
bool aa_mode = false;         // true if instant motor activation is needed
Servo servo;
MotorDriver motorDriver = MotorDriver(MOTORTYPE);

/* 'l' stands for left, 'r' stands for right
    ex) "l30", "r45"                         */
void servoCb(const std_msgs::String &servo_cmd)
{
    // 'l20'
    uint8_t value = (servo_cmd.data[1] - '0') * 10 + (servo_cmd.data[2] - '0');
    switch (servo_cmd.data[0])
    {
    case 'l':
        servo.write(servoNeutral - value);
        break;
    case 'r':
        servo.write(servoNeutral + value);
        break;
    default:
        break;
    }
    return;
}

/* 's' for stop, 'f' for forward, 'b' for backward, 'a' for auto
    ex) "s", "f255", "b100", "a"                                 */
void motorCb(const std_msgs::String &motor_cmd)
{
    uint8_t value = 0;
    switch (motor_cmd.data[0])
    {
    case 's':
        aa_mode = false;
        motorDriver.setMotor(0, 0);
        break;
    case 'b':
        aa_mode = false;
        value = (motor_cmd.data[1] - '0') * 100 + (motor_cmd.data[2] - '0') * 10 + (motor_cmd.data[3] - '0') * 1;
        motorDriver.setMotor(value, value); // sign changed may needed
        break;
    case 'f':
        aa_mode = false;
        value = (motor_cmd.data[1] - '0') * 100 + (motor_cmd.data[2] - '0') * 10 + (motor_cmd.data[3] - '0') * 1;
        motorDriver.setMotor(-1 * value, -1 * value);
    case 'a':
        aa_mode = true;
        break;

    default:
        break;
    }
}

void get_ping(uint16_t *dist)
{
    digitalWrite(TRIG_A, LOW);
    digitalWrite(ECHO_A, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_A, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_A, LOW);
    int dur = pulseIn(ECHO_A, HIGH);
    dist[0] = dur / 3.8 > 9999 ? 9999 : dur / 3.8; // units in mm

    digitalWrite(TRIG_B, LOW);
    digitalWrite(ECHO_B, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_B, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_B, LOW);
    dur = pulseIn(ECHO_B, HIGH);
    dist[1] = dur / 3.8 > 9999 ? 9999 : dur / 3.8;
}

ros::NodeHandle nh;
std_msgs::String ping_val;
ros::Subscriber<std_msgs::String> sub_servo("servo", servoCb);
ros::Subscriber<std_msgs::String> sub_motor("motor", motorCb);

void setup()
{
    Serial.begin(9600); // if rosserial is activated, try 57600
    nh.initNode();
    nh.subscribe(sub_servo);
    nh.subscribe(sub_motor);
    servo.attach(SERVO_PIN);
    motorDriver.motorConfig(1, 1);
    pinMode(TRIG_A, OUTPUT);
    pinMode(ECHO_A, INPUT);
    pinMode(TRIG_B, OUTPUT);
    pinMode(ECHO_B, INPUT);

    servo.write(servoNeutral);
}

void loop()
{
    if (aa_mode)
    {
        uint8_t i;
        uint16_t dist[2];
        get_ping(dist);
        Serial.print(dist[0]);
        Serial.print(" , ");
        Serial.println(dist[1]);
        if (dist[0] < stop_distance || dist[1] < stop_distance)
            motorDriver.setMotor(0, 0);
        else
            motorDriver.setMotor(-255, -255);
    }
    nh.spinOnce();
    delay(10);
}
