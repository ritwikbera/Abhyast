#include <Encoder.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>


/////////---PINS----/////////////////
int MaxSpeed = 3.0;

#define CW 0
#define CCW 1

#define LMotPWMPin  45 
#define RMotPWMPin  46
#define LMotAPin 35
#define LMotBPin 37
#define RMotAPin 34
#define RMotBPin 36
int LMotor = 0;
int RMotor = 1;

#define LEncoderA 20
#define LEncoderB 21
#define REncoderA 2
#define REncoderB 3

/////////////////////////////////////////
ros :: NodeHandle nh;

geometry_msgs::Twist odom_msg;
geometry_msgs::Twist debug_msg;

ros::Publisher Pub("ard_odom",&odom_msg);
ros::Publisher debugger("debug",&debug_msg);

int TPR= 3600; //ticks per rotation

long EncoderVal[2] = {0,0};
double DDis[2] = {0,0};
long Time[2] = {0,0};

double Vels[2] = {0,0};

double WheelSeparation=3.60;
double WheelDiameter=1.06;

Encoder LEncoder(LEncoderA, LEncoderB);
Encoder REncoder(REncoderA, REncoderB);

int inApin[2] = {LMotAPin, RMotAPin};  // INA: Clockwise input
int inBpin[2] = {LMotBPin, RMotBPin}; // INB: Counter-clockwise input
int pwmpin[2] = {LMotPWMPin, RMotPWMPin}; // PWM input
int MotorNum[2] = {LMotor, RMotor};

void wheelmotion(int pwm, int motor){
    if(pwm<0)
    {
	digitalWrite(inApin[motor], LOW);
    	digitalWrite(inBpin[motor], HIGH);
     	analogWrite(pwmpin[motor], 255+pwm);
    }
    else
    {
	digitalWrite(inApin[motor], HIGH);
    	digitalWrite(inBpin[motor], LOW);
    	analogWrite(pwmpin[motor], 255-pwm);
    }
}

void motor(double left_vel, double right_vel){
        double left_pwm,right_pwm;
        left_pwm=-left_vel*255/MaxSpeed;
        right_pwm=right_vel*255/MaxSpeed;
        wheelmotion(int(left_pwm), LMotor);
 	wheelmotion(int(right_pwm), RMotor);
}

void messageCb( const geometry_msgs::Twist& CVel){
   
    double vel_x = CVel.linear.x;
    double vel_th = CVel.angular.z;
    double right_vel = 0.0;
    double left_vel = 0.0;
    // turning
    if(vel_x == 0){  
        right_vel = vel_th * WheelSeparation / 2.0;
        left_vel = (-1) * right_vel;
    }
    // forward / backward
    else if(vel_th == 0){ 
        left_vel = right_vel = vel_x;
    }
    // moving doing arcs
    else{ 
        left_vel = vel_x - vel_th * WheelSeparation / 2.0;
        right_vel = vel_x + vel_th * WheelSeparation / 2.0;
    }
    //for debugging
    debug_msg.linear.x=left_vel;
    debug_msg.linear.y=right_vel;
    debugger.publish(&debug_msg);
    //drive motors
    motor(left_vel,right_vel);
	delay(500);   //for step movement of robot
    motor(0.0,0.0);
}

ros::Subscriber<geometry_msgs::Twist> Sub("cmd_vel", &messageCb );

double TicksToMeters(long Ticks){
	return (Ticks*3.14*WheelDiameter)/TPR;
}

void setup() {
    pinMode(34,OUTPUT);
        pinMode(35,OUTPUT);
    pinMode(36,OUTPUT);
    pinMode(37,OUTPUT);
    pinMode(45,OUTPUT);
    pinMode(46,OUTPUT);

    nh.initNode();	
    nh.advertise(Pub);
    nh.advertise(debugger);
    nh.subscribe(Sub);
    //Serial.begin(9600);
    //Serial.println("Basic Encoder Test:");
}

double read_encoder(int M){
//if fist time in program return 0 and init time vars
	if(Time[0]==0 && Time[1] == 0){
		Time[0] = millis();
		Time[1] = millis();
		return 0;
	}
	//read encoder ticks
	if(M == LMotor){
		EncoderVal[0] = LEncoder.read();
		LEncoder.write(0);
	}
	if(M == RMotor){
		EncoderVal[1] = REncoder.read();
		REncoder.write(0);
	}
	//differencial of time in seconds
	long T = millis();
	int DTime = T-Time[M];
	Time[M] = T;
	//diferential of distance in meters
	DDis[M] = TicksToMeters(EncoderVal[M]);
	//calculate short term measured velocity
	double EVel = (DDis[M]/DTime)*1000;
   return EVel;
}

void loop() {
    //start nodes
    nh.spinOnce(); 
  
 //preparing odom message
    odom_msg.linear.x=read_encoder(LMotor);
    odom_msg.linear.y=read_encoder(RMotor);
    Pub.publish(&odom_msg);
    delay(300); //position update every 0.3 s
}
