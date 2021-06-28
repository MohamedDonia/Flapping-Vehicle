//Arduino servo flap system
//copyright Steve Morris 10-25-16
#include <Servo.h>
volatile int pwm_value = 0;
volatile int prev_time = 0;
static int servo_comm1 = 0;
static int servo_comm2 = 0;
volatile int flapangle1 = 0;
volatile int flapangle2 = 0;
volatile int millisold = 0;
int millinow= 0;
float dt=0;
float flapmag = 0;
static float flapdeg = 0;
float tcommand = 0;
float floattime=0;
float temp=0;
float glide_deg = -10.0;
static int pwm_value_temp = 0;
float omegadot=0.0;
float thetadot=0.0;
static float omega=0.0;
static float theta=0.0;
static float k0=1.0;
static float k2=10.0;
static float servo_zero1=-4;
static float servo_zero2=0;

Servo myservo1, myservo2; // create servo object to control a servo

void setup() {
//Serial.begin(115200);
// when pin D2 goes high, call the rising function
attachInterrupt(0, rising, RISING);
myservo1.attach(5); // attaches the servo on pin 5 to the servo object
myservo2.attach(6); // attaches the servo on pin 6 to the servo object
}
void loop() {

millinow=millis();
floattime=millinow/1000.0;
dt=(millinow-millisold)/1000.0;
millisold=millinow;

tcommand=(pwm_value-480.0)/8.22;
omegadot=k0*tcommand-k2*omega;
thetadot=omega;
flapdeg=sin(theta);
theta=theta+omega*dt;
omega=omega+omegadot*dt;

flapmag=(pwm_value-880)/41.0+10;
flapdeg=flapmag*sin(theta);//variable amplitude+freq


flapangle1=(int)((30.0-flapdeg+servo_zero1)*2.0+25.0);
flapangle2=(int)((30.0+flapdeg+servo_zero2)*2.0+25 .0);

if (pwm_value > 930){
servo_comm1 = flapangle1;
servo_comm2 = flapangle2;
}

//Glide Lock
if (pwm_value < 910){

servo_comm1 = (int)((30.0-glide_deg+servo_zero1)*2.0+25.0);
servo_comm2 = (int)((30.0+glide_deg+servo_zero2)*2.0+25.0);

}

myservo1.write(servo_comm1); // tell servo to go to position in variable 'pos'
myservo2.write(servo_comm2); // tell servo to go to position in variable 'pos'

//Serial.println(servo_comm1);
//Serial.println(floattime);
//Serial.println(dt);
//Serial.println(flapangle);
//Serial.println(flapdeg);
//Serial.println(temp);
//Serial.println(tcommand);
//Serial.println(pwm_value);

}

void rising() {
attachInterrupt(0, falling, FALLING);
prev_time = micros(); 
}

void falling() {
attachInterrupt(0, rising, RISING);
pwm_value = micros()-prev_time;
}