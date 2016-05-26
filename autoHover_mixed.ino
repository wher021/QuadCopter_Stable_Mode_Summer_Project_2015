#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro.
#include <Servo.h>
#include <Math.h>

float rotationThreshold = 1;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;//1.3;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.05;//0.05              //Gain setting for the roll I-controller (0.3)
float pid_d_gain_roll = 15;//15;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle = 1510;
Servo m1, m2, m3, m4;

int cal_int;
unsigned long loop_timer = 0;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte highByte, lowByte;

float pid_error_temp = 0;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error = 0;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error = 0;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error = 0;

//Start Variable//
unsigned int start = 0;

//RC Controls
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
//============================================
float angle_roll = 0;
float angle_pitch = 0;
int setmaxangle = 30;

int cal_acc_int = 0;

//Accelerometer//
float Accx;
float Accy;
float Accz;

float Acc_x_cal;
float Acc_y_cal;
float Acc_z_cal;

float rollAcc, pitchAcc;
double compAngleX, compAngleY, dt;
unsigned long inner_loop_timer = 0;

unsigned long outer_loop_timer = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  PORTB &= B00000000;
  Serial.begin(57600);
  Serial3.begin(57600);
  Wire.begin();                                                //Start the I2C as master.

  //DDRD |= B11110000;                                         //Configure digital poort 4, 5, 6 and 7 as output.

  //Configure digital poort 2, 3, 4 and 5 as output.
  DDRE |= B00111000;//PWM 2,3, 5 = PE3 OUTPUT
  DDRG |= B00100000;//PWM 4 OUTPUT

  DDRB |= B10000000;

  //DDRB |= B00110000;   //Configure digital poort 12 and 13 as output.
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.

  //Use the led on the Arduino for startup indication
  //digitalWrite(12, HIGH);                                      //Turn on the warning led.
  PORTB |= B10000000;

  Wire.beginTransmission(0x6B);                                 //Start communication with the gyro (adress 1101001)
  Wire.write(0x20);                                            //We want to write to register 1 (20 hex)
  Wire.write(0x0F);                                            //Set the register bits as 00001111 (Turn on the gyro and enable all axis)
  Wire.endTransmission();                                      //End the transmission with the gyro

  Wire.beginTransmission(0x6B); //105                           //Start communication with the gyro (adress 1101001)
  Wire.write(0x23);                                            //We want to write to register 4 (23 hex)
  Wire.write(0x90);                                            //Set the register bits as 10010000 (Block Data Update active & 500dps full scale)
  Wire.endTransmission();                                      //End the transmission with the gyro

  delay(250);                                                  //Give the gyro time to start.

  Serial.println("initdone Calibrating...");

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 2000 ; cal_int ++) {             //Take 2000 readings for calibration.
    if (cal_int % 15 == 0)digitalWrite(12, !digitalRead(12));  //Change the led status to indicate calibration.
    gyro_signalen();                                           //Read the gyro output.
    gyro_roll_cal += gyro_roll;                                //Ad roll value to gyro_roll_cal.
    gyro_pitch_cal += gyro_pitch;                              //Ad pitch value to gyro_pitch_cal.
    gyro_yaw_cal += gyro_yaw;                                  //Ad yaw value to gyro_yaw_cal.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    //PORTD |= B11110000;      //Set digital poort 4, 5, 6 and 7 high.

    PORTE |= B00111000;//PWM 2,3, 5 = PE3 OUTPUT
    PORTG |= B00100000;//PWM 4 OUTPUT
    delayMicroseconds(1000);                                   //Wait 1000us.
    //PORTD &= B00001111;                                        //Set digital poort 4, 5, 6 and 7 low.
    PORTE &= B11000111;//PWM 2,3, 5 = PE3 OUTPUT
    PORTG &= B11011111;//PWM 4 OUTPUT
    delay(3);                                                  //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_roll_cal /= 2000;                                       //Divide the roll total by 2000.
  gyro_pitch_cal /= 2000;                                      //Divide the pitch total by 2000.
  gyro_yaw_cal /= 2000;                                        //Divide the yaw total by 2000.

  init_Compass();
  /*
  CalibrateAcc();

  Serial.print(Acc_x_cal);
  Serial.print("  ");
  Serial.println(Acc_y_cal);
  */

  //When everything is done, turn off the led.
  //digitalWrite(12, LOW);                                       //Turn off the warning led.
  PORTB &= B00000000;
  Serial.println("Wait for receiver!");

  PCICR |= (1 << PCIE0);                                       //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                     //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                     //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                     //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                     //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

  //Wait until the receiver is active and the throtle is set to the lower position.

  while (receiver_input_channel_3 < 1190 || receiver_input_channel_3 > 1270) {
    Serial.println("Waiting For throttle");
  }

  pid_roll_setpoint = 0;//gyro
  pid_pitch_setpoint = 0;

  m1.attach(3);
  m3.attach(2);
  m2.attach(5);
  m4.attach(4);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  //Let's get the current gyro data and scale it to degrees per second for the pid calculations.
  gyro_signalen();
  gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll / 57.14286) * 0.2);            //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch / 57.14286) * 0.2);         //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_yaw / 57.14286) * 0.2);               //Gyro pid input is deg/sec.

  if (gyro_roll_input >= rotationThreshold || gyro_roll_input <= -rotationThreshold)
  {
    angle_roll += (gyro_roll_input * dt);//Gyro Angle
  }
  if (gyro_pitch_input >= rotationThreshold || gyro_pitch_input <= -rotationThreshold)
  {
    angle_pitch += (gyro_pitch_input * dt);//Gyro Angle
    
  }

  /*
    Serial.print(angle_roll);
    Serial.print("  ");
    Serial.println(angle_pitch);
  */

  //Get Angles and combine Acc with Gyro, Complimentary Filter
  get_Accelerometer(&angle_roll, &angle_pitch);

  //Here are the inputs for the Rate PID
  //compAngleX_roll_input = compAngleX;//PID AUTOX input

  //compAngleY_pitch_input = compAngleY;//PID AUTOX input

  //For starting the motors: throttle low and yaw left (step 1).
  if (receiver_input_channel_3 < 1270 && receiver_input_channel_4 < 1050)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if (start == 1 && receiver_input_channel_3 < 1270 && receiver_input_channel_4 > 1450)
  {
    start = 2;
    //Reset the pid controllers for a bumpless start.
    ResetPID();
  }
  //Stopping the motors: throttle low and yaw right.
  if (start == 2 && receiver_input_channel_3 < 1270 && receiver_input_channel_4 > 1950)
  {
    start = 0;
    ResetPID();
    /*
    Wire.beginTransmission(0x6B);//we want to write to fyro
    Wire.write(0x39);//address
    Wire.write(0x04); //reset
    Wire.endTransmission();
    */
  }


  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;

  //We need a little dead band of 16us for better results.

  if (receiver_input_channel_3 > 1250) { //Do not yaw when turning off the motors.
    if (receiver_input_channel_4 > 1510)pid_yaw_setpoint = (receiver_input_channel_4 - 1510) / 3.0;
    else if (receiver_input_channel_4 < 1490)pid_yaw_setpoint = (receiver_input_channel_4 - 1490) / 3.0;
  }

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).

  pid_roll_setpoint = -compAngleX;

  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_1 > 1510 && compAngleX < setmaxangle)pid_roll_setpoint = (receiver_input_channel_1 - 1510) / 12.0;
  else if (receiver_input_channel_1 < 1490 && compAngleX > -setmaxangle)pid_roll_setpoint = (receiver_input_channel_1 - 1490) / 12.0;


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).

  pid_pitch_setpoint = -compAngleY;

  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_2 > 1510 && compAngleY < setmaxangle)pid_pitch_setpoint = (receiver_input_channel_2 - 1510) / 12.0;
  else if (receiver_input_channel_2 < 1490 && compAngleY > -setmaxangle)pid_pitch_setpoint = (receiver_input_channel_2 - 1490) / 12.0;

  checkSerial();

  calculate_pid();

  throttle = receiver_input_channel_3;//We need the throttle signal as a base signal.

  if (start == 2)
  {

    MotorControl(throttle, pid_output_roll, pid_output_pitch, pid_output_yaw);

    //String myGyro = "Pitch=" + String(gyro_pitch_input) + "  " + "Roll" + String(gyro_roll_input) + "  " + "YAW=" + String(gyro_yaw_input);
    //Serial.println(myGyro);

    if (esc_1 < 1200) esc_1 = 1200;                                         //Keep the motors running.
    if (esc_2 < 1200) esc_2 = 1200;                                         //Keep the motors running.
    if (esc_3 < 1200) esc_3 = 1200;                                         //Keep the motors running.
    if (esc_4 < 1200) esc_4 = 1200;                                         //Keep the motors running.

    if (esc_1 > 2200)esc_1 = 2200;                                          //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2200)esc_2 = 2200;                                          //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2200)esc_3 = 2200;                                          //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2200)esc_4 = 2200;                                          //Limit the esc-4 pulse to 2000us.
  }
  else
  {
    throttle = 1000;
    MotorControl(throttle, 0, 0, 0);
  }


  while (micros() - inner_loop_timer < 4000);//We wait until 4000us are passed. 250HZ
  dt = (double)(micros() - inner_loop_timer) / 1000000;
  //Serial.println(dt * 1000000);
  inner_loop_timer = micros();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This routine is called every time input 8, 9, 10 or 11 changed state
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect) {
  current_time = micros();
  //Channel 1=========================================
  if (PINB & B00000001) {                                      //Is input 8 high?
    if (last_channel_1 == 0) {                                 //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if (last_channel_1 == 1) {                              //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input_channel_1 = current_time - timer_1 + 8;//ROLL         //Channel 1 is current_time - timer_1
  }
  //Channel 2=========================================
  if (PINB & B00000010 ) {                                     //Is input 9 high?
    if (last_channel_2 == 0) {                                 //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if (last_channel_2 == 1) {                              //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input_channel_2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
  }
  //Channel 3=========================================
  if (PINB & B00000100 ) {                                     //Is input 10 high?
    if (last_channel_3 == 0) {                                 //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if (last_channel_3 == 1) {                              //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input_channel_3 = current_time - timer_3 + 100;//Throttle        //Channel 3 is current_time - timer_3
  }
  //Channel 4=========================================
  if (PINB & B00001000 ) {                                     //Is input 11 high?
    if (last_channel_4 == 0) {                                 //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if (last_channel_4 == 1) {                              //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input_channel_4 = current_time - timer_4 - 24;//YAW         //Channel 4 is current_time - timer_4
  }
}


void MotorControl(int throttle, float pid_output_roll, float pid_output_pitch, float pid_output_yaw)
{
  //The motors are started.
  if (throttle > 1800) throttle = 1800;//We need some room to keep full control at full throttle.
  //RC COnfiguration//

  //Serial.println(pid_output_roll);


  esc_1 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;
  esc_2 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;
  esc_3 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;
  esc_4 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;

  /*
    esc_1 = throttle + pid_output_roll;
    esc_2 = throttle + pid_output_roll;
    esc_3 = throttle - pid_output_roll;
    esc_4 = throttle - pid_output_roll;
  */

  /*
    esc_1 = throttle + pid_output_pitch;
    esc_2 = throttle - pid_output_pitch;
    esc_3 = throttle - pid_output_pitch;
    esc_4 = throttle + pid_output_pitch;
  */

  /*
    esc_1 = throttle - pid_output_yaw;
    esc_2 = throttle + pid_output_yaw;
    esc_3 = throttle - pid_output_yaw;
    esc_4 = throttle + pid_output_yaw;
  */


  m1.writeMicroseconds(esc_4);//FR
  m3.writeMicroseconds(esc_3);//BR
  m2.writeMicroseconds(esc_1);//FL
  m4.writeMicroseconds(esc_2);//BL

  //Viktigt!
  //String MotorOut = "FR=" + String(esc_4) + "  " + "BR=" + String(esc_3) + "  " + "BL=" + String(esc_2) + "  " + "FL=" + String(esc_1);
  //Serial.println(MotorOut);
}

void checkSerial()
{
  if (Serial.available()) {
    byte inpt = Serial.read();
    if (inpt == 'q') { // P -- NS
      pid_p_gain_roll += 0.01;

      printPIDGains();
    }
    else if (inpt == 'a') {
      pid_p_gain_roll -= 0.01;
      if (pid_p_gain_roll < 0)
        pid_p_gain_roll = 0;

      printPIDGains();
    }
    else if (inpt == 'w') { // I
      pid_i_gain_roll += 0.001;

      printPIDGains();
    }
    else if (inpt == 's') {
      pid_i_gain_roll -= 0.001;
      if (pid_i_gain_roll < 0)
        pid_i_gain_roll = 0;

      printPIDGains();
    }
    else if (inpt == 'e') { // D
      pid_d_gain_roll += 0.01;

      printPIDGains();
    }
    else if (inpt == 'd') {
      pid_d_gain_roll -= 0.01;
      if (pid_d_gain_roll < 0)
        pid_d_gain_roll = 0;

      printPIDGains();
    }
    else if (inpt == 'x')
    {
      if (throttle > 2000)
        throttle = 2000;
      throttle += 10;
      Serial.println(throttle);
    }
    else if (inpt == 'z')
    {
      throttle -= 10;
      Serial.println(throttle);
    }
    else if (inpt == 'u')
    {

      pid_p_gain_roll += 0.01;

      printPIDGains();
    }
    else if (inpt == 'j')
    {
      pid_p_gain_roll -= 0.01;
      if (pid_p_gain_roll < 0)
        pid_p_gain_roll = 0;

      printPIDGains();
    }
    else if (inpt == 'i')
    {
      pid_i_gain_roll += 0.001;

      printPIDGains();
    }
    else if (inpt == 'k')
    {
      pid_i_gain_roll -= 0.001;
      if (pid_i_gain_roll < 0)
        pid_i_gain_roll = 0;

      printPIDGains();
    }
    else if (inpt == 'o')
    {
      pid_d_gain_roll += 0.01;

      printPIDGains();
    }
    else if (inpt == 'l')
    {
      pid_d_gain_roll -= 0.01;
      if (pid_d_gain_roll < 0)
        pid_d_gain_roll = 0;

      printPIDGains();
    }
  }
}

void printPIDGains()
{
  Serial.print("P=");
  Serial.print(pid_p_gain_roll, 6);
  Serial.print("  ");
  Serial.print("I=");
  Serial.print(pid_i_gain_roll, 6);
  Serial.print("  ");
  Serial.print("D=");
  Serial.print(pid_d_gain_roll, 6);
  Serial.print("    ");
  Serial.print("rP=");
  Serial.print(pid_p_gain_roll, 6);
  Serial.print("  ");
  Serial.print("rI=");
  Serial.print(pid_i_gain_roll, 6);
  Serial.print("  ");
  Serial.print("rD=");
  Serial.println(pid_d_gain_roll, 6);
}

void ResetPID()
{
  pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_mem_yaw = 0;
  pid_last_yaw_d_error = 0;

  pid_roll_setpoint = 0;
  pid_pitch_setpoint = 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen() {
  Wire.beginTransmission(0x6B);                                 //Start communication with the gyro (adress 1101001)
  Wire.write(168);                                             //Start reading @ register 28h and auto increment with every read
  Wire.endTransmission();                                      //End the transmission
  Wire.requestFrom(0x6B, 6);                                    //Request 6 bytes from the gyro
  while (Wire.available() < 6);                                //Wait until the 6 bytes are received
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_roll = ((highByte << 8) | lowByte);                     //Multiply highByte by 256 (shift left by 8) and ad lowByte
  if (cal_int == 2000)gyro_roll -= gyro_roll_cal;              //Only compensate after the calibration
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_pitch = ((highByte << 8) | lowByte);                    //Multiply highByte by 256 (shift left by 8) and ad lowByte
  gyro_pitch *= -1;                                            //Invert axis
  if (cal_int == 2000)gyro_pitch -= gyro_pitch_cal;            //Only compensate after the calibration
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_yaw = ((highByte << 8) | lowByte);                      //Multiply highByte by 256 (shift left by 8) and ad lowByte
  gyro_yaw *= -1;                                              //Invert axis
  if (cal_int == 2000)gyro_yaw -= gyro_yaw_cal;                //Only compensate after the calibration
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//www.youtube.com/watch?v=JBvnB0279-Q

void calculate_pid() {
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;

}

void CalibrateAcc()
{
  for (cal_acc_int = 0; cal_acc_int < 2000 ; cal_acc_int ++) {             //Take 2000 readings for calibration.
    ReadAcc();                                  //Read the Acc output.
    Acc_x_cal += Accx;                                //Ad roll value to gyro_roll_cal.
    Acc_y_cal += Accy;                              //Ad pitch value to gyro_pitch_cal.
    Acc_z_cal += Accz;                                  //Ad yaw value to gyro_yaw_cal.

    delay(3);                                                  //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average Acc offset.
  Acc_x_cal /= 2000;                                       //Divide the roll total by 2000.
  Acc_y_cal /= 2000;                                      //Divide the pitch total by 2000.
  Acc_z_cal /= 2000;                                        //Divide the yaw total by 2000.
}

/*
Send register address and the byte value you want to write the accelerometer and
loads the destination register with the value you send
*/
void WriteAccRegister(byte data, byte regaddress)
{
  Wire.beginTransmission(0x19);   // Use accelerometer address for regs >=0x20
  Wire.write(regaddress);
  Wire.write(data);
  Wire.endTransmission();
}

void init_Compass(void)
{
  WriteAccRegister(0x77, 0x20);//400 Hz//Normal Mode
  WriteAccRegister(0x98, 0x23);//Blocked Update//+-4g acceloremeter//High Resolution Enable
  //WriteAccRegister(0x88, 0x23);//+-2g
}

void ReadAcc()
{
  Wire.beginTransmission(0x19);
  // assert the MSB of the address to get the accelerometer
  // to do slave-transmit subaddress updating.
  Wire.write(0x28 | 0x80);
  Wire.endTransmission();
  Wire.requestFrom(0x19, (byte)6);

  while (Wire.available() < 6) {
  }

  byte xla = Wire.read();
  byte xha = Wire.read();
  byte yla = Wire.read();
  byte yha = Wire.read();
  byte zla = Wire.read();
  byte zha = Wire.read();

  // combine high and low bytes
  // This no longer drops the lowest 4 bits of the readings from the DLH/DLM/DLHC, which are always 0
  // (12-bit resolution, left-aligned). The D has 16-bit resolution
  Accy = ((int16_t)(xha << 8 | xla));
  Accy -= -318.77;//Acc_y_cal;

  Accx = ((int16_t)(yha << 8 | yla));
  Accx -= 28.30;//Acc_x_cal;

  Accz = ((int16_t)(zha << 8 | zla));
  Accz -= 500;
}

/*
Readsthe X,Y,Z axis values from the accelerometer and sends the values to the
serial monitor.
*/
void get_Accelerometer(float *roll, float *pitch)
{
  ReadAcc();
  /*
  uint8_t xlo = Wire.read();
  uint8_t xhi = Wire.read();
  uint8_t ylo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t zhi = Wire.read();

  Accy = ((int16_t)(xlo | (xhi << 8)));
  Accx = (int16_t)(ylo | (yhi << 8));
  Accz = (int16_t)(zlo | (zhi << 8));
  */

  /*
    Serial.print(Accx);
    Serial.print("  ");
    Serial.print(Accy);
    Serial.print("  ");
    Serial.println(Accz);
  */
  //if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)

  int forceMagnitudeApprox = abs(Accx) + abs(Accy) + abs(Accz);
  if (forceMagnitudeApprox > 4096 && forceMagnitudeApprox < 32768)//4G
  {

    rollAcc = atan2f(Accx, Accz) * 180 / M_PI;//AccAngle

    //compAngleX = 0.90 * (compAngleX + *roll * dt) + 0.1 * rollAcc;//original//man vill ha mer av roll
    *roll = 0.9996 * *roll + 0.0004 * rollAcc;

    pitchAcc = (atan2f(Accy, Accz) * 180 / M_PI);//Adjustment

    //compAngleY = 0.90 * (compAngleY + *pitch * dt) + 0.1 * pitchAcc;//0.94,0.06
    *pitch = 0.9996 * *pitch + 0.0004 * pitchAcc;


    compAngleX = *roll;
    compAngleY = *pitch;

    /*
        Serial.print(*roll);
        Serial.print("  ");
        Serial.println(*pitch);
        */

    /*
        Serial.print(rollAcc);
        Serial.print("  ");
        Serial.println(pitchAcc);
    */
  }

}



