// Test MD03a / Pololu motor with encoder
// speed control (PI), V & I display
// Credits:
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923


#define InA1            7                      // INA motor pin
#define InB1            8                      // INB motor pin 
#define PWM1            5                       // PWM motor pin
#define encodPinA1      19                       // encoder A pin
#define encodPinB1      18                       // encoder B pin

#define InA2            4                      // INA motor pin
#define InB2            9                      // INB motor pin 
#define PWM2            6                       // PWM motor pin
#define encodPinA2      20                       // encoder A pin
#define encodPinB2      21                       // encoder B pin


#define LOOPTIME        100                     // PID loop time

unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing
int speed_req = 0;                            // speed (Set Point)
int speed_act1 = 0;                              // speed (actual value)
int speed_act2 = 0; 
int PWM_val1 = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val2 = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
float Distance1=0;
float Distance2=0;
volatile long count1 = 0;                        // rev counter
volatile long count2= 0;                        // rev counter2
float Kp =   0.4;                                // PID proportional control Gain
float Kd =    1;                                // PID Derivitave control gain


void setup() {
 Serial.begin(115600);
 pinMode(InA1, OUTPUT);
 pinMode(InB1, OUTPUT);
 pinMode(PWM1, OUTPUT);
 pinMode(encodPinA1, INPUT); 
 pinMode(encodPinB1, INPUT); 
 digitalWrite(encodPinA1, HIGH);                      // turn on pullup resistor
 digitalWrite(encodPinB1, HIGH);
 attachInterrupt(4, rencoder1, FALLING);

 pinMode(InA2, OUTPUT);
 pinMode(InB2, OUTPUT);
 pinMode(PWM2, OUTPUT);
 pinMode(encodPinA2, INPUT); 
 pinMode(encodPinB2, INPUT); 
 digitalWrite(encodPinA2, HIGH);                      // turn on pullup resistor
 digitalWrite(encodPinB2, HIGH);
 attachInterrupt(3, rencoder2, FALLING);

/*
 analogWrite(PWM1, PWM_val1);
 digitalWrite(InA1, LOW);
 digitalWrite(InB1, HIGH);

 analogWrite(PWM2, PWM_val2);
 digitalWrite(InA2, LOW);
 digitalWrite(InB2, HIGH);
 */
}



void loop() {
  
 //getParam1();                                                                 // check keyboard 

 
 
  if((millis()-lastMilli) >= LOOPTIME)   {     // enter tmed loop
  // Forward();
   lastMilli = millis();
   getMotorData1();                                                         // calculate speed, volts and Amps
   PWM_val1= updatePid(PWM_val1, speed_req, speed_act1);                        // compute PWM value  
   analogWrite(PWM1, PWM_val1);                                               // send PWM to motor
   getMotorData2();
   PWM_val2= updatePid2(PWM_val2, speed_req, speed_act2);
   analogWrite(PWM2, PWM_val2); 
   
 }
 
 printMotorInfo();   // display data
}



void Forward()
{
   //analogWrite(PWM1, PWM_val1);
 digitalWrite(InA1, LOW);
 digitalWrite(InB1, HIGH);

 //analogWrite(PWM2, PWM_val2);
 digitalWrite(InA2, LOW);
 digitalWrite(InB2, HIGH);
}

void reverse()
{
  // analogWrite(PWM1, PWM_val1);
 digitalWrite(InA1, HIGH);
 digitalWrite(InB1, LOW);

// analogWrite(PWM2, PWM_val2);
 digitalWrite(InA2, HIGH);
 digitalWrite(InB2, LOW);
}


void getMotorData1()  {                                                        // calculate speed, volts and Amps
static long countAnt1 = 0;                                                   // last count
 speed_act1 = ((count1 - countAnt1)*(60*(1000/LOOPTIME)))/(420);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
 Distance1= abs((count1 * 0.66)/420);
 countAnt1 = count1;                  
}

void getMotorData2()  {                                                        // calculate speed, volts and Amps
static long countAnt2 = 0;                                                   // last count
 speed_act2 = ((count2 - countAnt2)*(60*(1000/LOOPTIME)))/(420);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
  Distance2= abs((count2 * 0.66)/420);
 countAnt2 = count2;                  
 
}

int updatePid(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error=0;                                  
static int last_error=0;                             
 error = abs(targetValue) - abs(currentValue); 
 pidTerm = (Kp * error) + (Kd * (error - last_error));                            
 last_error = error;
 return constrain(command + int(pidTerm), 0, 255);
}

int updatePid2(int command2, int targetValue2, int currentValue2)   {             // compute PWM value
float pidTerm2 = 0;                                                            // PID correction
int error2=0;                                  
static int last_error2=0;                             
 error2 = abs(targetValue2) - abs(currentValue2); 
 pidTerm2 = (Kp * error2) + (Kd * (error2 - last_error2));                            
 last_error2 = error2;
 return constrain(command2 + int(pidTerm2), 0, 255);
}

void printMotorInfo()  {                                                      // display data
 if((millis()-lastMilliPrint) >= 500)   {                     
   lastMilliPrint = millis();
   Serial.print("SP:");             Serial.print(speed_req);  
   Serial.print("  RPM:");          Serial.print(speed_act1);
   Serial.print("  PWM:");          Serial.print(PWM_val1);  

   Serial.print("  RPM2:");          Serial.print(speed_act2);
   Serial.print("  PWM2:");          Serial.print(PWM_val2);
  Serial.print("  Distance1:");          Serial.print(Distance1);
   Serial.print("  Distance2:");          Serial.print(Distance2); 
  // if (current > CURRENT_LIMIT)               Serial.println("*** CURRENT_LIMIT ***");                
  // if (voltage > 1000 && voltage < LOW_BAT)   Serial.println("*** LOW_BAT ***");                
 }
}

void rencoder1()  {                                    // pulse and direction, direct port reading to save cycles
if(digitalRead(encodPinB1)==HIGH)   count1 ++;
 else   count1 --;
}

void rencoder2()  {                                    // pulse and direction, direct port reading to save cycles
 if(digitalRead(encodPinB2)==HIGH)   count2 ++;
 else    count2 --;
}

int getParam1()  {
char param, cmd;
 if(!Serial.available())    return 0;
 delay(10);                  
 param = Serial.read();                              // get parameter byte
 if(!Serial.available())    return 0;
 cmd = Serial.read();                                // get command byte
 Serial.flush();
 switch (param) {
   case 'v':                                         // adjust speed
     if(cmd=='+')  {
       speed_req += 20;
       if(speed_req>400)   speed_req=400;
     }
     if(cmd=='-')    {
       speed_req -= 20;
       if(speed_req<0)   speed_req=0;
     }
     break;
   case 'm':                                        // adjust direction
     if(cmd=='+'){
       digitalWrite(InA1, LOW);
       digitalWrite(InB1, HIGH);
       digitalWrite(InA2,LOW);
       digitalWrite(InB2,HIGH);
     }
     if(cmd=='-')   {
       digitalWrite(InA1, HIGH);
       digitalWrite(InB1, LOW);
       digitalWrite(InA2, HIGH);
       digitalWrite(InB2, LOW);
     }
          if(cmd=='/')   {
       digitalWrite(InA1, LOW);
       digitalWrite(InB1, HIGH);
       digitalWrite(InA2, HIGH);
       digitalWrite(InB2, LOW);
     }
          if(cmd=='*')   {
       digitalWrite(InA1, HIGH);
       digitalWrite(InB1, LOW);
       digitalWrite(InA2, LOW);
       digitalWrite(InB2, HIGH);
     }
     break;
   case 'o':                                        // user should type "oo"
     digitalWrite(InA1, LOW);
     digitalWrite(InB1, LOW);
         digitalWrite(InA2, LOW);
     digitalWrite(InB2, LOW);
     speed_req = 0;
     break;
   default: 
     Serial.println("???");
   }
}

