// Include the ESP32 Arduino Servo Library instead of the original Arduino Servo Library
#include <ESP32Servo.h> 

Servo servomotor;  // Servomotor
Servo linealmotor_1;  // Linearmotor 1
Servo linealmotor_2;  // Linearmotor 2
Servo linealmotor_3;  // Linearmotor 3

const int error_tol = 8;
const int nrows = 7;

int Positions[7][4] = {{90,150,100,100},  // Creation of matrix of positions for the Automatic Mode
                       {90,30,60,30},
                       {90,100,150,100},
                       {90,30,60,30},
                       {90,90,140,90},
                       {120,30,60,30},
                       {70,120,80,120}};

int a=0;      // variable used as index for the matrix of positions to reference rows inside it.

const int buttonPin = 25;     // The number of the pushbutton pin
 
// PWM output pins to control the actuators
int Rotary_Z = 18;      // GPIO pin used to control the Servo control (digital out)
int Lineal_M1 = 19;     // GPIO pin used to control the Linear Motor 1 (digital out)
int Lineal_M2 = 22;     // GPIO pin used to control the Linear Motor 2 (digital out)
int Lineal_M3 = 23;     // GPIO pin used to control the Linear Motor 3 (digital out)

// Analog Inputs for reading the Potentiometers in Manual Mode
int Pot_R = 32;        // GPIO pin used to connect the potentiometer for Servomotor (analog in)
int Pot_L1 = 33;       // GPIO pin used to connect the potentiometer for Linear Motor 1(analog in)
int Pot_L2 = 34;       // GPIO pin used to connect the potentiometer for Linear Motor 2(analog in)
int Pot_L3 = 35;       // GPIO pin used to connect the potentiometer for Linear Motor 3(analog in)

// Analog Inputs for reading the Feedback signal in Automatic Mode
int Fb_L1 = 13;        // GPIO pin used to connect the Linear Motor 1 Feedback signal (analog in)
int Fb_L2 = 12;        // GPIO pin used to connect the Linear Motor 2 Feedback signal (analog in)
int Fb_L3 = 14;        // GPIO pin used to connect the Linear Motor 3 Feedback signal (analog in)


int ADC_Max = 4096;     // Set the ADC resolution

// Auxiliary variables to read analog pins
int val_1;    // variable to read the value from the analog pin of the servomotor
int val_2;    // variable to read the value from the analog pin of the Linear Motor 1
int val_3;    // variable to read the value from the analog pin of the Linear Motor 2
int val_4;    // variable to read the value from the analog pin of the Linear Motor 3

int fb_1;     // variable to read the value from the analog pin of Feedback signal of the Linear Motor 1
int fb_2;     // variable to read the value from the analog pin of Feedback signal of the Linear Motor 2
int fb_3;     // variable to read the value from the analog pin of Feedback signal of the Linear Motor 3


// Variable to switch between work modes
int buttonState = 0;         

///////////////////////////////////////////////////////////////////////////////////////
// Interruption program: change the state of the "buttonState", therefore work-mode  //
///////////////////////////////////////////////////////////////////////////////////////
void IRAM_ATTR Ext_INT1_ISR()     
{
  delay(100);
  buttonState = !buttonState;         // Logical change of state of buttonState
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Automatic Mode Program: reads matrix of stored positions, execute them in sequence and in loop //
////////////////////////////////////////////////////////////////////////////////////////////////////
void automatic_mode()
{
    fb_1 = analogRead(Fb_L1);                 // Reads and stores Feedback signal from Linear Motor 1 in "fb_1"
    fb_1 = map(fb_1, 0, ADC_Max, 180, 0);     // Linear transformation of the feedback into [180, 0] range
    fb_2 = analogRead(Fb_L2);                 // Reads and stores Feedback signal from Linear Motor 2 in "fb_2"
    fb_2 = map(fb_2, 0, ADC_Max, 180, 0);     // Linear transformation of the feedback into [180, 0] range
    fb_3 = analogRead(Fb_L3);                 // Reads and stores Feedback signal from Linear Motor 3 in "fb_3"
    fb_3 = map(fb_3, 0, ADC_Max, 180, 0);     // Linear transformation of the feedback into [180, 0] range

    // The value of "a" is initially 0, so it would start with the position corresponding to the first row
    
    if (a==0){
      // With the ESP32 servo library a value between 0 - 180 can be sent directly to a servomotor
      servomotor.write(Positions[a][0]);      // Send first value of the first row (a=0) to the servomotor as PWM
      linealmotor_1.write(Positions[a][1]);   // Send second value of the first row (a=0) to the Lineal Motor 1 as PWM
      linealmotor_2.write(Positions[a][2]);   // Send third value of the first row (a=0) to the Lineal Motor 2 as PWM
      linealmotor_3.write(Positions[a][3]);   // Send fourth value of the first row (a=0) to the Lineal Motor 3 as PWM
      a++;                                    // Increase the variable of index of Matrix in 1
    }

    // Diference between the set point (position values in the matrix) and the real position (Feedback signal) must be
    // less than an error tolerance value, so it can go to the next position (next row of the matrix)
    
    if ((abs(Positions[a-1][1]-fb_1)<error_tol)&&(abs(Positions[a-1][2]-fb_2)<error_tol)&&(abs(Positions[a-1][3]-fb_3)<error_tol))
    {
      if (a==nrows)   // If the index variable gets to the last row of the Matrix, it has to be reset to the first row
      {
        a=0;
      }
      else            // If not, then the position "a" of the matrix is output for all actuators
      {
        servomotor.write(Positions[a][0]);      // Send first value of the "a" row to the servomotor as PWM
        linealmotor_1.write(Positions[a][1]);   // Send second value of the "a" row to the servomotor as PWM
        linealmotor_2.write(Positions[a][2]);   // Send third value of the "a" row to the servomotor as PWM
        linealmotor_3.write(Positions[a][3]);   // Send fourth value of the "a" row to the servomotor as PWM
        a++;                                    // Increase the variable of index of Matrix in 1
      }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Manual Mode Program: reads potentiometers, converts their value into a motor output in a proportional way //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void manual_mode()
{
  val_1 = analogRead(Pot_R);                  // Read the value of the potentiometer (value between 0 and 4095)
  val_2 = analogRead(Pot_L1);                 // Read the value of the potentiometer (value between 0 and 4095)
  val_3 = analogRead(Pot_L2);                 // Read the value of the potentiometer (value between 0 and 4095)
  val_4 = analogRead(Pot_L3);                 // Read the value of the potentiometer (value between 0 and 4095)
  val_1 = map(val_1, 0, ADC_Max, 0, 180);     // Scale it to use it with the servo (value between 0 and 180)
  val_2 = map(val_2, 0, ADC_Max, 0, 180);     // Scale it to use it with the servo (value between 0 and 180)
  val_3 = map(val_3, 0, ADC_Max, 0, 180);     // Scale it to use it with the servo (value between 0 and 180)
  val_4 = map(val_4, 0, ADC_Max, 0, 180);     // Scale it to use it with the servo (value between 0 and 180)
  servomotor.write(val_1);                    // Set the servo position according to the scaled value
  linealmotor_1.write(val_2);                 // Set the servo position according to the scaled value
  linealmotor_2.write(val_3);                 // Set the servo position according to the scaled value
  linealmotor_3.write(val_4);                 // Set the servo position according to the scaled value

  a=0;                                        // Reset the index variable, so the next time it goes to automatic mode starts in the first position

  // TESTING: Sending the feedback value and potenciometer value to serial for testing purposes
  fb_1 = analogRead(Fb_L1);                 // Reads and stores Feedback signal from Linear Motor 1 in "fb_1"
  fb_1 = map(fb_1, 0, ADC_Max, 180, 0);     // Linear transformation of the feedback into [180, 0] range
  fb_2 = analogRead(Fb_L2);                 // Reads and stores Feedback signal from Linear Motor 2 in "fb_2"
  fb_2 = map(fb_2, 0, ADC_Max, 180, 0);     // Linear transformation of the feedback into [180, 0] range
  fb_3 = analogRead(Fb_L3);                 // Reads and stores Feedback signal from Linear Motor 3 in "fb_3"
  fb_3 = map(fb_3, 0, ADC_Max, 180, 0);     // Linear transformation of the feedback into [180, 0] range

  // Print values from potentiometer and feedback values
  Serial.print("Pot:");
  Serial.print(val_1);
  Serial.print(" ");
  
  Serial.print("LM_1:");
  Serial.print(val_2);
  Serial.print(" ");
  Serial.print("Fb_1:");
  Serial.print(fb_1);
  Serial.print(" ");
  
  Serial.print("LM_2:");
  Serial.print(val_3);
  Serial.print(" ");
  Serial.print("Fb_2:");
  Serial.print(fb_2);
  Serial.print(" ");

  Serial.print("LM_3:");
  Serial.print(val_4);
  Serial.print(" ");
  Serial.print("Fb_3:");
  Serial.println(fb_3);
}

////////////////////////////////////////////////////////////////////////////////////////
// Setup: variables, classes, serial comunication and interruptions are initialized   //
////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  pinMode(buttonPin, INPUT_PULLUP);                 //The pin for the button is declared as input with a pullup resistance
  
  servomotor.setPeriodHertz(50);                    // Standard 50hz servo
  linealmotor_1.setPeriodHertz(50);                 // Standard 50hz servo
  linealmotor_2.setPeriodHertz(50);                 // Standard 50hz servo
  linealmotor_3.setPeriodHertz(50);                 // Standard 50hz servo
  
  servomotor.attach(Rotary_Z, 500, 2400);           // attaches the servo on pin 18 to the servo object
                                                    // using SG90 servo min/max of 500us and 2400us
                                                    // for MG995 large servo, use 1000us and 2000us,
                                                    // which are the defaults, so this line could be
                                                    // "myservo.attach(servoPin);"
  linealmotor_1.attach(Lineal_M1, 1000, 2000);
  linealmotor_2.attach(Lineal_M2, 1000, 2000);
  linealmotor_3.attach(Lineal_M3, 1000, 2000);

  Serial.begin(9600);                               // Serial comunication is initiated

  attachInterrupt(buttonPin, Ext_INT1_ISR, RISING); // External interruptions are activated for the pin connected to the button
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Loop: A variable connected to the button is evaluated to alternate between manual and automatic mode   //
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  
    if (buttonState == HIGH) {
      automatic_mode();
    } else {
      manual_mode();
    }
    delay(50);                                      // wait for the servo to get there
}
