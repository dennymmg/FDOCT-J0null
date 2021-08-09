// Date: 2nd July 2021

// Protocol for vibration detection
// Camera runs in non-triggered mode
// Speaker always on
// Piezo always on with amplitude V (such that m_bias is between 0 and 2.303), say V = 4.0
// step1: 5 frames are taken -- bscans computed and averaged to X1
// step2: Program sends a char 'P' (phase shift signal) to the Arduino
// step3: Arduino receives P and sends HIGH through pin10 (Port B2)
// step4: High on Pin 10 causes the phase shift of 180
// step5: the program discards the next frame from the camera
// step6: 5 frames are taken -- bscans computed and averaged to X2
// step7: displays X2 - X1
// step8: go back to step1

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    while (!Serial) 
    {
      ; // wait for serial port to connect. Needed for native USB port only
    }
    pinMode(8, OUTPUT);   
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);   
    pinMode(11, OUTPUT);

        
}// end of setup function

// Global variables
char ch;
bool toggle = true;

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available() < 0)
  {
    ; // do nothing
  }
  ch = Serial.read();

  if (ch == 'P')
  {
     if (toggle == true)     
     {
        // PORT B = B0 - pin8, B1 - pin9, B2 - pin 10, ..., B4 - pin12
        PORTB = B00000011; // pins 8 and 9 set to 1, pins 10 and 11 reset to 0
        toggle = false;
     }
     else
     {
        PORTB = B00001100; // pins 8 and 9 reset to 0, pins 10 and 11 set to 1
        toggle = true;
     }
      
  }
          
}
