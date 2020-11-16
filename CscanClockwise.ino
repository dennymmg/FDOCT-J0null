  void setup() {
  
  Serial.begin(115200);
  while (!Serial) 
  {
    ; // wait for serial port to   connect. Needed for native USB port only
  }
  
  pinMode(12, OUTPUT);   // for camera trigger
  
  // use port B for stepper motor
  // pin 08 of Arduino for IN4 D 
  // pin 09 of Arduino for IN3 C 
  // pin 10 of Arduino for IN2 B 
  // pin 11 of Arduino for IN1 A              
  DDRB = DDRB | B00001111; // make pins 8-11 as outputs
  
} // end of setup function

// Global variables
char ch;
unsigned int dt = 50; // delay time in milliseconds
unsigned int stepnum = 1;

void loop() {
  
  while(Serial.available() < 0)
  {
    ; // do nothing
  }
    ch = Serial.read();

    if (ch == 'J')
    {
      Serial.print("j");
      digitalWrite(12, HIGH);     
    }
    
    if (ch == 'K')
    {
      Serial.print("k");
      digitalWrite(12, LOW);
    }

    if (ch == 'D')
    {
      // rotate the stepper motor by a single step in the clockwise direction
      // followed by the necessary delay
      switch(stepnum)
      {
        case 1: //  A - 0001
                PORTB = B0001;
                 break;
        case 2: // AB - 0011;
                PORTB = B0011;
                break;
        case 3: //  B - 0010
                PORTB = B0010;
                break;
        case 4: // BC - 0110;
                PORTB = B0110;
                break;
        case 5: //  C - 0100
                PORTB = B0100;
                break;
        case 6: // CD - 1100;
                PORTB = B1100;
                break;
        case 7: //  D - 1000
                PORTB = B1000;
                break;
        case 8: // DA - 1001;
                PORTB = B1001;
                break;
      }
      if(stepnum == 8)
      {
        stepnum = 0;
      }
      else
      {
        stepnum ++;
      }
      //delay(dt);
      
      Serial.print("j");
      digitalWrite(12, HIGH);
    }
        
 } // end of loop function
