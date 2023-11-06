// UV-3D printer firmware
// 3D printer for print from resin with table swing for better mixing and manual control of table lifting

// To use with Creation Workshop profile mUVe.1.3.slicing

#include <Servo.h> 
Servo servo;  

// Table lift screw pitch 4mm, 3200 microsteps per revolution - 800 steps per mm, 0.00125 mm/step
// Distance 200mm - 50 revolutions
// For a speed of 1600 steps/sec - 0.5 rps - 2mm/sec (100 sec full cycle)

// Table lift screw - distance 65mm, pitch 10mm, 3200 microsteps per revolution - 320 steps per mm, 0.00312 mm/step
// Distance 65mm - 6.5 turns
// For a speed of 1600 steps/sec - 0.5 rps - 5 mm/sec (13 sec full cycle)

int PlatenStepsPerLayer = 80; 
// Steps to move one layer thick. Calculated by dividing Slice Thickness (mm) from the print settings
// Creation Workshop by mm/step
// For a layer of 0.05 mm, a screw with a pitch of 4 mm and 1/16 microstep = 0.05/0.00125=40
// For a layer of 0.1 mm, a screw with a pitch of 4 mm and 1/16 microsteps = 0.1/0.00125=80

int StepsToRaiseLowerBuildPlate = 3200; 
// steps to raise the table between exposures
// For a screw with a pitch of 4mm and 3200 steps per revolution - 3200 corresponds to a rise of 4mm (~2 sec at 1600step/sec, ~4 sec at 800step/sec)


int StepsToRaiseLowerVat = 6400; 
// Steps to raise/lower the rocking table
// For a screw with a pitch of 10 mm and 3200 steps per revolution - 6400 corresponds to a lowering of 20 mm (~2 sec at 3200step/sec, ~4 sec at 1600step/sec)


int TimeForTilt=1000;        // Time to raise/lower the swing table

int TimeForRaiseBuild=1200;  // Time from the start of the cycle to start raising the work plate
int TimeForGoZeroPos=3400;   // Time from the beginning of the cycle from which the plate and table begin to return
                             // to working position before starting the next cycle
                             // should not work before the work plate rises to the top position,
                             // and the swinging table will fall to the bottom
                              
int motorSpeed = 3200; 		// Movement speed in manual control mode, steps/s
int motorAccel = 12000;		// Acceleration of movement in manual control mode, steps/s²

int LiftDownSpeed = 800;	// Speed of lowering the working plate, step/s;
int LiftDownAccel = 4000;	// Acceleration when lowering the work plate, steps/sec²;
int LiftUpSpeed = 1600;		// Speed of raising the working plate, steps/s;
int LiftUpAccel = 12000;	// Acceleration when raising the work plate, steps/s²;

int TiltDownSpeed = 3200;	// Swing table lowering speed, steps/sec;
int TiltDownAccel = 12000;	// Acceleration when lowering the swinging table, steps/s²;
int TiltUpSpeed = 1600;		// Swinging table lowering speed, steps/s;
int TiltUpAccel = 8000;		// Acceleration when raising the swinging table, steps/s²;

bool LiftMotorReverse=false;  	// Inverts the direction of movement for the work plate motor
bool TiltMotorReverse=false; 	// Inverts the direction of motion for the swinging table motor
								// false – up clockwise, down – counterclockwise.
								// true – down clockwise, up – counterclockwise.  
                                                      
int servoPin = 6; 		//pin for connecting the lens servo shutter (if used)
int shutterclosed = 45;	// Position of the lens servo shutter when closing
int shutteropen = 0; 	// Position of the lens servo shutter when opening


#include <AccelStepper.h>




bool LiftUp=false;
bool LiftDown=false;
bool TiltUp=false;
bool TiltDown=false;

int UpZButtontPin=52;
int DownZButtontPin=63;

int TiltMotorDirPin = 55; //x 
int TiltMotorStepPin = 54; //x
int TiltMotorEnablePin = 38; //enable, can be ignored if no enable input on driver
//set up the accelStepper intance, the "1" tells it we are using a driver
AccelStepper TiltStepper(1, TiltMotorStepPin, TiltMotorDirPin); 

int LiftMotorDirPin = 48; //z 
int LiftMotorStepPin = 46; //z 
int LiftMotorEnablePin = 62; //enable, can be ignored if no enable input on driver
//set up the accelStepper intance, the "1" tells it we are using a driver
AccelStepper LiftStepper(1, LiftMotorStepPin, LiftMotorDirPin); 

#define SDATABUFFERSIZE 200
char sdataBuffer[SDATABUFFERSIZE+1]; //Add 1 for NULL terminator
byte sdataBufferIndex = 0;
boolean sstoreString = false; // flag to put the data in our buffer

#define xtop_limit 2	// top limit switch for tilting table (X-axis)
#define xbot_limit 3 	// bottom limit switch for tilting table (X-axis)

#define ztop_limit 19	// top limit switch for lifting table (Z-axis)
#define zbot_limit 18	// bottom limit switch for lifting table (Z-axis)

#define signal_led 13	 // general purpose signal, blinks on board led
#define UVLED 12		 // used for projector modification light sources like uv leds


int SIGNALLED_OFF = LOW; 	// off
int SIGNALLED_ON = HIGH; 	// on

int UVLED_OFF = LOW; 	// off
int UVLED_ON = HIGH; 	// on

int Zpos=0;
int ZabsPos=0;
int Xpos=0;
int CurrPrintLevel=0;

boolean printing3d = false;
boolean HomingBuildPlateTop = false;
boolean HomingBuildPlateBottom = false;

boolean RaiseBuildPlate =false;

boolean CycleLayer = false;
boolean tiltingDown = false;
boolean tiltingUp = false;

boolean on_top_limit = false;
boolean on_bottom_limit = false;

boolean tilt_top_limit = false;
boolean tilt_bottom_limit = false;

bool UpZButtonLast=false;
bool DownZButtonLast=false;

unsigned long LayerCycleTimer = 0;
String LayerCycle_TIME_STRING;
int LayerCycle_TIME = 0;

char buf[10];
char startChar = '<'; //
char endChar = '\r';
char gcodeStartChar = 'G';


void setup()
    {
      // init pc serial
      Serial.begin(115200);
      Serial.flush();
      
  // step/dir controller ***************************************************
      TiltStepper.setCurrentPosition(0);
      TiltStepper.setMaxSpeed(TiltDownSpeed);
      TiltStepper.setAcceleration(TiltDownAccel);
      if (TiltMotorReverse) TiltStepper.setPinsInverted(true, false, false); // adjust the direction of the tilt step motor here (dir,step,enable)
      
      pinMode(TiltMotorDirPin, OUTPUT);
      pinMode(TiltMotorStepPin, OUTPUT);
      pinMode(TiltMotorEnablePin, OUTPUT);
    
      digitalWrite(TiltMotorEnablePin, LOW); // enable controller (if controller has enable connect it to pin), set low or high or ignore depending on controller
      
      LiftStepper.setCurrentPosition(0);
      LiftStepper.setMaxSpeed(motorSpeed);
      LiftStepper.setAcceleration(motorAccel);
      if (LiftMotorReverse) LiftStepper.setPinsInverted(true, false, false); // adjust the direction of the z axis step motor here(dir,step,enable)
    
      pinMode(LiftMotorDirPin, OUTPUT);
      pinMode(LiftMotorStepPin, OUTPUT);    
      pinMode(LiftMotorEnablePin, OUTPUT);
    
      digitalWrite(LiftMotorEnablePin, LOW); // enable controller (if controller has enable connect it to pin), set low or high or ignore depending on controller
  
  //************************************************************************
    // projector shutter (controls a hobby servo)
      servo.attach(servoPin); 
      servo.write(shutterclosed); // close shutter
  
    // init the uv led to off 
      pinMode(UVLED, OUTPUT);
      digitalWrite(UVLED, UVLED_OFF);
   
      // signal led
      pinMode(signal_led,OUTPUT);
      digitalWrite(signal_led,SIGNALLED_OFF);
   
      // limit switches. use pin pullup resistors
      pinMode(xtop_limit,INPUT);
      digitalWrite(xtop_limit,HIGH);
      pinMode(xbot_limit,INPUT);
      digitalWrite(xbot_limit,HIGH);

      pinMode(ztop_limit,INPUT);
      digitalWrite(ztop_limit,HIGH);
      pinMode(zbot_limit,INPUT);
      digitalWrite(zbot_limit,HIGH);

      pinMode(UpZButtontPin,INPUT);
      digitalWrite(UpZButtontPin,HIGH);
      pinMode(DownZButtontPin,INPUT);
      digitalWrite(DownZButtontPin,HIGH);
    }
  
void loop()
  {
      TiltStepper.run(); // for step/dir controllers
      LiftStepper.run(); // for step/dir controllers
      
      while(Serial.available()>0) // get commands from Creation Workshop
      {
          char sincomingbyte = Serial.read();
          if(sincomingbyte==startChar || sincomingbyte == gcodeStartChar)
          {
                  sdataBufferIndex = 0;  //initialize the dataBufferIndex
                  sstoreString = true;
          }
          if(sstoreString)
          {
                  //check the serial buffer, bail if buffer overrun
                  if(sdataBufferIndex==SDATABUFFERSIZE){
                      // index is pointing to an array element outside our buffer. re init the buffer and bail
                      sdataBufferIndex = 0;
                      break;
                  }
                  if(sincomingbyte==endChar) // got a command line, parse it
            {
          Serial.println("ok"); //acknowlege the command

          // Start of the exposition
          if (String(sdataBuffer).substring(1,21) == "LAYER-ON############") 
			//turn on the projector backlight (if there is control)
          {
            digitalWrite(UVLED, UVLED_ON);
            servo.write(shutteropen); // open shutter
          }
          // swing table lowering mode with the image on the projector turned off
          else if (String(sdataBuffer).substring(1,21) == "LAYER-OFF-TILT-VAT##") // 
          {
            if ( printing3d)
            {
              digitalWrite(signal_led, SIGNALLED_ON);
              LayerCycleTimer = millis();  // start the loop timer for the current layer
              CycleLayer = true;  // Start the loop for the current layer
              // begin cycling vat and platen
              servo.write(shutterclosed); // close the lens shutter
              
              digitalWrite(UVLED, UVLED_OFF); //turn off the projector backlight (if there is control)
              TiltStepper.setMaxSpeed(TiltDownSpeed);
              TiltStepper.setAcceleration(TiltDownAccel);
              TiltStepper.moveTo(StepsToRaiseLowerVat * -1); //Lowering the swing table
              RaiseBuildPlate = true;     // turn on the raising of the work plate at the required interval
            }
          }
          
          else if (String(sdataBuffer).substring(1,19) == "LIFT-SEQUENCE-TIME")
			// Set the Lift and Sequence Time variable
			// see creation workshop > slice profile config > Options > Lift and Sequence Time (ms)
          {
            LayerCycle_TIME_STRING = String(sdataBuffer).substring(21,26);
            LayerCycle_TIME = LayerCycle_TIME_STRING.toInt();
            Serial.print("Set LayerCycle TIME to ");
            Serial.println(LayerCycle_TIME);
          }
          // Start printing, set all positions to zero
          else if (String(sdataBuffer).substring(1,21) == "INIT################") // reset position, open shutter
          {
              printing3d = true;
              
              TiltStepper.setCurrentPosition(0);
              TiltStepper.setMaxSpeed(motorSpeed);
              TiltStepper.setAcceleration(motorAccel);
              Xpos=0;
              Serial.print("Current X position: ");
              Serial.println(Xpos);
              Zpos=0;
              ZabsPos=0;
              Serial.print("Current Z position: ");
              Serial.println(ZabsPos);
              LiftStepper.setCurrentPosition(0);
              LiftStepper.setMaxSpeed(motorSpeed);
              LiftStepper.setAcceleration(motorAccel);

  
            servo.write(shutteropen); // open shutter
          }
          // Finishing printing
          else if (String(sdataBuffer).substring(1,21) == "END#################") // reset position, open shutter
          {
              printing3d = false;
              TiltStepper.setCurrentPosition(0);
              TiltStepper.stop();
              LiftStepper.setCurrentPosition(0);
              LiftStepper.stop();
              servo.write(shutterclosed); // close shutter
          }
          // Manual control - lowering the swing table
          else if (String(sdataBuffer).substring(0,6) == "G28 X0") // Stop raising the swing table
          {
            TiltStepper.stop();
          }
          else if (String(sdataBuffer).substring(0,7) == "G1 X100") // Raise the swing table
          {
            if (tilt_top_limit == false) // not already on bottom limit
              {
                
                TiltStepper.setMaxSpeed(motorSpeed);
                TiltStepper.setAcceleration(motorAccel);
                TiltStepper.move(100000); // move towards bottom limit
              }
          }
          else if (String(sdataBuffer).substring(0,8) == "G1 X-100") // Lower the swing table
          {
            if (tilt_bottom_limit == false) // not already on bottom limit
              {
                
                TiltStepper.setMaxSpeed(motorSpeed);
                TiltStepper.setAcceleration(motorAccel);
                TiltStepper.move(-100000); // move towards bottom limit
              }
          }
          // Manual control - lowering the swing table
          else if (String(sdataBuffer).substring(0,7) == "G1 X10 ") // Raise the swing table
          {
            if (tilt_top_limit == false) // not already on bottom limit
              {
                Xpos=TiltStepper.currentPosition();
                TiltStepper.setMaxSpeed(motorSpeed);
                TiltStepper.setAcceleration(motorAccel);
                TiltStepper.moveTo(0); 
                
              }
          }
          
          else if (String(sdataBuffer).substring(0,8) == "G1 X-10 ") // Lower the swing table
          {
            if (tilt_bottom_limit == false) // not already on bottom limit
              {
                Xpos=TiltStepper.currentPosition();
                TiltStepper.setMaxSpeed(motorSpeed);
                TiltStepper.setAcceleration(motorAccel);
                TiltStepper.moveTo(StepsToRaiseLowerVat*(-1)); 
              }
          }
          // Manual control - lowering the swing table
          else if (String(sdataBuffer).substring(0,6) == "G1 X1 ") // Raise the swing table
          {
            if (tilt_top_limit == false) // not already on bottom limit
              {
                Xpos=TiltStepper.currentPosition();
                /*Serial.print("Current X position: ");
                Serial.print(Xpos);
                Serial.print(" begint run to 0");*/
                TiltStepper.setMaxSpeed(motorSpeed);
                TiltStepper.setAcceleration(motorAccel);
                TiltStepper.move(16); 
                
              }
          }
          
          else if (String(sdataBuffer).substring(0,7) == "G1 X-1 ") // Lower the swing table
          {
            if (tilt_bottom_limit == false) // not already on bottom limit
              {
                Xpos=TiltStepper.currentPosition();

                TiltStepper.setMaxSpeed(motorSpeed);
                TiltStepper.setAcceleration(motorAccel);
                TiltStepper.move(-16); 
              }
          }
          else if (String(sdataBuffer).substring(0,6) == "G28 Z0") // Raise the swing table
          {
            LiftStepper.stop();
          }
          // Manual control - lowering the table by 1 step of the stepper
          else if (String(sdataBuffer).substring(0,7) == "G1 Z-1 ") // bottom limit (vat floor)
          {
              if (on_bottom_limit == false) // not already on bottom limit
                {
                  LiftStepper.setMaxSpeed(motorSpeed);
                  LiftStepper.setAcceleration(motorAccel);
                  LiftStepper.move(-16);
                }
          }
          else if (String(sdataBuffer).substring(0,6) == "G1 Z1 ") // bottom limit (vat floor)
          {
              if (on_top_limit == false) // not already on bottom limit
                {
                  LiftStepper.setMaxSpeed(motorSpeed);  
                  LiftStepper.setAcceleration(motorAccel);
                  LiftStepper.move(16);
                }
          }
          // Ручное упраление - поднятие стола на 1 оборот
          else if (String(sdataBuffer).substring(0,7) == "G1 Z50 ") // bottom limit (vat floor)
          {
                if (on_top_limit == false) // not already on bottom limit
                  {
                    LiftStepper.setMaxSpeed(motorSpeed); 
                    LiftStepper.setAcceleration(motorAccel);
                    LiftStepper.move(100000); // To extreme position
                  }
          }
          // Manual control - table lowering 1 revolution
          else if (String(sdataBuffer).substring(0,8) == "G1 Z-50 ") // bottom limit (vat floor)
          {
                if (on_bottom_limit == false) // not already on bottom limit
                  {
                    LiftStepper.setMaxSpeed(motorSpeed); 
                    LiftStepper.setAcceleration(motorAccel);
                    LiftStepper.move(-100000);  // To extreme position
                  }
          }                      
          else if (String(sdataBuffer).substring(0,7) == "G1 Z-10") // bottom limit (vat floor)
          {
                                          if (on_bottom_limit == false) // not already on bottom limit
                                            {
                                              HomingBuildPlateBottom = true;
                                              LiftStepper.setMaxSpeed(motorSpeed);
                                              LiftStepper.setAcceleration(motorAccel);
                                              LiftStepper.move(-3200); // move towards bottom limit
                                            }
          }

          else if (String(sdataBuffer).substring(0,6) == "G1 Z10") // top limit
          {
                                          if (on_top_limit == false) // not already on top limit
                                            {
                                              HomingBuildPlateTop = true;
                                              LiftStepper.setMaxSpeed(motorSpeed);
                                              LiftStepper.setAcceleration(motorAccel);
                                              LiftStepper.move(3200); // move towards top limit
                                            }
          }



          sstoreString = false;
        }
              else
        {
                  sdataBuffer[sdataBufferIndex++] = sincomingbyte;
                  sdataBuffer[sdataBufferIndex] = 0; //null terminate the C string
        }
      }
    }
       // Table raise mode
      if (RaiseBuildPlate == true)
      {
          if (LayerCycleTimer + TimeForRaiseBuild < millis())  //time to raise build plate )
              {
                //ZabsPos+= PlatenStepsPerLayer;
                LiftStepper.setCurrentPosition(LiftStepper.currentPosition() - PlatenStepsPerLayer);
                // Текущая позиция сдвигается на толщину слоя 
                LiftStepper.setMaxSpeed(LiftUpSpeed);
                LiftStepper.setAcceleration(LiftUpAccel);
                LiftStepper.moveTo(StepsToRaiseLowerBuildPlate); // lift build
                // Поднять стол на в нерабочее положение
                RaiseBuildPlate = false;
                // Конец цикла 
                //Serial.println("Time to raise Build Plate");                             
              }
      }


        if (CycleLayer == true)
        {
          if (LayerCycleTimer + TimeForGoZeroPos < millis()) // return the plate and table to the zero position
          {
              //Serial.println("Time to return Build Plate and tilt to 0");
              LiftStepper.setMaxSpeed(LiftDownSpeed);
             LiftStepper.setAcceleration(LiftDownAccel);
              LiftStepper.moveTo(0);
              TiltStepper.setMaxSpeed(TiltUpSpeed);
              TiltStepper.setAcceleration(TiltUpAccel);
              TiltStepper.moveTo(0); // un-tilt the vat
          }
        
          if (LayerCycleTimer + LayerCycle_TIME < millis()) // End of loop for current layer
          {
                Serial.println("Current cycle End");
                servo.write(shutteropen); // open shutter
                digitalWrite(signal_led, SIGNALLED_OFF);
                CycleLayer = false; // done cycling layer
          }
      }

      
      // Processing events of the upper end of the work plate
      if (digitalRead(ztop_limit) == LOW and on_top_limit == false) //the upper limit switch of the work plate is pressed
      {
          digitalWrite(signal_led,HIGH);
          Serial.println("Z top limit STOP");     
          if (printing3d == false)
            {
              on_top_limit = true;
              LiftStepper.stop();
              delay(250);
              //LiftStepper.moveTo(10000);
              LiftStepper.setCurrentPosition(100000);
              LiftStepper.setMaxSpeed(motorSpeed);
              LiftStepper.setAcceleration(motorAccel);
            } 
           else
           {
                printing3d = false;
                TiltStepper.setCurrentPosition(0);
                TiltStepper.stop();
                LiftStepper.setCurrentPosition(0);
                LiftStepper.stop();
                servo.write(shutterclosed); // close shutter
                on_top_limit = true;
                LiftStepper.stop();        
           }
      }
      if (digitalRead(ztop_limit) == HIGH and on_top_limit == true) // the upper limit switch of the working plate is released
      {
        on_top_limit = false;
        digitalWrite(signal_led,LOW);
        Serial.println("Z top limit NORMAL");
      }

      // Processing events of the lower end of the work plate
      if (digitalRead(zbot_limit) == LOW and on_bottom_limit == false) // the lower limit switch of the work plate is pressed
      {
          digitalWrite(signal_led,HIGH);
          Serial.println("Z bottom limit STOP");
          if (printing3d == false)    // If we are not in print mode
            {
              digitalWrite(signal_led,HIGH);
              
              on_bottom_limit = true;
              if (LiftDown) LiftStepper.stop();
              
              delay(250);
 
              LiftStepper.setCurrentPosition(0);
              LiftStepper.setMaxSpeed(motorSpeed);
              LiftStepper.setAcceleration(motorAccel);

              //Raising the swing table
              TiltStepper.setMaxSpeed(motorSpeed);
              TiltStepper.setAcceleration(motorAccel);
              TiltStepper.move(30000) ;
            }
            else
            {
              //LiftStepper.stop();
            }
      }

    if (digitalRead(zbot_limit) == HIGH and on_bottom_limit == true) //the lower limit switch of the working plate is released
      {
      Serial.println("Z bottom limit NORMAL");
      on_bottom_limit = false;
      digitalWrite(signal_led,LOW);
      }

     // Handling events of the upper limit switch of the swinging table
    if (digitalRead(xtop_limit) == LOW and tilt_top_limit == false) // The upper limit switch of the tilting table is pressed
      {
        tilt_top_limit=true;
        Serial.println("Tilt top limit STOP");
        if (printing3d == false)    // If we are not in print mode
        {
        TiltStepper.setCurrentPosition(0);
        digitalWrite(signal_led,LOW);

        TiltStepper.stop();
        }
      }
      if (digitalRead(xtop_limit) == HIGH and tilt_top_limit == true) //The upper limit switch of the tilting table is released
      {
         tilt_top_limit=false;
         Serial.println("Tilt top limit NORMAL");
         digitalWrite(signal_led,LOW);
      }
      // Обработка событий нижнего концевика качающегося стола

      if (digitalRead(xbot_limit) == LOW and tilt_bottom_limit == false) //the lower limit switch of the tilting table is pressed
      {
        if (printing3d == false)    // If we are not in print mode
        {
          tilt_bottom_limit=true;
          digitalWrite(signal_led,LOW);
          Serial.println("Tilt bottom limit STOP");
          TiltStepper.stop();
        }
      }
      if (digitalRead(xbot_limit) == HIGH and tilt_bottom_limit == true) //the lower limit switch of the tilting table is released
      {
         Serial.println("Tilt bottom limit NORMAL");
         tilt_bottom_limit=false;
         digitalWrite(signal_led,LOW);
      }

// Up button released
if ((digitalRead(UpZButtontPin) == HIGH)&&(UpZButtonLast==true))
{
  Serial.println("Up button Stop");

  if (!printing3d) 
  {
    if (LiftDown)   // If in downward mode
    {
      LiftDown=false;       // Stop
      LiftStepper.stop();
    }
    else
    if (on_top_limit == false) // not already on bottom limit
          {
            
            LiftUp = true;

            LiftStepper.setMaxSpeed(2000);
            LiftStepper.setAcceleration(12000);
            LiftStepper.move(100000); // move towards bottom limit
          }
         else 
         {
            LiftStepper.stop();
         }
  }
  UpZButtonLast=false;
}
// Up button pressed
    if ((digitalRead(UpZButtontPin) == LOW)&&(UpZButtonLast==false))
    {
      Serial.println("Up button ");
      if (printing3d) // In print mode, stops printing
      {
        printing3d = false;
        LiftStepper.stop();
        TiltStepper.stop();
      }
     
      UpZButtonLast=true;
    }
    
    // Down button released
    if ((digitalRead(DownZButtontPin) == HIGH)&&(DownZButtonLast==true))
    {
      Serial.println("Down button Stop");

      if (!printing3d)    //If not in print mode
      {
        if (LiftUp)
        {
          LiftUp=false;
          LiftStepper.stop();
        }
        else
        if (on_bottom_limit == false) // not already on bottom limit
          {
            LiftDown = true;
            LiftStepper.setMaxSpeed(2000);
            LiftStepper.setAcceleration(12000);
            LiftStepper.move(-100000); // move towards bottom limit
          }
         else 
         {
            LiftStepper.stop();
         }
      }
      DownZButtonLast=false;
    }
  // Down button pressed
  if ((digitalRead(DownZButtontPin) == LOW)&&(DownZButtonLast==false))
    {
      Serial.println("Down button");
      if (printing3d)    // In print mode, stops printing
      {
        printing3d = false;
        LiftStepper.stop();
        TiltStepper.stop();
      }

      DownZButtonLast=true;
    }
      
}


