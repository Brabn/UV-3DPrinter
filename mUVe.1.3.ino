// Прошивка для UV принтера
// использовать совместно с Creation Workshop

#include <Servo.h> 
Servo servo;  

// Винт поднятия стола шаг 4мм, 3200 микрошагов на оборот - 800 шагов на мм, 0,00125 мм/шаг
// Дистанция 200мм - 50 оборотов
// Для скорости 1600 шагов/сек - 0,5 об/сек - 2мм/сек (100 сек полный цикл)

// Винт поднятия стола - дистанция 65мм, шаг 10мм, 3200 микрошагов на оборот - 320 шагов на мм, 0,00312 мм/шаг
// Дистанция 65мм - 6,5 оборотов
// Для скорости 1600 шагов/сек - 0,5 об/сек - 5 мм/сек (13 сек полный цикл)

int PlatenStepsPerLayer = 80; // Шагов для перемещения на толщину одного слоя. Вычисляется делением Slice Thickness (mm) из настроек печати
//  Creation Workshop на величину мм/шаг 
// Для слоя 0,05 мм, винта с шагом 4мм и 1/16 микрошага = 0,05/0,00125=40
// Для слоя 0,1 мм, винта с шагом 4мм и 1/16 микрошага = 0,1/0,00125=80

int StepsToRaiseLowerBuildPlate = 3200; // шагов на поднятия стола между экспозициями
// Для винта с шагом 4мм и 3200 шагов на оборот - 3200 соответствует поднятию на 4мм (~2 сек при 1600шаг/сек, ~4 сек при 800шаг/сек)

// steps to raise/lower the build plate between exposures. I use 400 step per revolution motors microstepped times 16 for 6400 steps per revolution.
int StepsToRaiseLowerVat = 6400; // Шагов для поднятия/опускания качающегося стола
// Для винта с шагом 10 мм и 3200 шагов на оборот - 6400 соответствует опусканию на 20мм (~2 сек при 3200шаг/сек, ~4 сек при 1600шаг/сек)


int TimeForTilt=1000;        // Время для поднятия/опускания качающегося стола 

int TimeForRaiseBuild=1200;  // Время от начала цикла, для начала поднятия рабочей плиты 
int TimeForGoZeroPos=3400;   // Время от начала цикла, с которого плита и стол начинают возвращаться 
                              // в рабочее положение перед началом следующего цикла 
                              
                              // не должно сработать раньше, чем рабочая плита подымется в верхнее положение, 
                              // а качающийся стол - опустится в нижнее 
                              
int motorSpeed = 3200; //Скорость перемещения в режиме ручного управления, шак/сек
int motorAccel = 12000; //Ускорение перемещения в режиме ручного управления, шак/сек2

int LiftDownSpeed = 800;   // Скорость опускания рабочей плиты, шак/сек
int LiftDownAccel = 4000; // Ускорение при опускания рабочей плиты, шак/сек2
int LiftUpSpeed = 1600;  // Скорость поднятия рабочей плиты, шак/сек
int LiftUpAccel = 12000; // Ускорение при поднятии рабочей плиты, шак/сек2

int TiltDownSpeed = 3200; // Скорость опускания качающегося стола , шак/сек
int TiltDownAccel = 12000; // Ускорение при опускании качающегося стола , шак/сек2
int TiltUpSpeed = 1600;  // Скорость опускания качающегося стола, шак/сек
int TiltUpAccel = 8000; // Ускорение при поднятии качающегося стола, шак/сек2 

bool LiftMotorReverse=false;  // Инвертирует направление движения для двигателя  рабочей плиты. 
bool TiltMotorReverse=false;  // Инвертирует направление движения для двигателя качающегося стола
                              // false – вверх по часовой стрелке, вниз – против.
                              // true – вниз по часовой стрелке, вверх – против.   
                                                      
int servoPin = 6; //Пин подключения сервозаслонки объектива (при использовании)
int shutterclosed = 45; // Положение сервозаслонки объектива при закрытии 
int shutteropen = 0; // Положение сервозаслонки объектива при открытии 


#include <AccelStepper.h>



/*int TiltDownSpeed = 400; // vat tilt down speed
int TiltDownAccel = 200; // vat tilt down acceleration

int TiltUpSpeed = 1600; // vat tilt up speed
int TiltUpAccel = 1600; // vat tilt up acceleration
*/

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

#define xtop_limit 2 //концевик для качающегося стола верхний X
#define xbot_limit 3 //концевик для качающегося стола нижний верхний X

#define ztop_limit 19 //концевик для поднятия плиты верхний Z
#define zbot_limit 18 ////концевик для поднятия плиты нижний Z

#define signal_led 13 // general purpose signal, blinks on board led
#define UVLED 12 // used for projector modification light sources like uv leds


int SIGNALLED_OFF = LOW; // off
int SIGNALLED_ON = HIGH; // on

int UVLED_OFF = LOW; // off
int UVLED_ON = HIGH; // on

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

          // Начало экспозиции 
          if (String(sdataBuffer).substring(1,21) == "LAYER-ON############") 
          //включаем подсветку проектора (если есть управление)
          {
            digitalWrite(UVLED, UVLED_ON);
            servo.write(shutteropen); // open shutter
          }
          // режим опускания качающегося стола с выключенным изображением на проекторе
          else if (String(sdataBuffer).substring(1,21) == "LAYER-OFF-TILT-VAT##") // 
          {
            if ( printing3d)
            {
              digitalWrite(signal_led, SIGNALLED_ON);
              LayerCycleTimer = millis();  // запускаем таймер цикла для текущего слоя
              CycleLayer = true;  // Начинаем цикл для текущего слоя
              // begin cycling vat and platen
              servo.write(shutterclosed); // закрываем заслонку объектива
              
              digitalWrite(UVLED, UVLED_OFF); //выключаем подсветку проектора (если есть управление)
              TiltStepper.setMaxSpeed(TiltDownSpeed);
              TiltStepper.setAcceleration(TiltDownAccel);
              TiltStepper.moveTo(StepsToRaiseLowerVat * -1); //Опускаем качающийся стол
              RaiseBuildPlate = true;     // включаем поднятие рабочей плиты через нужный интервал
            }
          }
          
          else if (String(sdataBuffer).substring(1,19) == "LIFT-SEQUENCE-TIME")
          // Установка переменной  Lift and Sequence Time
          // см. creation workshop > slice profile config > Options > Lift and Sequence Time (ms)
          {
            LayerCycle_TIME_STRING = String(sdataBuffer).substring(21,26);
            LayerCycle_TIME = LayerCycle_TIME_STRING.toInt();
            Serial.print("Set LayerCycle TIME to ");
            Serial.println(LayerCycle_TIME);
          }
          // Начало печати, установка всех позиций в ноль
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
          // Конец печати
          else if (String(sdataBuffer).substring(1,21) == "END#################") // reset position, open shutter
          {
              printing3d = false;
              TiltStepper.setCurrentPosition(0);
              TiltStepper.stop();
              LiftStepper.setCurrentPosition(0);
              LiftStepper.stop();
              servo.write(shutterclosed); // close shutter
          }
          // Ручное упраление - опускание качающегося стола 
          else if (String(sdataBuffer).substring(0,6) == "G28 X0") // Поднять качающийся стол
          {
            TiltStepper.stop();
          }
          else if (String(sdataBuffer).substring(0,7) == "G1 X100") // Поднять качающийся стол
          {
            if (tilt_top_limit == false) // not already on bottom limit
              {
                
                TiltStepper.setMaxSpeed(motorSpeed);
                TiltStepper.setAcceleration(motorAccel);
                TiltStepper.move(100000); // move towards bottom limit
              }
          }
          else if (String(sdataBuffer).substring(0,8) == "G1 X-100") // Опустить качающийся стол
          {
            if (tilt_bottom_limit == false) // not already on bottom limit
              {
                
                TiltStepper.setMaxSpeed(motorSpeed);
                TiltStepper.setAcceleration(motorAccel);
                TiltStepper.move(-100000); // move towards bottom limit
              }
          }
          // Ручное упраление - опускание качающегося стола 
          else if (String(sdataBuffer).substring(0,7) == "G1 X10 ") // Поднять качающийся стол
          {
            if (tilt_top_limit == false) // not already on bottom limit
              {
                Xpos=TiltStepper.currentPosition();
                /*Serial.print("Current X position: ");
                Serial.print(Xpos);
                Serial.print(" begint run to 0");*/
                TiltStepper.setMaxSpeed(motorSpeed);
                TiltStepper.setAcceleration(motorAccel);
                TiltStepper.moveTo(0); 
                
              }
          }
          
          else if (String(sdataBuffer).substring(0,8) == "G1 X-10 ") // Опустить качающийся стол
          {
            if (tilt_bottom_limit == false) // not already on bottom limit
              {
                Xpos=TiltStepper.currentPosition();
                /*Serial.print("Current X position: ");
                Serial.print(Xpos);
                Serial.print(" begint run to bottom");*/
                TiltStepper.setMaxSpeed(motorSpeed);
                TiltStepper.setAcceleration(motorAccel);
                TiltStepper.moveTo(StepsToRaiseLowerVat*(-1)); 
              }
          }
          // Ручное упраление - опускание качающегося стола 
          else if (String(sdataBuffer).substring(0,6) == "G1 X1 ") // Поднять качающийся стол
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
          
          else if (String(sdataBuffer).substring(0,7) == "G1 X-1 ") // Опустить качающийся стол
          {
            if (tilt_bottom_limit == false) // not already on bottom limit
              {
                Xpos=TiltStepper.currentPosition();
                /*Serial.print("Current X position: ");
                Serial.print(Xpos);
                Serial.print(" begint run to bottom");*/
                TiltStepper.setMaxSpeed(motorSpeed);
                TiltStepper.setAcceleration(motorAccel);
                TiltStepper.move(-16); 
              }
          }
          else if (String(sdataBuffer).substring(0,6) == "G28 Z0") // Поднять качающийся стол
          {
            LiftStepper.stop();
          }
          // Ручное упраление - опускание стола на 1 шаг шаговика
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
                  LiftStepper.setMaxSpeed(motorSpeed);  //2000 шагов в сек - 
                  LiftStepper.setAcceleration(motorAccel);
                  LiftStepper.move(16);
                }
          }
          // Ручное упраление - поднятие стола на 1 оборот
          else if (String(sdataBuffer).substring(0,7) == "G1 Z50 ") // bottom limit (vat floor)
          {
                if (on_top_limit == false) // not already on bottom limit
                  {
                    LiftStepper.setMaxSpeed(motorSpeed);  //3200 шагов в сек - оборот в сек
                    LiftStepper.setAcceleration(motorAccel);
                    LiftStepper.move(100000); // В крайнее положение
                  }
          }
          // Ручное упраление - опускание стола на 1 оборот
          else if (String(sdataBuffer).substring(0,8) == "G1 Z-50 ") // bottom limit (vat floor)
          {
                if (on_bottom_limit == false) // not already on bottom limit
                  {
                    LiftStepper.setMaxSpeed(motorSpeed);  //3200 шагов в сек - оборот в сек
                    LiftStepper.setAcceleration(motorAccel);
                    LiftStepper.move(-100000); // В крайнее положение
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
       // Режим поднятия стола 
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
          if (LayerCycleTimer + TimeForGoZeroPos < millis()) // возвращаем в нулевое положение плиту и стол
          {
              //Serial.println("Time to return Build Plate and tilt to 0");
              LiftStepper.setMaxSpeed(LiftDownSpeed);
             LiftStepper.setAcceleration(LiftDownAccel);
              LiftStepper.moveTo(0);
              TiltStepper.setMaxSpeed(TiltUpSpeed);
              TiltStepper.setAcceleration(TiltUpAccel);
              TiltStepper.moveTo(0); // un-tilt the vat
          }
        
          if (LayerCycleTimer + LayerCycle_TIME < millis()) // Конец цикла для текущего слоя
          {
                Serial.println("Current cycle End");
                servo.write(shutteropen); // open shutter
                digitalWrite(signal_led, SIGNALLED_OFF);
                CycleLayer = false; // done cycling layer
          }
      }

      
      // Обработка событий верхнего концевика рабочей плиты
      if (digitalRead(ztop_limit) == LOW and on_top_limit == false) //верхний концевик рабочей плиты  нажат
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
      if (digitalRead(ztop_limit) == HIGH and on_top_limit == true) // верхний концевик рабочей плиты отпущен
      {
        on_top_limit = false;
        digitalWrite(signal_led,LOW);
        Serial.println("Z top limit NORMAL");
      }

      // Обработка событий нижнего концевика рабочей плиты
      if (digitalRead(zbot_limit) == LOW and on_bottom_limit == false) // нижний концевик рабочей плиты нажат
      {
          digitalWrite(signal_led,HIGH);
          Serial.println("Z bottom limit STOP");
          if (printing3d == false)    // Если мы не в режиме печати
            {
              digitalWrite(signal_led,HIGH);
              
              on_bottom_limit = true;
              if (LiftDown) LiftStepper.stop();
              
              delay(250);
              //LiftStepper.moveTo(0); 
              LiftStepper.setCurrentPosition(0);
              LiftStepper.setMaxSpeed(motorSpeed);
              LiftStepper.setAcceleration(motorAccel);

              // Подымаем качающийся стол
              TiltStepper.setMaxSpeed(motorSpeed);
              TiltStepper.setAcceleration(motorAccel);
              TiltStepper.move(30000) ;
            }
            else
            {
              //LiftStepper.stop();
            }
      }

    if (digitalRead(zbot_limit) == HIGH and on_bottom_limit == true) // нижний концевик рабочей плиты отпущен
      {
      Serial.println("Z bottom limit NORMAL");
      on_bottom_limit = false;
      digitalWrite(signal_led,LOW);
      }

     // Обработка событий верхнего концевика качающегося стола
    if (digitalRead(xtop_limit) == LOW and tilt_top_limit == false) // верхнеий концевик качающегося стола нажат
      {
        tilt_top_limit=true;
        Serial.println("Tilt top limit STOP");
        if (printing3d == false)    // Если мы не в режиме печати
        {
        TiltStepper.setCurrentPosition(0);
        digitalWrite(signal_led,LOW);
        /*Xpos=TiltStepper.currentPosition();
        Serial.print("Current X position: ");
        Serial.println(Xpos);*/
        TiltStepper.stop();
        }
      }
      if (digitalRead(xtop_limit) == HIGH and tilt_top_limit == true) //верхнеий концевик качающегося стола отпущен
      {
         tilt_top_limit=false;
         Serial.println("Tilt top limit NORMAL");
         digitalWrite(signal_led,LOW);
      }
      // Обработка событий нижнего концевика качающегося стола

      if (digitalRead(xbot_limit) == LOW and tilt_bottom_limit == false) //нижний концевик качающегося стола  нажат
      {
        if (printing3d == false)    // Если мы не в режиме печати
        {
          tilt_bottom_limit=true;
          digitalWrite(signal_led,LOW);
          Serial.println("Tilt bottom limit STOP");
          /*Xpos=TiltStepper.currentPosition();
          Serial.print("Current X position: ");
          Serial.print(Xpos);*/
          TiltStepper.stop();
        }
      }
      if (digitalRead(xbot_limit) == HIGH and tilt_bottom_limit == true) //нижний концевик качающегося стола отпущен
      {
         Serial.println("Tilt bottom limit NORMAL");
         tilt_bottom_limit=false;
         digitalWrite(signal_led,LOW);
      }

// Кнопка вверх отпущена
if ((digitalRead(UpZButtontPin) == HIGH)&&(UpZButtonLast==true))
{
  Serial.println("Up button Stop");
  /*HomingBuildPlateBottom = false;
  if (LiftUp) 
  {
    LiftStepper.stop();
    LiftUp = false;
  }*/
  if (!printing3d) 
  {
    if (LiftDown)   // Если в режиме перемещения вниз
    {
      LiftDown=false;       //Останавливаемся
      LiftStepper.stop();
    }
    else
    if (on_top_limit == false) // not already on bottom limit
          {
            
            LiftUp = true;
            //HomingBuildPlateTop = true;
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
// Кнопка вверх нажата
    if ((digitalRead(UpZButtontPin) == LOW)&&(UpZButtonLast==false))
    {
      Serial.println("Up button ");
      if (printing3d) // В режиме печати останавливает печать
      {
        printing3d = false;
        LiftStepper.stop();
        TiltStepper.stop();
      }
      /*else
      {
        if (on_top_limit == false) // not already on bottom limit
          {
            
            LiftUp = true;
            //HomingBuildPlateTop = true;
            LiftStepper.setMaxSpeed(2000);
            LiftStepper.setAcceleration(12000);
            LiftStepper.move(100000); // move towards bottom limit
          }
         else 
         {
            LiftStepper.stop();
         }
      }*/
      UpZButtonLast=true;
    }
    
    // Кнопка вниз отпущена
    if ((digitalRead(DownZButtontPin) == HIGH)&&(DownZButtonLast==true))
    {
      Serial.println("Down button Stop");
     /* HomingBuildPlateBottom = false;
      if (LiftDown) 
      {
        LiftStepper.stop();
        LiftDown = false;
      }*/
      if (!printing3d)    // Если не в режиме печати
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
  // Кнопка вниз нажата
  if ((digitalRead(DownZButtontPin) == LOW)&&(DownZButtonLast==false))
    {
      Serial.println("Down button");
      if (printing3d)    // В режиме печати останавливает печать
      {
        printing3d = false;
        LiftStepper.stop();
        TiltStepper.stop();
      }
      /*else
      {
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
      }*/
      DownZButtonLast=true;
    }
      
}


