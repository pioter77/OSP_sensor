#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDR 0x68 // When nothing is connected to the addres pin of sensor this is its i2c address
#define REF_COUNT 100 // 100 pomiarow sredniej

//main structore for storing values read from sensor
struct gyro_stru{
  float acc_X,acc_Y,acc_Z;
  float gyro_X,gyro_Y,gyro_Z; ///this values can can descend below 0!
  float temperature;
  float  angleAccX;
  float  angleAccY;
}gyro_holder{};

gyro_stru reference_holder{0,0,0,0,0};  //used when we want to set reference point away from ground level
gyro_stru adjust_holder{0,0,0,0,0}; //groudn level calibaration offset

char buff[6]; //Used for storing orders got from labview master through serial port
unsigned long timer=0;  //used for time synchro in main loop eg to get values every 1 second

//-----------------------------------------------------------------FUNCTION DECLARATIONS----------------------------------------//
void gyro_init_fcn(void);
void gyro_readout_fcn(gyro_stru *gs);
void gyro_serialtestprint(gyro_stru *gs,gyro_stru *ref_s,gyro_stru *adj_s);
void set_angle_reference(gyro_stru *gs,gyro_stru *ref);
bool recieve_order(gyro_stru *gs,gyro_stru *ref_s,gyro_stru *adj_s);
void gyro_serial_print(gyro_stru *gs,gyro_stru *ref_s,gyro_stru *adj_s);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  gyro_init_fcn();
  // gyro_set_reference(&gyro_holder,&start_offset);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long ac_time=millis();
  recieve_order(&gyro_holder,&reference_holder,&adjust_holder);   //process the data got form com port immediatelly
  
  //testowo co 1 sekunde
  if(ac_time-timer>100)
  {   //send and refresh the sensor values every 1 second for now
    gyro_readout_fcn(&gyro_holder);
   // gyro_serialtestprint(&gyro_holder,&reference_holder,&adjust_holder);
    gyro_serial_print(&gyro_holder,&reference_holder,&adjust_holder);
    timer=millis();
  }
}

//todo : introduce the option to set sensor parameters such sensitivity or sleep 
void gyro_init_fcn(void)
{
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); //power register disables sleep and tem redout internal 8 mhz osc
  Wire.write(0);
 // Wire.write(0x1C);//setiing gyro sensitivity registers to 500
 // Wire.write(0b00001000);
  //Wire.endTransmission(false);
  //Wire.write(0x1C); //acc sensi
 // Wire.write(0b00010000); //8g

   Wire.endTransmission(false);
  Wire.write(0x1B); //gyr sensi
  Wire.write(0b00000000); //250
  Wire.endTransmission();
}

//gyro accelerometer and temperature readout refresh
void gyro_readout_fcn(gyro_stru *gs)
{
  //fucnction to read all interesting values from gyro to struct 
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,14,true);

  gs->acc_X=(Wire.read()<<8 | Wire.read() )/16384.0;//2 reg 59-
  gs->acc_Y=(Wire.read()<<8 | Wire.read() )/ 16384.0;//4
  gs->acc_Z=(Wire.read()<<8 | Wire.read() )/ 16384.0;//6 reg -64

  gs->temperature=Wire.read()<<8 | Wire.read(); //8 reg 65,66

  gs->gyro_X=(Wire.read()<<8 | Wire.read())/131.0;//2 reg 67-
  gs->gyro_Y=(Wire.read()<<8 | Wire.read())/131.0;//4
  gs->gyro_Z=(Wire.read()<<8 | Wire.read())/131.0;//6 reg -72

  gs->angleAccX=atan2(gs->acc_Y,  gs->acc_Z + abs(gs->acc_X)) * 360 / 2.0 / PI;
  gs->angleAccY=atan2(gs->acc_X,  gs->acc_Z + abs(gs->acc_Y)) * 360 / -2.0 / PI;

}

//original struct for storing up to date records. then reference structure for storing reference values
void set_angle_reference(gyro_stru *gs,gyro_stru *ref)
{
  //set refernce point this should also be run once at in startup of microcontroller to zero in
  float tempx,tempy,tempz;
  float tempangX,tempangY;
  for(int i=0;i<REF_COUNT;i++)
  {
    gyro_readout_fcn(gs);
    tempx+=gs->acc_X;
    tempy+=gs->acc_Y;
    tempz+=gs->acc_Z;
    tempangX+=gs->angleAccX;
    tempangY+=gs->angleAccY;
  }
  ref->acc_X=tempx/(float)REF_COUNT;
  ref->acc_Y=tempy/(float)REF_COUNT;
  ref->acc_Z=tempz/(float)REF_COUNT;  //values saved to structure
  ref->angleAccX=tempangX/(float)REF_COUNT;
  ref->angleAccY=tempangY/(float)REF_COUNT;
}
//serial readout printout for development and debugging
void gyro_serialtestprint(gyro_stru *gs,gyro_stru *ref_s,gyro_stru *adj_s)
{
  float Xfromref,Yfromref;
  float Xfromadj,Yfromadj;

  Xfromadj=gs->angleAccX-adj_s->angleAccX;
  Yfromadj=gs->angleAccY-adj_s->angleAccY;
  Xfromref=gs->angleAccX-adj_s->angleAccX-ref_s->angleAccX;
  Yfromref=gs->angleAccY-adj_s->angleAccY-ref_s->angleAccY;
  //float  angleAccX = atan2(newY,  newZ + abs(newX)) * 360 / 2.0 / PI;
  //float  angleAccY = atan2(newX,  newZ + abs(newY)) * 360 / -2.0 / PI;
  Serial.print("xangle:"); Serial.print(gs->angleAccX);
  Serial.print("  |  yangle: "); Serial.print(gs->angleAccY);
  Serial.println();
  Serial.print("xadjusted:"); Serial.print(Xfromadj);
  Serial.print("  |  yadjusted: "); Serial.print(Yfromadj);
  Serial.println();
  Serial.print("xfromref:"); Serial.print(Xfromref);
  Serial.print("  |  yfromref: "); Serial.print(Yfromref);
  Serial.println();
  Serial.println();
}

//todo send data formatted for labview
void gyro_serial_print(gyro_stru *gs,gyro_stru *ref_s,gyro_stru *adj_s)
{
  //final function forr labview program, the data being sent should be formatted in a way suitable for labview

float Xfromref;
Xfromref=gs->angleAccX-adj_s->angleAccX-ref_s->angleAccX;
Serial.println(Xfromref);

  /*final forumla:
  +angle after zero coreection 
  +angle relative to selected point
  +raw accx accy accz
  +raw gyro 4 axis data

  */
}

//check wheather sth was sent to arduino through serial port and interpret the order
bool recieve_order(gyro_stru *gs,gyro_stru *ref_s,gyro_stru *adj_s)
{
  int i=0;
  if(Serial.available()>0){
  while(Serial.available()>0)
  {
    buff[i++]=Serial.read();
  }
  }else{
    return 0; //to speed up operations
  }

  int order=atoi(buff);

  switch (order)
  {
    case 1: //set reference angle
      set_angle_reference(gs,ref_s);
      Serial.println("refrence angle set");
    return 0;

    case 2: //remove reference angle
      ref_s->acc_X=0;
      ref_s->acc_Y=0;
      ref_s->acc_Z=0;
      ref_s->gyro_X=0;
      ref_s->gyro_Y=0;
      ref_s->gyro_Z=0;
      ref_s->angleAccX=0;
      ref_s->angleAccY=0;
      Serial.println("reference angle removed");
    return 0;

    case 3: //zero-in at starting place set adjust values
      set_angle_reference(gs,adj_s);
      Serial.println("adjust angle set");
    return 0;

    case 4: //remove zero-in values at starting place  values
      adj_s->acc_X=0;
      adj_s->acc_Y=0;
      adj_s->acc_Z=0;
      adj_s->gyro_X=0;
      adj_s->gyro_Y=0;
      adj_s->gyro_Z=0;
      adj_s->angleAccX=0;
      adj_s->angleAccY=0;
      Serial.println("adjust angle removed");
    return 0;

    default: //some error occured
      Serial.println("error while reading orders form master");
    return 1;

  }

}

