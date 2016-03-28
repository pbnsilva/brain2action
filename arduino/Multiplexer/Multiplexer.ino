/**
 * TCA9548 I2CScanner.pde -- I2C bus scanner for Arduino
 *
 * Based on code c. 2009, Tod E. Kurt, http://todbot.com/blog/
 *
 */
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
extern "C" { 
  #include "utility/twi.h"  // from Wire library, so we can do bus scanning
}
 
#define TCAADDR 0x70
 
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


int numIMUs = 5; 
MPU6050 imu[8];
uint16_t packetSize; //expected DMP packet size (defult 42) -- ?
 
// standard Arduino setup()
void setup()
{
    while (!Serial);
    delay(1000);
 
    Wire.begin();
    
    Serial.begin(115200);
    for (uint8_t t=0; t<numIMUs; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);
  
      Serial.println("Initializing I2C devices...");
      imu[t].initialize();

      
      // verify connection
      Serial.println("Testing device connections...");
      Serial.println(imu[t].testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

      //load and configure DMP 
      Serial.println("initializing DMP"); 
      uint8_t deviceStatus = imu[t].dmpInitialize(); //use MPU6050 library to inilisalize the dmp 

      //make sure it works 
      if (deviceStatus == 0) {
        Serial.println("DMP initialization success, now enable DMP for use");
        //turn on DMP 
        imu[t].setDMPEnabled(true); //use MPU6050 library to enable DMP)

        Serial.println("DMP is ready to use.");

        //get expected DMP packet size for later comparison 
        packetSize = imu[t].dmpGetFIFOPacketSize();
        Serial.print("Packet size: "); Serial.println(packetSize); 
      } else {
        //ERROR! , device status !=0 when initializing DMP
        Serial.print("DMP initialization failed when using MPU6050 library:");
        if (deviceStatus == 1) {
          Serial.println(" intial memory load failed");
        } else if (deviceStatus == 2) {
          Serial.println(" failed to update DMP configuration");
        } else {
          Serial.print(" unknow error with code: ");
          Serial.println(deviceStatus);
        }
      }
    }
    //feed offsets 
    imu[0].setXGyroOffset(76);
    imu[0].setYGyroOffset(-24);
    imu[0].setZGyroOffset(11);
    imu[0].setXAccelOffset(-290); // 1688 factory default for my test chip
    imu[0].setYAccelOffset(-440); // 1688 factory default for my test chip
    imu[0].setZAccelOffset(1809); // 1688 factory default for my test chip
    
    imu[1].setXGyroOffset(116);
    imu[1].setYGyroOffset(-17);
    imu[1].setZGyroOffset(19);
    imu[1].setXAccelOffset(-1998); // 1688 factory default for my test chip
    imu[1].setYAccelOffset(-770); // 1688 factory default for my test chip
    imu[1].setZAccelOffset(1943); // 1688 factory default for my test chip
      
    imu[2].setXGyroOffset(41);
    imu[2].setYGyroOffset(0);
    imu[2].setZGyroOffset(15);
    imu[2].setXAccelOffset(389); // 1688 factory default for my test chip
    imu[2].setYAccelOffset(1906); // 1688 factory default for my test chip
    imu[2].setZAccelOffset(1816); // 1688 factory default for my test chip
      
    imu[3].setXGyroOffset(20);
    imu[3].setYGyroOffset(-25);
    imu[3].setZGyroOffset(26);
    imu[3].setXAccelOffset(-3643); // 1688 factory default for my test chip
    imu[3].setYAccelOffset(185); // 1688 factory default for my test chip
    imu[3].setZAccelOffset(1688); // 1688 factory default for my test chip

    imu[4].setXGyroOffset(59);
    imu[4].setYGyroOffset(-12);
    imu[4].setZGyroOffset(-6);
    imu[4].setXAccelOffset(-1642); // 1688 factory default for my test chip
    imu[4].setYAccelOffset(-325); // 1688 factory default for my test chip
    imu[4].setZAccelOffset(1799); // 1688 factory default for my test chip
    
    Serial.println("Arduino IMU sever ready.");
}

uint16_t fifoCount[8]; //count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO buffer storage 
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 aa;         // [x, y, z]            accel sensor measurements

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

void loop() 
{
  for (uint8_t t=0; t<numIMUs; t++) {
    tcaselect(t);
    imu[t].resetFIFO();
    fifoCount[t] = imu[t].getFIFOCount();
    while (fifoCount[t] < packetSize) {
        //waiting until get enough
        fifoCount[t] = imu[t].getFIFOCount();
    }
    
    //read this packet from FIFO buffer 
    imu[t].getFIFOBytes(fifoBuffer,packetSize);

    //display stage 
    imu[t].dmpGetQuaternion(&q, fifoBuffer);
//    //Tab delimited
//    Serial.print("IMU #"); 
//    Serial.print(t);
//    Serial.print("\tquat\t");
//    Serial.print(q.w);
//    Serial.print("\t");
//    Serial.print(q.x);
//    Serial.print("\t");
//    Serial.print(q.y);
//    Serial.print("\t");
//    Serial.print(q.z);
//    Serial.print("\t");
    //csv delimited
    Serial.print(q.w);
    Serial.print(",");
    Serial.print(q.x);
    Serial.print(",");
    Serial.print(q.y);
    Serial.print(",");
    Serial.print(q.z);
    Serial.print(",");
//    imu[t].dmpGetAccel(&aa, fifoBuffer);
//    Serial.print("\taa\t");
//    Serial.print(aa.x);
//    Serial.print("\t");
//    Serial.print(aa.y);
//    Serial.print("\t");
//    Serial.println(aa.z);
          
//      imu[t].dmpGetGravity(&gravity, &q);
//      imu[t].dmpGetYawPitchRoll(ypr, &q, &gravity);
//      Serial.print("\typr\t");
//      Serial.print(ypr[0] * 180/M_PI);
//      Serial.print("\t");
//      Serial.print(ypr[1] * 180/M_PI);
//      Serial.print("\t");
//      Serial.print(ypr[2] * 180/M_PI);
//      Serial.print("\t");
//      imu[t].dmpGetEuler(euler, &q);
//      Serial.print("euler\t");
//      Serial.print(euler[0] * 180/M_PI);
//      Serial.print("\t");
//      Serial.print(euler[1] * 180/M_PI);
//      Serial.print("\t");
//      Serial.println(euler[2] * 180/M_PI);
        // display quaternion values in InvenSense Teapot demo format:
//        teapotPacket[2] = fifoBuffer[0];
//        teapotPacket[3] = fifoBuffer[1];
//        teapotPacket[4] = fifoBuffer[4];
//        teapotPacket[5] = fifoBuffer[5];
//        teapotPacket[6] = fifoBuffer[8];
//        teapotPacket[7] = fifoBuffer[9];
//        teapotPacket[8] = fifoBuffer[12];
//        teapotPacket[9] = fifoBuffer[13];
//        Serial.write(teapotPacket, 14);
//        teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
  } 
  Serial.println("");
  delay(100);
}
