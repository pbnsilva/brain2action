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


MPU6050 imu;
uint16_t packetSize; //expected DMP packet size (defult 42) -- ?

 int16_t ax, ay, az;
int16_t gx, gy, gz;

// standard Arduino setup()
void setup()
{
    while (!Serial);
    delay(1000);
 
    Wire.begin();
    
    Serial.begin(115200);

    //Wire.beginTransmission(TCAADDR);
    //Wire.write(1 << i);
    //Wire.endTransmission(); 
      
      Serial.println("Initializing I2C devices...");
      imu.initialize();
      
      // verify connection
      Serial.println("Testing device connections...");
      Serial.println(imu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

      //load and configure DMP 
      Serial.println("initializing DMP"); 
      uint8_t deviceStatus = imu.dmpInitialize(); //use MPU6050 library to inilisalize the dmp 

      //make sure it works 
      if (deviceStatus == 0) {
        Serial.println("DMP initialization success, now enable DMP for use");
        //turn on DMP 
        imu.setDMPEnabled(true); //use MPU6050 library to enable DMP)

        Serial.println("DMP is ready to use.");

        //get expected DMP packet size for later comparison 
        packetSize = imu.dmpGetFIFOPacketSize();
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
   
    //feed offsets 
    imu.setXGyroOffset(76);
    imu.setYGyroOffset(-24);
    imu.setZGyroOffset(11);
    imu.setXAccelOffset(-290); // 1688 factory default for my test chip
    imu.setYAccelOffset(-440); // 1688 factory default for my test chip
    imu.setZAccelOffset(1809); // 1688 factory default for my test chip
    
    Serial.println("Arduino IMU sever ready.");
}

uint16_t fifoCount; //count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO buffer storage 
Quaternion q;           // [w, x, y, z]         quaternion container

void loop() 
{
    imu.resetFIFO();
    fifoCount = imu.getFIFOCount();
    while (fifoCount < packetSize) {
        //waiting until get enough
        fifoCount = imu.getFIFOCount();
    }
    
    //read this packet from FIFO buffer 
    imu.getFIFOBytes(fifoBuffer,packetSize);

    //display stage 
    imu.dmpGetQuaternion(&q, fifoBuffer);
    //csv delimited
    Serial.print(q.w);
    Serial.print(",");
    Serial.print(q.x);
    Serial.print(",");
    Serial.print(q.y);
    Serial.print(",");
    Serial.print(q.z);
    Serial.println("");
    delay(100);
}
