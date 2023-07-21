#define BLYNK_TEMPLATE_ID "TMPL2axjqWFxU"
#define BLYNK_TEMPLATE_NAME "IoT Detector de caídas"
#define BLYNK_AUTH_TOKEN "MjEF0RDEi9JCUuOxEBNqwn6VBlOeTMyc"

#define BLYNK_PRINT Serial
#include <WiFiClient.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

char auth[] = BLYNK_AUTH_TOKEN;

char ssid[] = "Cristian S9";
char pass[] = "nomeacuerdo";

BlynkTimer timer;

void sendSensor()
{
  if(mpu.getMotionInterruptStatus()) {
    //Traigo nuevos sensos con las lecturas:
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    //Calcular los angulos de inclinacion:
    float accel_ang_x=atan(a.acceleration.x/sqrt(pow(a.acceleration.y,2) + pow(a.acceleration.z,2)))*(180.0/(3.14));
    float accel_ang_y=atan(a.acceleration.y/sqrt(pow(a.acceleration.x,2) + pow(a.acceleration.z,2)))*(180.0/(3.14));
    float accel_ang_z=atan((sqrt(pow(a.acceleration.x,2)+pow(a.acceleration.y,2)))/a.acceleration.z)*(180.0/(3.14));

    float gyro_ang_x=atan(g.gyro.x/sqrt(pow(g.gyro.y,2) + pow(g.gyro.z,2)))*(180.0/(3.14));
    float gyro_ang_y=atan(g.gyro.y/sqrt(pow(g.gyro.x,2) + pow(g.gyro.x,2)))*(180.0/(3.14));
    float gyro_ang_z=atan((sqrt(pow(g.gyro.x,2)+pow(g.gyro.y,2)))/g.gyro.z)*(180.0/(3.14));
    //Imprimo valores:
    Serial.print("AccelX:");
    Serial.print(accel_ang_x);
    Serial.print("  ");
    Serial.print("AccelY:");
    Serial.print(accel_ang_y);
    Serial.print("  ");
    Serial.print("AccelZ:");
    Serial.print(accel_ang_z);
    Serial.print("  ");
    Serial.print("GyroX:");
    Serial.print(gyro_ang_x);
    Serial.print("  ");
    Serial.print("GyroY:");
    Serial.print(gyro_ang_y);
    Serial.print("  ");
    Serial.print("GyroZ:");
    Serial.print(gyro_ang_z);
    Serial.println("");
    //No enviar mas de 10 valores por segundo!
    Blynk.virtualWrite(V0, accel_ang_x);
    Blynk.virtualWrite(V1, accel_ang_y);
    Blynk.virtualWrite(V2, accel_ang_z);
    Blynk.virtualWrite(V3, gyro_ang_x);
    Blynk.virtualWrite(V4, gyro_ang_y);
    Blynk.virtualWrite(V5, gyro_ang_z);
   delay(100);
  }
}
void setup()
{   
  
   Serial.begin(9600);
    while (!Serial)
    delay(100); 

  Serial.println("Adafruit MPU6050 test!");

  // Inicializo el sensor!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(100);
    }
  }
  Serial.println("MPU6050 Found!");

  //Configuro el detector de movimiento
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true); //lo mantengo corriendo, se apaga cuando reinicio
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);
 
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(100L, sendSensor);
 
  }

void loop()
{
  Blynk.run();
  timer.run();
 }
