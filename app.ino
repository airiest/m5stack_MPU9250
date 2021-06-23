#include <M5Stack.h>
#include <SparkFunMPU9250-DMP.h>
#include <MadgwickAHRS.h>

MPU9250_DMP imu;
Madgwick filter;

SemaphoreHandle_t xMutexSerial = NULL;
QueueHandle_t xQueueMAD = NULL;
QueueHandle_t xQueueDMP = NULL;

typedef struct st_acc_gyro
{
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
} acc_gyro;

typedef struct st_quat
{
  float w;
  float x;
  float y;
  float z;
} quat;

void setup()
{
  M5.begin();
  M5.Lcd.setCursor(0, 0, 4);
  M5.Lcd.fillScreen(BLACK);

  Serial.begin(115200);

  imu.begin();

  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.setGyroFSR(2000);         // Set gyro to 2000 dps
  imu.setAccelFSR(2);           // Set accel to +/-2g
  imu.setLPF(5);                // Set LPF corner frequency to 5Hz
  imu.setSampleRate(10);        // Set sample rate to 10Hz
  imu.setCompassSampleRate(10); // Set mag rate to 10Hz
  filter.begin(10);             // Set Madgwick filter rate  to 10Hz

  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT |  // Enable 6-axis quat
                   DMP_FEATURE_GYRO_CAL, // Use gyro calibration
               10);

  xMutexSerial = xSemaphoreCreateMutex();
  xQueueDMP = xQueueCreate(1, sizeof(quat));
  xQueueMAD = xQueueCreate(1, sizeof(acc_gyro));

  if ((xMutexSerial != NULL) && (xQueueDMP != NULL) && (xQueueMAD != NULL))
  {
    xTaskCreatePinnedToCore(mad_func, "mad_func", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(dmp_func, "dmp_func", 4096, NULL, 1, NULL, 0);
  }
}

void mad_func(void *arg)
{
  xSemaphoreGive(xMutexSerial);
  acc_gyro ag;

  while (1)
  {
    xQueueReceive(xQueueMAD, &ag, portMAX_DELAY);

    if (xSemaphoreTake(xMutexSerial, portMAX_DELAY) == pdTRUE)
    {
      filter.updateIMU(ag.gx, ag.gy, ag.gz, ag.ax, ag.ay, ag.az);

      float roll = filter.getRoll();
      float pitch = filter.getPitch();
      float yaw = filter.getYaw();

      M5.Lcd.setCursor(0, 0);
      M5.Lcd.printf("Madgwick Euler angles");
      M5.Lcd.setCursor(0, 40);
      M5.Lcd.printf(" %5.2f  %5.2f  %5.2f     ", roll, pitch, yaw);
      Serial.printf("%5.2f, %5.2f, %5.2f\r\n", roll, pitch, yaw);
    }
    xSemaphoreGive(xMutexSerial);

    delay(100);
  }
}

void dmp_func(void *arg)
{
  xSemaphoreGive(xMutexSerial);
  quat q;

  while (1)
  {
    xQueueReceive(xQueueDMP, &q, portMAX_DELAY);

    if (xSemaphoreTake(xMutexSerial, portMAX_DELAY) == pdTRUE)
    {
      M5.Lcd.setCursor(0, 80);
      M5.Lcd.printf("DMP Quaternion");
      M5.Lcd.setCursor(0, 120);
      M5.Lcd.printf(" %3.2f  %3.2f  %3.2f  %3.2f     ", q.w, q.x, q.y, q.z);
      Serial.printf("%3.2f, %3.2f, %3.2f, %3.2f\r\n", q.w, q.x, q.y, q.z);
    }
    xSemaphoreGive(xMutexSerial);
  }
}

void loop()
{
  acc_gyro ag;
  quat q;

  if (imu.dataReady())
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    float accelX = imu.calcAccel(imu.ax);
    float accelY = imu.calcAccel(imu.ay);
    float accelZ = imu.calcAccel(imu.az);
    float gyroX = imu.calcGyro(imu.gx);
    float gyroY = imu.calcGyro(imu.gy);
    float gyroZ = imu.calcGyro(imu.gz);
    float magX = imu.calcMag(imu.mx);
    float magY = imu.calcMag(imu.my);
    float magZ = imu.calcMag(imu.mz);

    ag.ax = accelX;
    ag.ay = accelY;
    ag.az = accelZ;
    ag.gx = gyroX;
    ag.gy = gyroY;
    ag.gz = gyroZ;

    xQueueSend(xQueueMAD, &ag, 0);
  }

  if (imu.fifoAvailable())
  {
    if (imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      imu.computeEulerAngles();
      q.w = imu.calcQuat(imu.qw);
      q.x = imu.calcQuat(imu.qx);
      q.y = imu.calcQuat(imu.qy);
      q.z = imu.calcQuat(imu.qz);

      xQueueSend(xQueueDMP, &q, 0);
    }
  }
}