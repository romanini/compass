#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_LSM303_Accel.h>


// Hard-iron calibration settings
const float hard_iron[3] = {
  -8.55,  21.21,  -31.25
};

// Soft-iron calibration settings
const float soft_iron[3][3] = {
  {  0.932,  -0.009, 0.011  },
  {  -0.009,  1.040, -0.033  },
  { 0.011, -0.033,  1.033  }
};

Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);


void setup(void)
{
  Serial.begin(115200);
  Serial.println("Magnetometer Test"); Serial.println("");

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the LIS2MDL ... check your connections */
    Serial.println("Ooops, no LIS2MDL detected ... Check your wiring!");
    while(1);
  }

  /* Initialise the sensor */
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }
  
  accel.setRange(LSM303_RANGE_4G);
  accel.setMode(LSM303_MODE_NORMAL);

}

void loop(void)
{
  static float hi_cal[3];

  /* Get a new sensor event */
  sensors_event_t mag_event;
  mag.getEvent(&mag_event);

  sensors_event_t accel_event;
  accel.getEvent(&accel_event);

  float altitude = (atan(accel_event.acceleration.x / accel_event.acceleration.z));//*180/PI;
  float roll = atan(accel_event.acceleration.y / accel_event.acceleration.z);

   
  // Put raw magnetometer readings into an array
  float mag_data[] = {mag_event.magnetic.x,
                      mag_event.magnetic.y,
                      mag_event.magnetic.z};

  // Apply hard-iron offsets
  for (uint8_t i = 0; i < 3; i++) {
    hi_cal[i] = mag_data[i] - hard_iron[i];
  }

  // Apply soft-iron scaling
  for (uint8_t i = 0; i < 3; i++) {
    mag_data[i] = (soft_iron[i][0] * hi_cal[0]) +
                  (soft_iron[i][1] * hi_cal[1]) +
                  (soft_iron[i][2] * hi_cal[2]);
  }

  float xh = mag_data[0] * cos(altitude)+mag_data[2]*sin(altitude);
  float yh = mag_data[0] * sin(roll) * sin(altitude) + mag_data[1] * cos(roll) - mag_data[2] * sin(roll) * cos(altitude);
  // Calculate the angle of the vector y,x
  float heading = (atan2(yh, xh) * 180) / PI;
  
  // Normalize to 0-360
  if (heading < 0){
    heading = 360 + heading;
  }

  Serial.print("X: ");
  Serial.print(accel_event.acceleration.x);
  Serial.print("  ");
  Serial.print("Y: ");
  Serial.print(accel_event.acceleration.y);
  Serial.print("  ");
  Serial.print("Z: ");
  Serial.print(accel_event.acceleration.z);
  Serial.print("  ");
  Serial.println("m/s^2");
  Serial.print("Headig: ");
  Serial.println(heading);
  
  delay(500);
}
