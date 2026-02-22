// #include "api/Common.h"
#include <sys/_intsup.h>
#define QUAT_ANIMATION // DMP setup mode; serial debug output is disabled below for micro-ROS.
// #define IMU_SERIAL_DEBUG
#include "ICM_20948.h" 

//#define USE_SPI       
#define SPI_PORT SPI
#define CS_PIN 2
#define WIRE_PORT Wire
#define AD0_VAL 1

#ifdef USE_SPI
  ICM_20948_SPI myICM; 
#else
  ICM_20948_I2C myICM; 
#endif

class IMU {
  public:
    unsigned long previous_time = 0;
    const unsigned long EVENT_TIME = 10;

    bool publish_state = false;
    double yaw = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    double gx = 0.0;
    double gy = 0.0;
    double gz = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    
    // Track if we are currently handling a disconnect to avoid spamming the console
    bool is_reconnecting = false; 

    void init();
    void run();

  private:
    // Helper function to setup DMP settings (reused in init and reconnect)
    bool configureDMP(); 
    // Helper to attempt to revive the sensor
    void attemptReconnect();
};

// --- New Helper Function: Configure DMP ---
// This contains the logic previously in init() to setup the sensor features
bool IMU::configureDMP() {
  bool success = true;

  // Initialize the DMP
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  // Set ODR rates
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); 

  // Enable FIFO and DMP
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  return success;
}

void IMU::init() {
  #ifndef QUAT_ANIMATION
    SERIAL_PORT.println(F("ICM-20948 Example"));
  #endif

  delay(100);

  // Keep the user-wait logic only in init (Startup), not during reconnection
  #ifndef QUAT_ANIMATION
    while (SERIAL_PORT.available()) SERIAL_PORT.read();
    SERIAL_PORT.println(F("Press any key to continue..."));
    while (!SERIAL_PORT.available()); 
  #endif

  #ifdef USE_SPI
    SPI_PORT.begin();
  #else
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
  #endif

  bool initialized = false;
  while (!initialized) {
    #ifdef USE_SPI
      myICM.begin(CS_PIN, SPI_PORT);
    #else
      myICM.begin(WIRE_PORT, AD0_VAL);
    #endif

    if (myICM.status != ICM_20948_Stat_Ok) {
      #ifndef QUAT_ANIMATION
        SERIAL_PORT.println(F("Trying again..."));
      #endif
      delay(500);
    } else {
      initialized = true;
    }
  }

  #ifndef QUAT_ANIMATION
    SERIAL_PORT.println(F("Device connected!"));
  #endif

  if (configureDMP()) {
     #ifndef QUAT_ANIMATION
       SERIAL_PORT.println(F("DMP enabled!"));
     #endif
  } else {
     SERIAL_PORT.println(F("Enable DMP failed!"));
     while (1); 
  }
}

// --- New Helper Function: Attempt Reconnect ---
void IMU::attemptReconnect() {
  // Try to re-establish the low-level connection
  #ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
  #else
    // Sometimes toggling Wire.begin() helps reset the bus if it's hung
    WIRE_PORT.begin(); 
    myICM.begin(WIRE_PORT, AD0_VAL);
  #endif

  // If the low-level connection (WhoAmI check) works, try to re-setup DMP
  if (myICM.status == ICM_20948_Stat_Ok) {
    if (configureDMP()) {
      is_reconnecting = false; // We are back online!
      #ifndef QUAT_ANIMATION
        SERIAL_PORT.println(F("Sensor Reconnected & DMP Configured!"));
      #endif
    }
  }
}

void IMU::run() {
  // If we are currently in a disconnected state, try to reconnect periodically
  if (is_reconnecting) {
    static unsigned long last_reconnect_attempt = 0;
    if (millis() - last_reconnect_attempt > 500) { // Try every 500ms
      last_reconnect_attempt = millis();
      attemptReconnect();
    }
    return; // Don't try to read data while reconnecting
  }

  bool more_data = (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail);
  unsigned long current_time = millis();

  if (more_data || (current_time - previous_time >= EVENT_TIME)) {
    icm_20948_DMP_data_t data;
    myICM.readDMPdataFromFIFO(&data);

    if (!more_data) { previous_time = current_time; }

    // --- CHECK FOR DISCONNECTION HERE ---
    // If status is NOT Ok, NOT MoreData, and NOT NoData, we have an error (e.g., I2C timeout)
    if (myICM.status != ICM_20948_Stat_Ok && 
        myICM.status != ICM_20948_Stat_FIFOMoreDataAvail && 
        myICM.status != ICM_20948_Stat_FIFONoDataAvail) {
        
        is_reconnecting = true;
        publish_state = false;
        #ifndef QUAT_ANIMATION
          SERIAL_PORT.println(F("Connection Lost! Attempting to reconnect..."));
        #endif
        return; 
    }

    if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
      if ((data.header & DMP_header_bitmap_Quat6) > 0) {
        double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; 
        double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; 
        double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; 

        double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
        double q2sqr = q2 * q2;

        // roll (x-axis rotation)
        double t0 = +2.0 * (q0 * q1 + q2 * q3);
        double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
        double roll = atan2(t0, t1) * 180.0 / PI;

        // pitch (y-axis rotation)
        double t2 = +2.0 * (q0 * q2 - q3 * q1);
        t2 = t2 > 1.0 ? 1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;
        double pitch = asin(t2) * 180.0 / PI;

        // yaw (z-axis rotation)
        double t3 = +2.0 * (q0 * q3 + q1 * q2);
        double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
        yaw = atan2(t3, t4);
        qw = q0;
        qx = q1;
        qy = q2;
        qz = q3;

        // Pull raw accel / gyro and convert to SI units for sensor_msgs/Imu.
        myICM.getAGMT();
        gx = (double)myICM.gyrX() * DEG_TO_RAD;                 // deg/s -> rad/s
        gy = (double)myICM.gyrY() * DEG_TO_RAD;                 // deg/s -> rad/s
        gz = (double)myICM.gyrZ() * DEG_TO_RAD;                 // deg/s -> rad/s
        ax = (double)myICM.accX() * 0.001 * 9.80665;            // mg -> m/s^2
        ay = (double)myICM.accY() * 0.001 * 9.80665;            // mg -> m/s^2
        az = (double)myICM.accZ() * 0.001 * 9.80665;            // mg -> m/s^2

        publish_state = true;

        #if defined(QUAT_ANIMATION) && defined(IMU_SERIAL_DEBUG)
          SERIAL_PORT.print(F("Roll:"));
          SERIAL_PORT.print(roll, 1);
          SERIAL_PORT.print(F(" Pitch:"));
          SERIAL_PORT.print(pitch, 1);
          SERIAL_PORT.print(F(" Yaw:"));
          SERIAL_PORT.println(yaw, 1);
        #endif

      } else { publish_state = false; }
    } else { publish_state = false; }
  }
}
