#define ENCODER_PIN           2
#define WHEEL_RADIUS          0.05
#define DISTANCE_PER_COUNT    2 * PI * WHEEL_RADIUS

// Number of samples to use for the moving average
#define NUMSAMPLES            10

// Define the timeout duration in milliseconds for motor stop detection (adjust as needed)
#define MOTOR_STOP_TIMEOUT    1000

class MotorEncoder {
  private:
    unsigned long prevPulseTime = 0; // Record the time when the last pulse was detected
    unsigned long prevTime = 0;
    int prevEncoderState = LOW;      // Declare prevEncoderState and initialize it to LOW

    //Moving average filter
    double x_velSum = 0.0;
    double x_velSamples[NUMSAMPLES];
    int x_velIndex = 0;

  public:
    double x_vel = 0.0;

    void init();
    void run();
    void calculateVelocity();
};

void MotorEncoder::init() {
  pinMode(ENCODER_PIN, INPUT_PULLUP);    // Set encoder pin as input with pull-up resistor
  prevTime = millis();                  // Initialize prevTime

  // Moving average filter
  // Initialize the x_velSamples array
  for (int i = 0; i < NUMSAMPLES; i++) {
    x_velSamples[i] = 0.0;
  }
}

void MotorEncoder::run() {
  // Check for encoder pulses and calculate linear velocity based on time between pulses
  calculateVelocity();
  // After calculating x_vel, print it to the Serial Monitor
  //SERIAL_PORT.print("Linear Velocity (x_vel): ");
  SERIAL_PORT.println(x_vel);
  //SERIAL_PORT.println(" m/s");
}

void MotorEncoder::calculateVelocity() {
  // Read the current state of the encoder pin
  int encoderState = digitalRead(ENCODER_PIN);

  // Detect a rising edge (low to high transition)
  if (encoderState == HIGH && prevEncoderState == LOW) {
    // Calculate the time elapsed since the last pulse
    unsigned long pulseTime = millis() - prevPulseTime;

    // Calculate linear velocity (distance traveled per second) based on time between pulses
    x_vel = DISTANCE_PER_COUNT / pulseTime * 1000.0; // Convert to m/s
    //SERIAL_PORT.print("Case 1");
    //SERIAL_PORT.print("\n");
    // Record the time when the pulse was detected
    prevPulseTime = millis();
  }
  else if (encoderState == LOW && prevEncoderState == HIGH) {
    // This is the falling edge, which may indicate the last pulse.
    //SERIAL_PORT.print("Case 2");
    //SERIAL_PORT.print("\n");
    // Update prevTime for the next interval
    prevTime = millis();
  }
  else {
    //This case can be either HIGH-->HIGH or LOW-->LOW
    // Calculate the time elapsed since the last reading
    unsigned long deltaTime = millis() - prevTime;
    if (deltaTime >= MOTOR_STOP_TIMEOUT) {
      x_vel = 0.0;
    }
    //SERIAL_PORT.print("Case 3");
    //SERIAL_PORT.print("\n");
  }
  // Update the previous encoder state
  prevEncoderState = encoderState;

  // Update the moving average filter
  x_velSum -= x_velSamples[x_velIndex]; // Subtract the oldest value
  x_velSamples[x_velIndex] = x_vel;     // Store the current value
  x_velSum += x_vel;                     // Add the new value
  x_velIndex = (x_velIndex + 1) % NUMSAMPLES;

  // Calculate the moving average
  x_vel = x_velSum / NUMSAMPLES;
}