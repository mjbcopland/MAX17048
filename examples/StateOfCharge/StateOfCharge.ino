#include <MAX17048.h>

void setup() {
  Serial.begin(9600);

  // Try to open the MAX17048
  if (Gauge.open()) {
    Serial.println("Device detected!");

    // Load the default compensation value
    Gauge.compensation(Gauge.RCOMP0);
  } else {
    Serial.println("Device not detected!");
    exit(EXIT_FAILURE);
  }
}

void loop() {
  // Print the current state of charge
  Serial.print("SOC = ");
  Serial.print((float)Gauge);
  Serial.println("%");

  // Sleep for 0.5 seconds
  delay(500);
}
