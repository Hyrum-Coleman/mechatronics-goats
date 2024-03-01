// The SHARP distance sensor needs 5v, ground, and an anlog pin each (to tell us voltage of sensor reading)
// Define the pins that the sensors are connected to
const int distPin1 = A4; // Connect the first sensor's output to Arduino's A4 pin
const int distPin2 = A5; // Connect the second sensor's output to Arduino's A5 pin

void setup() {
  Serial2.begin(9600); // Start the serial communication
}

void loop() {
  int distSensorValue1 = analogRead(distPin1); // Read the first sensor output (0 to 1023)
  float voltage1 = distSensorValue1 * (5.0 / 1023.0); // Convert the reading to voltage
  float distance1 = calculateDistance(voltage1); // Convert voltage to distance in cm for the first sensor

  int distSensorValue2 = analogRead(distPin2); // Read the second sensor output (0 to 1023)
  float voltage2 = distSensorValue2 * (5.0 / 1023.0); // Convert the reading to voltage
  float distance2 = calculateDistance(voltage2); // Convert voltage to distance in cm for the second sensor

  Serial2.print("Distance Sensor 1: ");
  Serial2.print(distance1);
  Serial2.println(" cm");
  Serial2.print("Distance Sensor 2: ");
  Serial2.print(distance2);
  Serial2.println(" cm");

  delay(100);
}

// This polynomial fit is from: https://www.youtube.com/watch?v=rdNCIyL6OA8
// We should probably do a log log fit (less clock cycles) like we learned in class but I can't collect the data to do that right now
float calculateDistance(float voltage) {
  float distance = 33.9 - 69.5*voltage + 62.3*pow(voltage, 2) - 25.4*pow(voltage, 3) + 3.83*pow(voltage, 4);
  return distance;
}
