#include <Arduino.h>
#include <SoftwareSerial.h>

// Define pins for sensor 1
#define TRIG_PIN_1 9
#define ECHO_PIN_1 10

// Define pins for sensor 2
#define TRIG_PIN_2 11
#define ECHO_PIN_2 12

// Define pins for relay channels
#define RELAY_PIN_1 4
#define RELAY_PIN_2 5

#define SensorPin1 A0 // pH meter Analog output to Arduino Analog Input 0
#define SensorPin2 A1 // pH meter Analog output to Arduino Analog Input 1
#define SensorPin3 A5 // pH meter Analog output to Arduino Analog Input 5

float calibration_value1 = 21.34 + 2.5; // Calibration value for Sensor 1
float calibration_value2 = 21.34 + 3.1; // Calibration value for Sensor 2
float calibration_value3 = 21.34 + 3.8; // Calibration value for Sensor 3

#define analogInPinTDS A3 // Analog input pin untuk tds

// variable
int sensorValue0; // adc value
float calibration_value0 = 1203.08;

SoftwareSerial mySerial(2, 3); // RX, TX , Software serial from ESP-01 to Arduino

int detikGerbang = 0;

bool bukaGerbang = false;
bool tutupGerbang = false;

String statusGerbang = "idle";

float tdsReading(int sensorPin, float calibration_value)
{
  int sensorValue = analogRead(sensorPin);
  float outputTds = (0.3417 * sensorValue) + calibration_value;
  return outputTds;
}

float processPHsensor(int pin, float calibration_value)
{
  int buf[10];
  int temp;
  unsigned long int avgValue;

  // Read and smooth the sensor data
  for (int i = 0; i < 10; i++)
  {
    buf[i] = analogRead(pin);
    delay(10);
  }

  // Sort the values
  for (int i = 0; i < 9; i++)
  {
    for (int j = i + 1; j < 10; j++)
    {
      if (buf[i] > buf[j])
      {
        temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }

  // Calculate the average of the middle 6 values
  avgValue = 0;
  for (int i = 2; i < 8; i++)
    avgValue += buf[i];

  // Convert the average value to pH
  float phValue = (float)avgValue * 5.0 / 1024 / 6;
  phValue = -5.70 * phValue + calibration_value;

  return phValue;
}

long measureDistance(int trigPin, int echoPin)
{
  // Clear the trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a 10 microsecond pulse to the trigger
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the pulse duration
  return pulseIn(echoPin, HIGH);
}

void setup()
{
  Serial.begin(115200);
  mySerial.begin(115200);

  // built in led
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // initialize ph sensor
  pinMode(SensorPin1, INPUT);
  pinMode(SensorPin2, INPUT);
  pinMode(SensorPin3, INPUT);

  // Initialize sensor 1
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);

  // Initialize sensor 2
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);

  // Initialize relay pins
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);

  // Ensure relays are off at the start
  digitalWrite(RELAY_PIN_1, LOW);
  digitalWrite(RELAY_PIN_2, LOW);

  // delay(1000);
}

void loop()
{
  String IncomingStr = "";
  bool flag = false;

  // Check if data is available from ESP-01
  while (mySerial.available())
  {
    IncomingStr = mySerial.readString();
    flag = true;
  }

  Serial.println("Ini flag : " + String(flag));

  if (flag)
  {
    String response = IncomingStr;
    // Serial.println(response); // Print to Serial monitor for debugging
    // mySerial.print("aran"); // Send response back to ESP-01
    // mySerial.print(i);
    // mySerial.print ("\n");
    int commaIndex1 = response.indexOf(',');
    String wifi = response.substring(0, commaIndex1);
    String gerbang = response.substring(commaIndex1 + 1);
    Serial.println(wifi);
    Serial.println(gerbang);

    if (wifi == "Wifi connected")
    {
      Serial.println("Wifi Connected");
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
      Serial.println("Wifi Not Connected");
      digitalWrite(LED_BUILTIN, LOW);
    }

     // Assuming `gerbang` is the received command
    if (gerbang == "buka" && statusGerbang != "buka") {
        bukaGerbang = true;
    } else if (gerbang == "tutup" && statusGerbang != "tutup") {
        tutupGerbang = true;
    }

    // Handle the "buka" command
    if (bukaGerbang == true) {
        if (detikGerbang < 2) { // Open for 5 seconds
            detikGerbang++;
            digitalWrite(RELAY_PIN_1, HIGH); // Open the gate
            digitalWrite(RELAY_PIN_2, LOW);  // Ensure the other relay is off
        } else { // After 5 seconds
            bukaGerbang = false;
            statusGerbang = "buka"; // Update the status
            detikGerbang = 0;
            digitalWrite(RELAY_PIN_1, LOW); // Close the gate
            digitalWrite(RELAY_PIN_2, LOW); // Ensure both relays are off
        }
    }

    // Handle the "tutup" command
    if (tutupGerbang == true) {
        if (detikGerbang < 2) { // Close for 5 seconds
            detikGerbang++;
            digitalWrite(RELAY_PIN_1, LOW); // Ensure the other relay is off
            digitalWrite(RELAY_PIN_2, HIGH); // Close the gate
        } else { // After 5 seconds
            tutupGerbang = false;
            statusGerbang = "tutup"; // Update the status
            detikGerbang = 0;
            digitalWrite(RELAY_PIN_1, LOW); // Ensure both relays are off
            digitalWrite(RELAY_PIN_2, LOW); // Ensure both relays are off
        }
    }

    // Optionally, print the status
    Serial.println("Gate Status: " + statusGerbang);
  }

  // Measure distance for sensor 1
  long duration1 = measureDistance(TRIG_PIN_1, ECHO_PIN_1);
  float distance1 = (duration1 / 2.0) * 0.0344;

  // Measure distance for sensor 2
  long duration2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);
  float distance2 = (duration2 / 2.0) * 0.0344;

  float phValue1 = processPHsensor(SensorPin1, calibration_value1);
  float phValue2 = processPHsensor(SensorPin2, calibration_value2);
  float phValue3 = processPHsensor(SensorPin3, calibration_value3);
  // float phValue2 = processPHsensor(SensorPin2, calibration_value2);

  //
  float tdsValue1 = tdsReading(analogInPinTDS, calibration_value0);

  // Print the pH values to the serial monitor

  Serial.print("Distance1: ");
  Serial.print(distance1);
  Serial.print("cm ,Distance2: ");
  Serial.print(distance2);
  Serial.println("cm ");

  Serial.print("pH Value 1: ");
  Serial.println(phValue1);

  Serial.print("pH Value 2: ");
  Serial.println(phValue2);

  Serial.print("pH Value 3: ");
  Serial.println(phValue3);

  Serial.print("TDS Value 1: ");
  Serial.println(tdsValue1);

  // if (distance1 <= 30)
  // {
  //   digitalWrite(RELAY_PIN_1, HIGH);
  //   digitalWrite(RELAY_PIN_2, LOW);
  // }
  // else if (distance1 >= 60)
  // {
  //   digitalWrite(RELAY_PIN_2, HIGH);
  //   digitalWrite(RELAY_PIN_1, LOW);
  // }
  // else
  // {
  //   digitalWrite(RELAY_PIN_1, LOW);
  //   digitalWrite(RELAY_PIN_2, LOW);
  // }

  mySerial.print(statusGerbang);
  mySerial.print(",");
  mySerial.print(distance1);
  mySerial.print(",");
  mySerial.print(distance2);
  mySerial.print(",");
  mySerial.print(phValue1);
  mySerial.print(",");
  mySerial.print(phValue2);
  mySerial.print(",");
  mySerial.print(phValue3);
  mySerial.print(",");
  mySerial.print(tdsValue1);
  mySerial.print("\n");

  delay(2000);
}
