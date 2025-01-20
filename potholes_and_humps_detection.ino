#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// Define Ultrasonic Sensor pins
#define TRIG1 7
#define ECHO1 8
#define TRIG2 9
#define ECHO2 10

// Define DHT22 pin
#define DHTPIN 2
#define DHTTYPE DHT22

// Set up the DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// Set up the LCD screen (I2C address is usually 0x27 or 0x3F, depending on your LCD model)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define motor control pins for L298N
#define IN1 3
#define IN2 4
#define IN3 5
#define IN4 6

// Variables for ultrasonic sensors
long duration1, distance1;
long duration2, distance2;

// Variables for DHT22
float temperature, humidity;

// Bluetooth command variable
String datain;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize ultrasonic sensor pins
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  // Initialize DHT22 sensor
  dht.begin();

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();

  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  // Read Bluetooth data for movement control
  if (Serial.available()) {
    while (Serial.available() > 0) {
      char c = Serial.read();  // Read the incoming Bluetooth data
      datain += c;
    }

    // Execute movement based on Bluetooth command
    if (datain == "R") { // Move Forward
      MoveForward();
    }
    else if (datain == "L") { // Move Backward
      MoveBackward();
    }
    else if (datain == "B") { // Turn Left
      turnLeft();
    }
    else if (datain == "F") { // Turn Right
      turnRight();
    }
    else if (datain == "S") { // Stop Motors
      stopMotors();
    }
    datain = ""; // Reset command string after execution
  }

  // Read ultrasonic sensors for potholes and humps
  distance1 = getPotholeDistance(); // Pothole detection
  distance2 = getHumpDistance(); // Hump detection

  // Read temperature and humidity from DHT22
  readDHT();

  // Check if the sensor readings are valid
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
https://www.onlinegdb.com/
  // Display Road Condition (Pothole, Hump, or Normal)
  lcd.setCursor(0, 0);
  if (distance1 > 20) {
    lcd.print("Pothole Detected ");
    Serial.println("Pothole Detected");
  }
  else if (distance2 < 10
  ) {
    lcd.print("Hump Detected ");
    Serial.println("Hump Detected");
  }
  else {
    lcd.print("Normal Road     ");
    Serial.println("Normal Road");
  }

  // Display Temperature and Humidity
  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(temperature);
  lcd.print("C H:");
  lcd.print(humidity);
  lcd.print("%");

  delay(1000); // Delay for stability and to avoid flickering
}

// Function to get distance from pothole ultrasonic sensor
long getPotholeDistance() {
  // Send a 10us pulse to trigger the ultrasonic sensor
  digitalWrite(TRIG1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);

  // Read the duration of the pulse
  duration1 = pulseIn(ECHO1, HIGH);

  // Calculate the distance in cm
  long distance = (duration1 / 2) * 0.0344;
  return distance;
}

// Function to get distance from hump ultrasonic sensor
long getHumpDistance() {
  // Send a 10us pulse to trigger the ultrasonic sensor
  digitalWrite(TRIG2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG2, LOW);

  // Read the duration of the pulse
  duration2 = pulseIn(ECHO2, HIGH);

  // Calculate the distance in cm
  long distance = (duration2 / 2) * 0.0344;
  return distance;
}

// Function to read temperature and humidity from DHT22 sensor
void readDHT() {
  temperature = dht.readTemperature();  // Get temperature in Celsius
  humidity = dht.readHumidity();  // Get humidity percentage
}

// Function to move the car forward
void MoveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Moving Forward");
}

// Function to move the car backward
void MoveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Moving Backward");
}

// Function to turn the car left
void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Turning Left");
}

// Function to turn the car right
void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Turning Right");
}

// Function to stop the motors
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Motors Stopped");
}
