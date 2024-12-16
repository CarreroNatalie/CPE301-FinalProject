// Natalie Carrero
// CPE 301 - Final Project

// NO LIBRARY FUNCTION (pinMode) -> USE PREDEFINED MACROS
#include <LiquidCrystal.h> // LCD library
#include <DHT.h> // DHT sensor library (temp and humidity)
#include <Stepper.h> // stepper motor library
#include <Wire.h>
#include <RTClib.h>

// water
#define WATER_SENSOR_PIN A0
#define WATER_THRESHOLD 300

// LCD Pin Configuration using predefined macros
#define RS 12
#define EN 11
#define D4 5
#define D5 4
#define D6 3
#define D7 2

// DHT Pin Configuration
#define DHTPIN 7 // pin 7 for temp & hum.
#define DHTTYPE DHT11

// motor control pins
int controlPin = 13;

// button pin Configuration
#define ONOFFBUTTON 6
volatile bool buttonPressed = false;

// Reset Button
#define RESETBUTTON 10
volatile bool resetButtonPressed = false;
int lastResetButtonState = HIGH;
int currentResetButtonState = HIGH;

// Stepper Motor Pins
#define motorPin1 22
#define motorPin2 24
#define motorPin3 26
#define motorPin4 28
#define motorButton 8

// LED 
#define BLUE_LED 40
#define RED_LED 38
#define YELLOW_LED 36 
#define GREEN_LED 34

// initialize rtc
RTC_DS3231 rtc;

// states
enum State {DISABLED, IDLE, ERROR, RUNNING};
State currentState = DISABLED;

// stepper motor state
bool motorState = false;
bool lastmotorButtonState = HIGH;
bool currentmotorButtonState = HIGH;

// DHT object
DHT dht(DHTPIN, DHTTYPE);

// initialize LCD
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// FAN states
bool buttonState = false;
int lastButtonState = HIGH;

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2); // initialize LCD
  dht.begin(); // initialize DHT sensor

  // Fan as output and start off
  DDRB |= (1 << PB5); // Set PB5 (pin 13) as output (controlPin)
  changeFanState(0); // set to off

  // button as input
  DDRD &= ~(1 << PD6); // Set PD6 (pin 6) as input (ONOFFBUTTON)
  PORTD |= (1 << PD6);  // Enable pull-up resistor
  attachInterrupt(digitalPinToInterrupt(ONOFFBUTTON), startButtonISR, FALLING);

  DDRB &= ~(1 << PB2); // Set pin 10 (RESETBUTTON) as input
  PORTB |= (1 << PB2); // Enable pull-up resistor

  // set up stepper motor control
  DDRF |= (1 << PF4) | (1 << PF5) | (1 << PF6) | (1 << PF7);  // Stepper motor pins as output

  // set motor button 
  DDRD &= ~(1 << PD0); // Set motorButton (pin 8) as input
  PORTD |= (1 << PD0); // Enable pull-up resistor

  // LEDS
  DDRC |= (1 << PC4) | (1 << PC5) | (1 << PC6) | (1 << PC7);  // LED pins as output

  // Initialize RTC
  rtc.begin();

  // set initial state
  currentState = DISABLED;

  delay(2000); 
}

void loop() {

  buttonControl();
  stepperMotorControl();

  handleResetButton();

  switch (currentState){
    case DISABLED:
      handleDisabledState();
      changeFanState(0);
      break;
    case IDLE:
      handleIdleState();
      changeFanState(0);
      break;
    case ERROR:
      handleErrorState();
      changeFanState(0);
      break;
    case RUNNING:
      handleRunningState();
      changeFanState(1);
      break;
  }

  delay(1000); // update every second
}

void handleResetButton(){
  currentResetButtonState = PINB & (1 << PB2) ? HIGH : LOW;
  if (currentResetButtonState == LOW && lastResetButtonState == HIGH) {
    resetButtonPressed = true;
    delay(200);  // Add a small delay to debounce the button
  }

  lastResetButtonState = currentResetButtonState;
}

void handleDisabledState(){
  // turn on YELLOW LED
  PORTC |= (1 << PC5); // Yellow LED ON
  PORTC &= ~(1 << PC4); // Red LED OFF
  PORTC &= ~(1 << PC6); // Blue LED OFF
  PORTC &= ~(1 << PC7); // Green LED OFF

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DISABLED");

  if (buttonPressed) {
    buttonPressed = false; 
    if (currentState == DISABLED){
      currentState = IDLE;
    }
  }
}

void handleIdleState(){
  PORTC &= ~(1 << PC5); // Yellow LED OFF
  PORTC &= ~(1 << PC4); // Red LED OFF
  PORTC &= ~(1 << PC6); // Blue LED OFF
  PORTC |= (1 << PC7);  // Green LED ON

  // Monitor water level
  watercheck();

  // Monitor temperature and humidity
  tempandhum();
  
  if(dht.readTemperature() > 22.0){
    clockreport();
    Serial.println("State changed from IDLE to RUNNING");
    currentState = RUNNING;
  }
}

void handleErrorState(){
  PORTC |= (1 << PC4);  // Red LED ON
  PORTC &= ~(1 << PC7);  // Turn off Green LED
  PORTC &= ~(1 << PC6);  // Turn off Blue LED
  PORTC &= ~(1 << PC5);  // Turn off Yellow LED
  
  // Display error on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ERROR: low water");
  
  // Stop the motor if it's running
  stopMotor();

  // Wait for reset button to transition back to IDLE if water is above threshold
  watercheck();
}

void handleRunningState(){
  PORTC |= (1 << PC6);  // Blue LED ON
  PORTC &= ~(1 << PC7);  // Turn off Green LED
  PORTC &= ~(1 << PC4);  // Turn off Red LED
  PORTC &= ~(1 << PC5);  // Turn off Yellow LED

  // Monitor temperature and humidity
  tempandhum();
  
  // Monitor water level
  watercheck();
  
  if (dht.readTemperature() < 21.0) {
    clockreport();
    Serial.println("State changed from RUNNING to IDLE");
    currentState = IDLE;
  }
}

void resetButtonISR(){
  resetButtonPressed = true;
}

void startButtonISR(){
  buttonPressed = true;
}

void clockreport(){
  DateTime now = rtc.now(); // current time
  delay(1000);
  Serial.print("Time: ");
  Serial.print(now.timestamp(DateTime::TIMESTAMP_FULL));
}

void stepperMotorControl(){
  currentmotorButtonState = PIND & (1 << PD0) ? HIGH : LOW;

  if (lastmotorButtonState == HIGH && currentmotorButtonState == LOW){
    motorState = !motorState;
    if (motorState) {
      startMotor();
    } else {
      stopMotor();
    }

    delay(200);  // Small delay to debounce button
  }
  lastmotorButtonState = currentmotorButtonState;

  // If motor is on, run the stepper motor
  if (motorState) {
    runMotor();
  }

  delay(100);  // Small delay for stability
}

void runMotor() {
  // Simple step sequence for the 28BYJ-48 stepper motor
  PORTF |= (1 << PF4); // Motor step sequence
  delay(5); 

  PORTF |= (1 << PF4) | (1 << PF5);
  delay(5);

  PORTF &= ~(1 << PF4);
  PORTF |= (1 << PF5);
  delay(5);

  PORTF &= ~(1 << PF5);
  PORTF |= (1 << PF6);
  delay(5);

  PORTF &= ~(1 << PF6);
  PORTF |= (1 << PF7);
  delay(5);

  PORTF &= ~(1 << PF7);
  PORTF |= (1 << PF4);
  delay(5);
}

void startMotor() {
  // The motor will start running when the button is pressed
}

void stopMotor() {
  // Set all motor control pins to LOW to stop the motor
  PORTF &= ~((1 << PF4) | (1 << PF5) | (1 << PF6) | (1 << PF7));
}

void watercheck(){
  int waterlevel = analogRead(WATER_SENSOR_PIN);
  if (waterlevel < WATER_THRESHOLD){
    clockreport();
    Serial.println("State changed from IDLE to ERROR");
    currentState = ERROR;
  } else if (currentState == ERROR && waterlevel >= WATER_THRESHOLD){
    if(resetButtonPressed){
      clockreport();
      Serial.println("State changed from ERROR to IDLE");
      currentState = IDLE;
      resetButtonPressed = false;
    }
  }
}

void tempandhum(){
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Display the temperature and humidity on the LCD
  lcd.setCursor(0, 0);  // Set the cursor to the first row
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" C   ");

  lcd.setCursor(0, 1);  // Set the cursor to the second row
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print(" %");
}

void changeFanState(int state){
  if(state == 0){
    PORTB &= ~(1 << PB5); // Fan Off
  }else{
    PORTB |= (1 << PB5); // Fan On
  }
}

void buttonControl(){
  buttonState = PIND & (1 << PD6) ? HIGH : LOW;
  if (buttonState == LOW && lastButtonState == HIGH){
    buttonState = !buttonState;
    if (currentState == DISABLED) {
      clockreport();
      Serial.println("State changed from DISABLED to IDLE");
      currentState = IDLE;
    } else if (currentState != DISABLED){
      clockreport();
      Serial.println("State changed to DISABLED");
      currentState = DISABLED;
    }
    delay(200);  // Change this delay if needed for better response
  }

  lastButtonState = buttonState;
}
