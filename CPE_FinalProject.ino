// Displaying Data
long lastDisplayTime = 0;

// RTC
#include <RTClib.h>
RTC_DS3231 rtc;

// States
bool OFF = true;
bool IDLE = false;
bool ACTIVE = false;
bool ERROR = false;

// States LED pins
int offLED = 22;
int idleLED = 24;
int activeLED = 26;
int errorLED = 28;

// Turns 1 of 4 LEDs depending on current state of system
void state() {
  // Turn all state LEDs off first
  PORTA &= ~((1 << PA0) | (1 << PA2) | (1 << PA4) | (1 << PA6));

  if (OFF) {
    PORTA |= (1 << PA0);
  } else if (IDLE) {
    PORTA |= (1 << PA2);
  } else if (ACTIVE) {
    PORTA |= (1 << PA4);
  } else if (ERROR) {
    PORTA |= (1 << PA6);
  } 
}

// Start button pin (interrupt pin)
int startButtonPin = 2;
// Flag for ISR
volatile bool buttonPressed = false;

// ISR for Start Button
void buttonISR() {
  buttonPressed = true;
}

// Off button pin
int offButtonPin = 38;
// Reset button pin
int resetButtonPin = 36;

// Servo
#include <Servo.h>
Servo servo;
int servoAngle = 0;

// Potentiometer pin for adjusting detection distance
const unsigned char potChannel = 0;

volatile unsigned char* my_ADMUX    = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB   = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA   = (unsigned char*) 0x7A;
volatile unsigned int*  my_ADC_DATA = (unsigned int*)  0x78;
volatile unsigned char* my_DIDR0    = (unsigned char*) 0x7E;

// ADC value read from potentiometer
int potValue = 0;
// Distance threshold used by the ultrasonic sensors
int distanceThreshold = 60;

// Potentiometer initializing 
void adcInit() {
  *my_ADCSRA |= 0b10000000; // Enable ADC
  *my_ADCSRA &= 0b11011111; // Disable ADC trigger
  *my_ADCSRA &= 0b11110111; // Disable ADC interrupt
  *my_ADCSRA |= 0b00000111; // Set presclar to 128

  // B register
  *my_ADCSRB &= 0b11110111; // Reset channel and gain bits
  *my_ADCSRB &= 0b11111000; // Free running mode

  // MUX Register
  *my_ADMUX &= 0b01111111; // Bit 7 to 0 for AVCC analog reference
  *my_ADMUX |= 0b01000000; // Bit 6 to 1 for AVCC analog reference
  *my_ADMUX &= 0b11011111; // Right adjust result
  *my_ADMUX &= 0b11100000; // Reset channel

  *my_DIDR0 |= 0b00000001; // Disable digital input
}

unsigned int adcRead(uint8_t channel) {
  *my_ADMUX &= 0b11100000; // Clear the channel selection bits (MUX 4:0)
  *my_ADCSRB &= 0b11110111; // Clear the selection bits (MUX 5)
  *my_ADMUX |= (channel & 0b00011111);

  *my_ADCSRA |= 0b01000000; // Start conversion
  while ((*my_ADCSRA & 0b01000000) != 0) { }

  return *my_ADC_DATA;
}

// Reads potentiometer and maps it to a usuable distance range
void readPot() {
  potValue = adcRead(potChannel);
  distanceThreshold = map(potValue, 0, 1023, 20, 100);
}

// Outside Ultrasonic Sensor pins and measured distance
int sensorOutTrigPin = 4;
int sensorOutEchoPin = 5;
float outsideSensorDistance;

// Inside Ultrasonic Sensor pins and measured distance
int sensorInTrigPin = 6;
int sensorInEchoPin = 7;
float insideSensorDistance;

// Sends an ultrasonic pulse and returns measured distance, returns -1 if no valid echo is received
float getDistance(int trigPin, int echoPin) {
  float duration, distance;  
            
  // Trigger LOW
  if (trigPin == 4) {
    PORTG &= ~(1 << PG5);   // pin 4
  } else if (trigPin == 6) {
    PORTH &= ~(1 << PH3);   // pin 6
  }

  delayMicroseconds(2);  

  // Trigger HIGH
  if (trigPin == 4) {
    PORTG |= (1 << PG5);    // pin 4
  } else if (trigPin == 6) {
    PORTH |= (1 << PH3);    // pin 6
  }

  delayMicroseconds(10);  

  // Trigger LOW again
  if (trigPin == 4) {
    PORTG &= ~(1 << PG5);   // pin 4
  } else if (trigPin == 6) {
    PORTH &= ~(1 << PH3);   // pin 6
  }
  
  duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    return -1;
  }

  distance = (duration*.0343)/2;  
  return distance;
}

// Reads both ultrasonic sensors and stores their measured distances
void readSensors() {
  outsideSensorDistance = getDistance(sensorOutTrigPin, sensorOutEchoPin);
  if (!waitMs(60)) {
    return;
  }
  insideSensorDistance = getDistance(sensorInTrigPin, sensorInEchoPin);
}

// LCD Display for display data
#include <LiquidCrystal.h>
LiquidCrystal lcd(13, 12, 8, 9, 10, 11);

// Tracks whether door is currently open or closed
bool doorOpen = false;

// Initializes Serial Monitor, RTC, pins, LCD, interrupt, and starting state
void setup() {
  // Analog Read
  adcInit();

  // Start Serial Monitor for RTC event logging
  Serial.begin(9600);

  // Start RTC module and set time if power was lost
  rtc.begin();
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  // LEDs as outputs
  DDRA |= (1 << PA0) | (1 << PA2) | (1 << PA4) | (1 << PA6);
  
  // Button pins using internal pull-up resistors
  // Start button
  DDRE &= ~(1 << PE4);
  PORTE |= (1 << PE4);
  // Reset button
  DDRC &= ~(1 << PC1);
  PORTC |= (1 << PC1);
  // Off button
  DDRD &= ~(1 << PD7);
  PORTD |= (1 << PD7);
  
  // Attach servo to pin 3 and start in closed position
  servo.attach(3);
  servoAngle = 0;
  servo.write(servoAngle);
  
  // Ultrasonic Sensor trigger pins as outputs and echo pins as inputs
  // Ultrasonic Sensor (Outside)
  DDRG |= (1 << PG5);
  DDRE &= ~(1 << PE3);

  // Ultrasonic Sensor (Inside)
  DDRH |= (1 << PH3);
  DDRH &= ~(1 << PH4);
  
  // Initialize 16x2 LCD
  lcd.begin(16,2); 
  
  // Attach hardware interrupt to Start button
  attachInterrupt(digitalPinToInterrupt(startButtonPin), buttonISR, FALLING);

  // Start system in OFF state
  offState();
}

void loop() {
  // Read potentiometer value and update LCD output
  readPot();
  displayData();
  
  // Start button: from OFF to IDLE
  if (buttonPressed) {
    buttonPressed = false; 
    if (OFF == true) {
      idleState();
    }
    while (startPressed()) {} // Waits until button is released
  }
  
  // Off button: from any state to OFF
  if (offPressed()) {
    offState();
    while (offPressed()) {} // Waits until button is released
    return;
  }

  // Reset button: resets system variables and returns to IDLE
  if (resetPressed()) {
    if (ERROR) {
      reset();
      }
    while (resetPressed()) {} // Waits until button is released
    return;
  }
  
  // Stop running system logic while off
  if (OFF == true) {
    return;
  }

  // Stop running system logic while in ERROR state
  if (ERROR == true) {
    return;
  }
  
  // Run door control logic
  sensors();
}


bool openingDoor() {
  // Transition to ACTIVE while door is opening
  activeState();

  // Check if door is in closed position
  if (servo.read() == 0) {
    logEvent("Door opening");

    // Gradually rotates servo to open door
    for (int i = 0; i <= 90; i += 2) {
      
      // Allow Off button to interrupt door movement
      if (offPressed()) {
        offState();
        while (offPressed()) {}
        return false;
      }
      
      servo.write(i);
      
      if (!waitMs(50) || OFF) {
        return false;
      }


    }
    // Update door status after opening
    doorOpen = true;
    logEvent("Door opened, monitoring animal presence");
    return true;
    
  } else {
    // Enter ERROR if opening starts from the wrong position
    logEvent("ERROR: Door not in closed start position");
	  errorState();
    return false;
  }
}

bool closingDoor() {
  // Grace period before closing to allow animal to fully pass through and
  // prevent the door from closing while it is between both sensors
  if (!waitMs(4000) || OFF) {
    return false;
  }


  // Check if door is currently in open position
  if (servo.read() == 90) {
    logEvent("Door closing");

    // Gradually rotates servo to close door
    for (int i = 90; i >= 0; i -= 2) {
      
      // Allow Off button to interrupt door movement
      if (offPressed()) {
        offState();
        while (offPressed()) {}
        return false;
      }
      
      // Keep reading sensors while door is closing
      readSensors();
      
      servo.write(i);
      if (!waitMs(50) || OFF) {
        return false;
      }


      // Check if sensors readings are valid
      bool outsideValid = outsideSensorDistance != -1;
      bool insideValid  = insideSensorDistance != -1;

      // Reopen the door if an animal is detected while closing
      if ((insideValid && insideSensorDistance <= distanceThreshold) || (outsideValid && outsideSensorDistance <= distanceThreshold)) {
        safetyReopen();
        return false;
      }
    }

    logEvent("Door closed");
  } else {
    // Enter ERROR if closing starts from the wrong position
    errorState();
    logEvent("ERROR: Door not fully closed/opened");
    return false;
  }
  
  // Transition to IDLE the door is fully closed
  idleState();

  return true;
}

// Reopens door if an animal is detected while door is closing
void safetyReopen() {
  servo.write(90);

  if (!waitMs(1000) || OFF) {
    return;
  }

  doorOpen = true;
  logEvent("Door reopening");
}

void sensors() {
  // Read both ultrasonic sensors
  readSensors();

  // Check if sensor readings are valid
  bool outsideValid = outsideSensorDistance != -1;
  bool insideValid  = insideSensorDistance != -1;

  // Enter ERROR if both sensors fail while in IDLE
  if (IDLE && !outsideValid && !insideValid) {
    errorState();
    logEvent("ERROR: Invalid sensor readings");
    return;
  }

  // Enter ERROR if servo position is not in the expected door state while in IDLE
  if (IDLE) {
    if ((!doorOpen && servo.read() != 0) || (doorOpen && servo.read() != 90)) {
      errorState();
      logEvent("ERROR: Servo out of position");
      return;
    }
  }

  // Check if an animal is within the detection threshold
  bool nearby = (outsideValid && outsideSensorDistance <= distanceThreshold) || (insideValid && insideSensorDistance <= distanceThreshold);
  // Open door if animal is detected
  if (nearby && doorOpen == false) {
    openingDoor();
    if (!waitMs(1000) || OFF) {
    return;
  }

  
  // Close door if no animal is detected
  } else if (!nearby && doorOpen == true) {
    bool closedSuccessfully = closingDoor();

    if (OFF == true) {
      return;
    }
  
    if (closedSuccessfully) {
      doorOpen = false;
    } else {
      doorOpen = true;
    }
  }
}

// Resets door position and system variables, then returns to IDLE
void reset() {
  if (!ERROR) {
    return;
  }

  logEvent("RESET button pressed");
  servoAngle = 0;
  servo.write(servoAngle);
  doorOpen = false;
  buttonPressed = false;

  // 5 second wait after reset has been pressed
  logEvent("Cooldown started");
  if (!waitMs(5000) || OFF) {
    return;
  }

  idleState();
}

// Sets system to OFF state
void offState() {
  logEvent("OFF state entered");
  OFF = true;
  IDLE = false;
  ACTIVE = false;
  ERROR = false;
  state();
}

// Sets system to IDLE state
void idleState() {
  logEvent("IDLE state entered");

  OFF = false;
  IDLE = true;
  ACTIVE = false;
  ERROR = false;
  state();
}

// Sets system to ACTIVE state
void activeState() {
  logEvent("ACTIVE state entered");

  OFF = false;
  IDLE = false;
  ACTIVE = true;
  ERROR = false;
  state();
}

// Sets system to ERROR state
void errorState() {
  logEvent("ERROR state entered");

  OFF = false;
  IDLE = false;
  ACTIVE = false;
  ERROR = true;
  state();
}

// Updates the LCD every 1 minute with sensor values, threshold, servo angle, and current state
void displayData() {
  unsigned long currentTime = millis();

  // Check if 1 minute has elapsed
  if (currentTime - lastDisplayTime >= 60000UL) {
    lastDisplayTime = currentTime;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("O:");
    lcd.print(outsideSensorDistance, 0);
    lcd.print(" I:");
    lcd.print(insideSensorDistance, 0);
    lcd.print(" S:");
	  lcd.print(servo.read());

    lcd.setCursor(10, 1);
    lcd.print(" T:");
	  lcd.print(distanceThreshold);

    lcd.setCursor(0, 1);
    if (OFF) {
      lcd.print("OFF");
    } else if (IDLE) {
      lcd.print("IDLE");
    } else if (ACTIVE) {
      lcd.print("ACTIVE");
    } else if (ERROR) {
      lcd.print("ERROR");
    }
  }
}

// Logs system events to Serial Monitor with RTC timestamp
void logEvent(const char* message) {
  DateTime now = rtc.now();
  Serial.print(now.hour());
  Serial.print(":");

  if (now.minute() < 10) Serial.print("0");
  Serial.print(now.minute());
  Serial.print(":");

  if (now.second() < 10) Serial.print("0");
  Serial.print(now.second());
  
  Serial.print(" - ");
  Serial.println(message);
}

// Wait function in milliseconds
bool waitMs(unsigned long waitTime) {
  unsigned long startTime = millis();
  while (millis() - startTime < waitTime) {
    readPot();
    displayData();

    if (offPressed()) {
      offState();
      waitForRelease(offButtonPin);
      return false;
    }

  }
  return true;
}

// Wait until a button is released
void waitForRelease(int buttonPin) {
  if (buttonPin == startButtonPin) {
    while (startPressed()) {
      readPot();
      displayData();
    }
  } else if (buttonPin == offButtonPin) {
    while (offPressed()) {
      readPot();
      displayData();
    }
  } else if (buttonPin == resetButtonPin) {
    while (resetPressed()) {
      readPot();
      displayData();
    }
  }
}

// Helper functions for repetitive use
bool startPressed() {
  return !(PINE & (1 << PE4));
}

bool offPressed() {
  return !(PIND & (1 << PD7));
}

bool resetPressed() {
  return !(PINC & (1 << PC1));   
}