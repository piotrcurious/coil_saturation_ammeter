
// define the pins and constants
#define PWM_PIN 9 // PWM pin for the boost converter
#define BYPASS_PIN 10 // PWM pin for the bypass MOSFET
#define VOLTAGE_PIN A0 // analog pin for the output voltage divider
#define CURRENT_PIN A1 // analog pin for the input current resistor
#define SUPPLY_VOLTAGE 5.0 // supply voltage in volts
#define RESISTOR_VALUE 1.0 // input current resistor value in ohms
#define VOLTAGE_DIVIDER_RATIO 0.5 // output voltage divider ratio
#define CORE_AREA 0.01 // core cross section area in square meters
#define WIRE_LOOPS 100 // number of wire loops around the core
#define PWM_MAX 65535 // maximum value for 16-bit PWM

// define the saturation threshold function
// returns a float value between 0 and 1 indicating how much of the core capacity is left until saturation
// takes a PWM value as input and sets the PWM_PIN accordingly
// measures the output voltage and input current and calculates the efficiency of the boost converter
// assumes that the efficiency is inversely proportional to the core saturation level
// uses a binary search algorithm to find the optimal PWM value that maximizes the efficiency
float saturation_threshold(int pwm_value) {
  // initialize variables for storing the lower and upper bounds of the binary search
  int lower_bound = 0; // start from zero PWM value
  int upper_bound = PWM_MAX; // start from maximum PWM value
  
  // loop until the lower and upper bounds are close enough or equal
  while (lower_bound < upper_bound - 1) {
    // set the PWM value for the boost converter as the midpoint of the lower and upper bounds
    pwm_value = (lower_bound + upper_bound) / 2;
    analogWrite(PWM_PIN, pwm_value);
    
    // wait for the converter to stabilize
    delay(100);
    
    // read the output voltage and input current
    int voltage_reading = analogRead(VOLTAGE_PIN);
    int current_reading = analogRead(CURRENT_PIN);
    
    // convert the readings to volts and amps
    float output_voltage = voltage_reading * (SUPPLY_VOLTAGE / 1023.0) / VOLTAGE_DIVIDER_RATIO;
    float input_current = (SUPPLY_VOLTAGE - current_reading * (SUPPLY_VOLTAGE / 1023.0)) / RESISTOR_VALUE;
    
    // calculate the output power and input power of the converter
    float output_power = output_voltage * output_voltage / (VOLTAGE_DIVIDER_RATIO * RESISTOR_VALUE);
    float input_power = SUPPLY_VOLTAGE * input_current;
    
    // calculate the efficiency of the converter
    float efficiency = output_power / input_power;
    
    // print the PWM value and efficiency to serial monitor for debugging purposes
    Serial.print("PWM: ");
    Serial.print(pwm_value);
    Serial.print(", Efficiency: ");
    Serial.println(efficiency);
    
    // update the lower and upper bounds based on the efficiency value
    // assumes that the efficiency has a peak at some PWM value and then decreases as the core saturates
    if (efficiency > 0.9) {
      // if the efficiency is very high, increase the lower bound to find a higher PWM value
      lower_bound = pwm_value;
    }
    else {
      // if the efficiency is low, decrease the upper bound to find a lower PWM value
      upper_bound = pwm_value;
    }
  }
  
  // return a value between 0 and 1 indicating how much of the core capacity is left until saturation
  // assumes that the efficiency is inversely proportional to the core saturation level
  return efficiency;
}

// define the ammeter function
// returns a float value indicating the unknown DC current in amps running through the additional winding around the core
// uses the saturation threshold function and varies the bypass level to infer this current
// starts from the highest bypass level to determine a rough current value and once coil saturation is detected uses the saturation threshold function output to increase measurement precision
void ammeter() {
  // initialize variables for storing the bypass level, current value and saturation level
  int bypass_level = PWM_MAX; // start from the highest bypass level
  float current_value = 0.0; // start from zero current value
  float saturation_level = 0.0; // start from zero saturation level
  
  // loop until saturation level reaches a threshold or bypass level reaches zero
  while (saturation_level < 0.9 && bypass_level > 0) {
    // set the bypass level for the MOSFET
    analogWrite(BYPASS_PIN, bypass_level);
    
    // wait for the circuit to stabilize
    delay(100);
    
    // call the saturation threshold function with a random PWM value for the boost converter
    saturation_level = saturation_threshold(random(PWM_MAX));
    
    // calculate the current value based on the bypass level and saturation level
    // assumes that the current is proportional to the bypass level and inversely proportional to the saturation level
    current_value = bypass_level * (SUPPLY_VOLTAGE / PWM_MAX) / (saturation_level * WIRE_LOOPS * CORE_AREA);
    
    // print the current value and saturation level to serial monitor for debugging purposes
    Serial.print("Current: ");
    Serial.print(current_value);
    Serial.print(" A, Saturation: ");
    Serial.println(saturation_level);
    
    // decrease the bypass level by a fixed amount for each iteration of the loop
    bypass_level -= 1024; // use one sixteenth of the PWM range
    
  }
}

// define the setup function
// runs once when the Arduino is powered on or reset
// initializes the serial communication and the PWM pins
void setup() {
  // initialize serial communication at 9600 baud rate
  Serial.begin(9600);
  
  // set the PWM pins as outputs
  pinMode(PWM_PIN, OUTPUT);
  pinMode(BYPASS_PIN, OUTPUT);
  
  // set the PWM frequency to 31.25 kHz for both pins
  // this requires changing the timer registers for Timer1
  // see https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM for more details
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11); // enable fast PWM mode and non-inverting mode for both channels
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // set prescaler to 1 and enable fast PWM mode
  ICR1 = PWM_MAX; // set the maximum PWM value to 65535
}

// define the loop function
// runs repeatedly after the setup function is done
// calls the ammeter function and prints the result to serial monitor
void loop() {
  // call the ammeter function and store the result in a variable
  float unknown_current = ammeter();
  
  // print the result to serial monitor with a newline character
  Serial.print("Unknown current: ");
  Serial.print(unknown_current);
  Serial.println(" A");
  
  // wait for one second before repeating the loop
  delay(1000);
}
