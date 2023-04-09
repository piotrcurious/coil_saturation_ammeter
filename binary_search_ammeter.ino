
// define the pins and constants
#define PWM_PIN 9 // PWM pin for the boost converter
#define BYPASS_PIN 10 // PWM pin for the bypass MOSFET
#define INPUT_PIN A0 // analog input pin for the coil voltage drop
#define OUTPUT_PIN A1 // analog input pin for the output voltage divider
#define SUPPLY_VOLTAGE 5.0 // supply voltage in volts
#define INPUT_RESISTOR 100.0 // input resistor in ohms
#define OUTPUT_RESISTOR 200.0 // input resistor in ohms
#define VOLTAGE_DIVIDER_RATIO 0.5 // ratio of the voltage divider
#define CORE_AREA 0.0001 // cross section area of the core in square meters
#define WIRE_LOOPS 100 // number of wire loops around the core
#define MAX_PWM 65535 // maximum value for 16-bit PWM

// initialize the variables
float inputCurrent; // input current in amps
float outputVoltage; // output voltage in volts
float outputPower; // output power in watts
float efficiency; // efficiency of the dc-dc converter
float saturationThreshold; // saturation threshold of the core in teslas
float magneticFlux; // magnetic flux of the core in webers
float unknownCurrent; // unknown current in the additional winding in amps

// initialize the 16-bit PWM functions
void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(BYPASS_PIN, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11); // set fast PWM mode and non-inverting mode
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // set fast PWM mode and no prescaler
  ICR1 = MAX_PWM; // set the top value for 16-bit PWM
}

// main loop
void loop() {
  // call the test function to measure the saturation threshold and efficiency
  testFunction();
  
  // call the ammeter function to measure the unknown current
  ammeterFunction();
  
  // print the results to the serial monitor
  Serial.print("Saturation threshold: ");
  Serial.print(saturationThreshold);
  Serial.println(" T");
  
  Serial.print("Efficiency: ");
  Serial.print(efficiency);
  Serial.println(" %");
  
  Serial.print("Unknown current: ");
  Serial.print(unknownCurrent);
  Serial.println(" A");
  
}

// test function to measure the saturation threshold and efficiency of the dc-dc converter using the core
void testFunction() {
  
  // initialize some variables for the test function
  float maxEfficiency = 0.0; // maximum efficiency achieved so far
  float maxFlux = 0.0; // maximum magnetic flux achieved so far
  
  // loop through different PWM values from low to high
  for (int pwmValue = 0; pwmValue <= MAX_PWM; pwmValue++) {
    
    // set the PWM value to the boost converter
    OCR1A = pwmValue;
    
    // wait for a short time to stabilize the output
    delay(10);
    
    // read the input and output voltages from the analog pins and convert them to volts
    float inputVoltage = analogRead(INPUT_PIN) * (SUPPLY_VOLTAGE / 1023.0);
    outputVoltage = analogRead(OUTPUT_PIN) * (SUPPLY_VOLTAGE / VOLTAGE_DIVIDER_RATIO / 1023.0);
    
    // calculate the input current from the input voltage drop and the input resistor value
    inputCurrent = (SUPPLY_VOLTAGE - inputVoltage) / INPUT_RESISTOR;
    
    // calculate the output power from the output voltage and the voltage divider ratio (assuming equal resistors)
   // outputPower = outputVoltage * outputVoltage / (VOLTAGE_DIVIDER_RATIO * INPUT_RESISTOR);
    outputPower = outputVoltage * outputVoltage / (OUTPUT_RESISTOR);
    // calculate the efficiency from the input and output power
    efficiency = outputPower / (inputCurrent * SUPPLY_VOLTAGE) * 100.0;
    
    // calculate the magnetic flux from the input current, the wire loops and the core area (assuming a linear relationship)
    magneticFlux = inputCurrent * WIRE_LOOPS * CORE_AREA;
    
    // check if the efficiency is higher than the maximum efficiency so far and update it if so
    if (efficiency > maxEfficiency) {
      maxEfficiency = efficiency;
    }

    // check if the magnetic flux is higher than the maximum flux so far and update it if so
    if (magneticFlux > maxFlux) {
      maxFlux = magneticFlux;
    }
    
    // check if the efficiency has dropped significantly from the maximum efficiency, indicating saturation of the core
    if (efficiency < maxEfficiency * 0.9) {
      // set the saturation threshold to the maximum flux achieved so far
      saturationThreshold = maxFlux;
      // break the loop as no further increase in flux is possible
      break;
    }
  }
}

// ammeter function to measure the unknown current in the additional winding by using the test function and varying the bypass level
void ammeterFunction() {
  
  // initialize some variables for the ammeter function
  float minBypass = 0.0; // minimum bypass level (0% duty cycle)
  float maxBypass = 1.0; // maximum bypass level (100% duty cycle)
  float midBypass; // middle bypass level (50% duty cycle)
  float prevEfficiency; // previous efficiency measured
  float currEfficiency; // current efficiency measured
  
  // loop until the bypass range is small enough to get a precise measurement
  while (maxBypass - minBypass > 0.01) {
    
    // set the middle bypass level to the average of the minimum and maximum bypass levels
    midBypass = (minBypass + maxBypass) / 2.0;
    
    // set the PWM value to the bypass MOSFET according to the middle bypass level
    OCR1B = midBypass * MAX_PWM;
    
    // wait for a short time to stabilize the output
    delay(10);
    
    // call the test function to measure the efficiency with the current bypass level
    testFunction();
    
    // store the current efficiency in a variable
    currEfficiency = efficiency;
    
    // check if this is the first iteration of the loop
    if (minBypass == 0.0 && maxBypass == 1.0) {
      // store the current efficiency as the previous efficiency for comparison
      prevEfficiency = currEfficiency;
      // reduce the maximum bypass level by half to increase the unknown current
      maxBypass = midBypass;
    }
    else {
      // compare the current efficiency with the previous efficiency
      if (currEfficiency < prevEfficiency * 0.9) {
        // efficiency has dropped significantly, indicating saturation of the core due to increased unknown current
        // increase the minimum bypass level by half to decrease the unknown current
        minBypass = midBypass;
      }
      else {
        // efficiency has not dropped significantly, indicating no saturation of the core due to decreased unknown current
        // reduce the maximum bypass level by half to increase the unknown current
        maxBypass = midBypass;
      }
      // store the current efficiency as the previous efficiency for comparison
      prevEfficiency = currEfficiency;
    }
  }
  
  // calculate the unknown current from the saturation threshold, the wire loops and the core area (assuming a linear relationship)
  unknownCurrent = saturationThreshold / (WIRE_LOOPS * CORE_AREA);
}
