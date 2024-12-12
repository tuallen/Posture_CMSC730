// #include <Arduino.h>

// const int stretchPin = 39;
// // const int vin = 3.3;
// // const int resistor1 = 10;


// void setup() {
//   Serial.begin(9600);
//   pinMode(stretchPin, INPUT);
// }

// void loop() {
//   // float vout = 0;        // Store output voltage 
//   // float refresistor2 = 0;  //  Value is determined by the voltage 
//                           //that falls across after it has been measured.  
//   // float buffer = 0;      // Buffer variable for calculation 
//   int value = analogRead(stretchPin);       //Read the value
//                                           //Just like we did earlier
//   // vout = (5.0 / 1023.0) * value;         // Calculates the voltage 
//   // buffer = (vin / vout) - 1; 
//   // refresistor2 = resistor1 / buffer; 
//   // Serial.print("Voltage: "); 
//   // Serial.println(vout);                  // Outputs the information 
//   // Serial.print("Resistance: ");  
//   // Serial.println(refresistor2);   
//   // delay(1000); 
//   Serial.printf(">%d", value);  
//   delay(100); 
// }