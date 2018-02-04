 /** mtca_configurator.c
  *
  * Code written by Nuno Barros to program the MTC/A thresholds 
  * all at once.
  * Based on the original code of David Rivera
  *
  **/
  
#define INPUT_SIZE 30

/// -- Declare a couple of constants to set the threshold
// Number of thresholds to set
const int ndacs = 3;
// name of each threshold
const char ch_name[ndacs][2] = {"LO","ME","HI"};
// -- Pinout
// threshold channels
// These can be actively programmed during execution
int ch_pin[ndacs] = {11,12,13};
// clock
const int ch_clk=10;
// syncbar
const int ch_syncbar=9;

// -- Thresholds
/** NOTE: setting the thresholds
 
 - According to the SNO trigger electronics documentation
 the programed threshold is a 12 bit integer. Each unit
 sets an increment of 2.44 mV. The total dynamic range goes from -5V to 5V

 From the documentation:

  The MTC/A has three 12-bit serial DACs which provide individual
   threshold voltages to each of the comparators. The three channels leading to 
   raw triggers are identical except for the different DAC thresholds
   which can be programmed onto the comparator inputs. These thresholds 
   can be programmed between +5V and -5V in increments of 2.44 mV. 
*/
const int threshold[ndacs] = {1,5,10};
//TODO: Check how the threshold compares to the V1730

// -- setup clock counter
const int ncycles = 64; // -- One would need 32 cycles to program 16 rises
// The other 32 are to fiddle with the syncbar
/// -- Declare some global counters for execution control
int i=0,j=0,bit=0;
// String inputString = "";
// bool commandComplete = false;
// Get next command from Serial (add 1 for final 0)
char input[INPUT_SIZE + 1];
bool threshold_updated = false;
  // // Configurations to be loaded into the DAC. 
  // const int thresholds[3][16] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
  //                          {0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0},
  //                          {0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0}};
  


/// -- print_thresholds
///
/// Prints the current thresholds 
void print_thresholds() {
    Serial.println("MTC/A thresholds : \n");
    for (j = 0; j < ndacs; ++j) {
      Serial.print(" [");
      Serial.print(ch_name[j]);
      Serial.print(" : ");
      Serial.print(threshold[j]);
      Serial.print(" (");
      Serial.print(threshold[j],BIN);
      Serial.println(")] ,");
    }
}

/// -- program_thresholds
///
/// This function takes the pinout and threshold information 
/// And sets it there.
void program_thresholds() {
  Serial.println("Setting the MTC/A thresholds...");

  // Start by setting the syncbar


  for (i = 0; i < ncycles; ++i) {
    // Switch the clock
    if (i%2 == 1) {
      digitalWrite(ch_clk ,LOW); 
      //Serial.print(”\\ ”);
      delay (5);
    } else {
      // -- clock rise
      // Where all the work is done
      digitalWrite ( ch_clk ,HIGH); 
      //Serial.print(” /”);
      ///////////Data Line
      // In the first 16 cycles lock syncbar high
      if (i < ncycles/4) {
        // set syncbar high to prepare for programming
        digitalWrite ( ch_syncbar ,HIGH); 
      } else if (i < ncycles/2) {
        digitalWrite ( ch_syncbar ,HIGH); 
      } else {
        for (j = 0; j < ndacs; ++j) {
          // if (index < 4) {
          //   Serial.print("[X]");
          // } else

          // Need to know the exact bit to set

          if ((threshold[j] & (0x1 << bit)) != 0) {
            digitalWrite(ch_pin[j], HIGH);
            //Serial.print("[1]");
          } else {
            digitalWrite(ch_pin[j], LOW);
            //Serial.print("[0]");
          }
        }
        bit += 1;
      }
      delay(5);
    }
  }
  // -- Raise the syncbar again
  digitalWrite(ch_syncbar , HIGH); 
  delay(100);
  Serial.println("New MTC/A thresholds programmed...");
}


void set_single_threshold(char *channel, int th_value) {
  
  if (!strcmp(channel,"LO") {
    threshold[0] = th_value;
  } else if (!strcmp(channel,"ME") {
    threshold[1] = th_value;
  } else if (!strcmp(channel,"HI") {
    threshold[2] = th_value;
  } else {
    Serial.print("\n\n ERROR: Unknown DAC [");
    Serial.print(channel);
    Serial.println("] (ignored)");
  }
}

void serialEvent() {
  while (Serial.available()) {
    byte size = Serial.readBytes(input, INPUT_SIZE);
    // Add the final 0 to end the C string
    input[size] = 0;
    // Read each command pair 
    char* command = strtok(input, ",");
    while (command != 0)
    {
      // Split the command in two values
      char* separator = strchr(command, ':');
      if (separator != 0)
      {
          // Actually split the string in 2: replace ':' with 0
          *separator = 0;
          /// -- The command               
          int servoId = atoi(command);
          ++separator;
          int threshold = atoi(separator);
          set_single_threshold(command,threshold);
          threshold_updated = true;
      }
      // Find the next command in input string
      command = strtok(0, ",");
    }
  }
}


/// -- setup
///
/// This function executes only once at the beginning
/// It is a mandatory function for the Arduino
void setup() {
  Serial.begin(9600);
  Serial.println("Arduino reset..."); 
  pinMode(ch_syncbar,OUTPUT);
  digitalWrite(ch_syncbar,LOW);

  pinMode(ch_clk , OUTPUT); 
  digitalWrite(ch_clk,LOW);
  for (j = 0; j < ndacs; j++) {
    pinMode(ch_pin[j],OUTPUT);
    digitalWrite(ch_pin[j],LOW);
  }
  //digitalWrite (13 ,LOW);
  // -- Start the serial monitor... it helps understand what is going on
  Serial.println("Using initial thresholds...");
  print_thresholds();
  serial.println("Setting MTC/A thresholds...");
  program_thresholds();
  status = 0;
  // Reserve 200 chars for programming
  inputString.reserve(200);

}


void loop() {
  if (threshold_updated) {
    Serial.print("Thresholds have been changed... updating MTC/A");
    print_thresholds();
    program_thresholds();
    threshold_updated = false;
  }

}

/** Unused code

/// -- Aux function to grab a configuration from a string
// String getValue(String data, char separator, int index)
// {
//     int found = 0;
//     int strIndex[] = { 0, -1 };
//     int maxIndex = data.length() - 1;

//     for (int i = 0; i <= maxIndex && found <= index; i++) {
//         if (data.charAt(i) == separator || i == maxIndex) {
//             found++;
//             strIndex[0] = strIndex[1] + 1;
//             strIndex[1] = (i == maxIndex) ? i+1 : i;
//         }
//     }
//     return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
// }


/// -- serialEvent
///
/// Listens to the serial console and checks when a new threshold was added
void serialEventOld() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

///// -- old code

  void loop() {
    Serial.println('In loop...');
    if (status == 1) {
      Serial.println('Setup finished...doing nothing!');
      break;
    }

    // -- Do the programming
    // -- Pull syncbar high to commence programming
    if (i < 32) {
      digitalWrite(syncbar ,HIGH); 
      Serial.print(1);
      delay (50); 
      digitalWrite(syncbar ,LOW); 
      Serial.println(0);
      delay (50);
      Serial.println(”Clock and Data”); 
      Serial.print(”MSB: ”);
    }

    // Program the DAQs
    int index = 0;
    for (i = 0; i < 32; i++) {
      // -- clock fall
      // where changes are effectively committed
      if (i%2 == 1) {
        digitalWrite(clk ,LOW); 
        //Serial.print(”\\ ”);
        delay (5);
      } else {
        // -- clock rise
        // Where all the work is done
        digitalWrite ( clk ,HIGH); 
        //Serial.print(” /”);
        ///////////Data Line
        for (j = 0; j < ndacs; ++j) {
          if (index < 4) {
            Serial.print("[X]");
          } else
          if (thresholds[j][index] == 1) {
            digitalWrite(ch_pin[j], HIGH);
            Serial.print("[1]");
          } else {
            digitalWrite(ch_pin[j], LOW);
            Serial.print("[0]");
          }
        }
        delay(5);
        index += 1;
        // if (index<4){
        //   Serial.print(”[X]”);
        // }
        // else if (array[index] == 1){
        //   digitalWrite(datapulse , HIGH); 
        //   Serial.print(”[1]”);
        // } else {
        //   digitalWrite(datapulse , LOW);
        //   Serial.print(”[0]”);
        // }
        // delay (5);
        // index+=1;
      }
      delay (5);
    }
    if (i == 32) {
      digitalWrite(syncbar , HIGH); 
      Serial.print(” :LSB”);
      delay (1000);
      i =33;
      break ; 
    } else {
      Serial.println("Something really weird just happened...");
    }


    // //pull syncbar High, to commence programming
    // while ( i <32) { 
    //   digitalWrite(syncbar ,HIGH); 
    //   Serial.print(1);
    //   delay (50); 
    //   digitalWrite(syncbar ,LOW); 
    //   Serial.println(0);
    //   delay (50);
    //   Serial.println(”Clock and Data”); 
    //   Serial.print(”MSB: ”);
    //   break ; 
    // }
    // int index = 0;
    // for (i; i<32; i++){
    //   /////////Clock Line
    //   if (i % 2 ==1){
    //     digitalWrite(clockpulse ,LOW); 
    //     Serial.print(”\\ ”);
    //     delay (5);
    //   } else {
    //     digitalWrite ( clockpulse ,HIGH); 
    //     Serial.print(” /”);
    //     ///////////Data Line
    //     if (index<4){
    //       Serial.print(”[X]”);
    //     }
    //     else if (array[index] == 1){
    //       digitalWrite(datapulse , HIGH); 
    //       Serial.print(”[1]”);
    //     } else {
    //       digitalWrite(datapulse , LOW);
    //       Serial.print(”[0]”);
    //     }
    //     delay (5);
    //     index+=1;
    //   }
    //   delay (5);
    // }
    // if (i == 32) {
    //   digitalWrite(syncbar , HIGH); 
    //   Serial.print(” :LSB”);
    //   delay (1000);
    //   i =33;
    //   break ; 
    // }
    // while ( i ==32) {
    //   digitalWrite(syncbar , HIGH); 
    //   Serial.print(” :LSB”);
    //   delay (1000);
    //   i =33;
    //   break ; 
    // }
  }
**/
