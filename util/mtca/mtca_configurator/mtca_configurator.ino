 /** mtca_configurator.c
  *
  * Code written by Nuno Barros to program the MTC/A thresholds 
  * all at once.
  * Based on the original code of David Rivera
  *
  **/

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
/////////////// GLOBAL VARIABLES AND DEFINES ///////////////////////////
////////////////////////////////////////////////////////////////////////

#define INPUT_SIZE 30
#define UNIT_DELAY 5 // -- production unit delay
// -- use this for esting using the serial plotter
// -- Usually with this one should also set the debug variable to true
//#define fcdelay mydelay
#define fcdelay delay 

const bool debug = false;

/// -- Declare a couple of constants to set the threshold
// Number of thresholds to set
const int ndacs = 3;
// name of each threshold
const char ch_name[ndacs][4] = {"LO\0","ME\0","HI\0"};
// -- Pinout
// threshold channels
// These can be actively programmed during execution
const int ch_pin[ndacs] = {11,12,13};
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
// -- 2105 --> ~100 mV
int threshold[ndacs] = {2105,2305,2505};

// -- setup clock counter
const int ncycles = 32; // -- One would need 32 cycles to program 16 rises

/// -- Declare some global counters for execution control
int i=0,j=0,k=0,bit=15;

// These control the serial input
char input[INPUT_SIZE + 1];
bool threshold_updated = false;

// auxiliary plot figures
int bitlo=0, bitme=0,bithi=0, bitclk=0,bitsyncbar=0;

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

/** -- mydelay 
///
/// Implementation that breaks a delay into smaller delays and forces 
/// the plotter to be refreshed
/// The serial plotter is stupid and plots one point per println
///    so the trick is to make a lot of println
*/
void mydelay(int value) {
  for (int aux = 0; aux < value; aux++) {
    print_levels();
    delay(1);
  }
}

/** -- print_thresholds
///
/// Prints the current thresholds, both in dec and bin format 
*/
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

/** -- set_bit
///
/// Sets a specific bit for dataidx register
*/
void set_bit(int bit, int dataidx) {
  if ((threshold[dataidx] & (0x1 << bit)) != 0) {
    digitalWrite(ch_pin[dataidx], HIGH);
    if (debug) {
      /** for plotting */
      switch(dataidx) {
        case 0: 
          bitlo = 6; break;
        case 1:
          bitme = 6; break;
        case 2:
          bithi = 6; break;
      }
    }
  } else {
    digitalWrite(ch_pin[dataidx], LOW);
    /** plot */
    if (debug) {
      switch(dataidx) {
        case 0: 
          bitlo = 0; break;
        case 1:
          bitme = 0; break;
        case 2:
          bithi = 0; break;
      }
    }
  }
}

/** -- print_levels
///
/// Used only with debug mode turned on. It prints the contents of 
/// the debugging variables into the serial plotter, adding a point 
/// to the plot.
*/
void print_levels() {
  if (debug) {
    Serial.print(bitlo);
    Serial.print("\t");
    Serial.print(bitme);
    Serial.print("\t");
    Serial.print(bithi);
    Serial.print("\t");
    Serial.print(bitclk);
    Serial.print("\t");
    Serial.println(bitsyncbar);
  }
}
 
/** -- program_thresholds
///
/// This function takes the pinout and threshold information 
/// And sets the appropriate thresholds.
/// It is basically the workhorse of this program.
*/
void program_thresholds() {
  if (!debug) {
    Serial.println("Setting the MTC/A thresholds...");
  }

  //Reset the bit counter
  bit = 15;
  // -- Reset all the signals into a NULL position
  digitalWrite(ch_syncbar, HIGH);
  bitsyncbar = 6;
  
  digitalWrite(ch_clk, LOW); 
  bitclk = 0;

  
  // Set the first of the 16 bits
  // into the DACs
  // Once the clock rizes, this bit becomes set
  for (j = 0; j < ndacs; j++) {
    set_bit(bit,j);
  }
  
  /*plot */
  print_levels();

  // Since we just raised the syncbar
  // we can add a long delay 
  // We do not need to, but it doesn't hurt is is easier to 
  // spot in the scope
  fcdelay(10*UNIT_DELAY);
  
  // Since bit 15 is already assigned, we can decrease
  // this bit
  bit--;

  // -- Once the syncbar is lowered start running the clock to 
  // program the bits
  digitalWrite(ch_syncbar , LOW); 
  bitsyncbar = 0;
  
  for (i = 0; i < ncycles; ++i) {
    // Switch the clock
    // fastest way to check for an odd number is by looking at 
    // the very first bit
    // Since we started at 0, the odd cycles are the clock low
    if (i & 0x1) {
      digitalWrite(ch_clk ,LOW);
      bitclk = 0;
      // Make the bit assigment on the low swing
      // So that the thresholds are set on the high swing
      // this avoids troubles of race conditions and bit shifting
      if (bit >= 0) { // -- protection for bit rollover
        for (j = 0; j < ndacs; ++j) {
          set_bit(bit,j);
        }
        bit--;
      } 
    } else {
      // -- clock rise
      // Where everything is committed
      digitalWrite ( ch_clk ,HIGH); 
      bitclk=6;
    }
    /*plot */
    print_levels();
    fcdelay(UNIT_DELAY);
  }
  // -- Raise the syncbar again
  // to stop the DAC programming
  digitalWrite(ch_syncbar , HIGH); 
  bitsyncbar = 6;
  // Add a couple extra clock cycles to make sure that everything is committed
  // totally unnecessary but helps see that things are consistent in the scope
  for (i=0; i < 4; ++i) {
    if (i & 0x1) {
      digitalWrite(ch_clk , LOW);
      bitclk=0;
    } else {
      digitalWrite(ch_clk , HIGH);
      bitclk=6;
    }
    /*plot */
    print_levels();
    fcdelay(UNIT_DELAY);
  }
    /*plot */
  print_levels();

  // final delay to see the bar locked... also 
  fcdelay(20*UNIT_DELAY);
  if (!debug) {
    Serial.println("MTC/A thresholds programmed...");
  }
}


/** -- set_single_threshold
 * 
 * Parses a serial command and updates the appropriate threshold
 * It refuses to update in case of bad data
 */
void set_single_threshold(char *channel, int th_value) {
  // Check for invalid threshold set
  if (th_value > ((0x1 << 12)-1) || (th_value < 0)) {
    Serial.println("Threshold out of range. ");
    Serial.print("Received : ");
    Serial.print(channel);
    Serial.print(":");
    Serial.println(th_value);
    Serial.print("Valid range is [0,");
    Serial.print(((0x1 << 12)-1));
    Serial.println("]");
    Serial.println("Ignoring this setting...");
    return;    
  }

  if (!strcmp(channel,"LO")) {
    threshold[0] = th_value;
  } else if (!strcmp(channel,"ME")) {
    threshold[1] = th_value;
  } else if (!strcmp(channel,"HI")) {
    threshold[2] = th_value;
  } else {
    Serial.print("\n\n ERROR: Unknown DAC [");
    Serial.print(channel);
    Serial.println("] (ignored)");
  }
}

/** -- serialEvent
 *  
 * Reserved arduino function for serial monitor monitor input
 * It takes a threshold setting and calls set_single_threshold
 * 
 * The threshold setting should have the form
 * LO:123,ME:456,HI:789
 * 
 * It is perfectly valid to change the order and even to only set 1 or 2 thresholds
 * If more than a threshold is set, each must be separated by a comma
 */
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


/** -- setup
 *  
 *  This function executes only once at the beginning
 *  It is a mandatory function for the Arduino
 */
void setup() {
  
   // Initialize the serial console
  Serial.begin(9600);
  if (!debug) {
    Serial.println("Arduino reset..."); 
  } else {
    Serial.print("LO");
    Serial.print("\t");
    Serial.print("ME");
    Serial.print("\t");
    Serial.print("HI");
    Serial.print("\t");
    Serial.print("CLK");
    Serial.print("\t");
    Serial.println("SYNCBAR");
  }

  // Initial setting for all pins
  pinMode(ch_syncbar,OUTPUT);
  // syncbar should start in high
  digitalWrite(ch_syncbar,HIGH);

  pinMode(ch_clk , OUTPUT); 
  digitalWrite(ch_clk,LOW);
  for (j = 0; j < ndacs; j++) {
    pinMode(ch_pin[j],OUTPUT);
    digitalWrite(ch_pin[j],LOW);
  }

  if (!debug) {
    Serial.println("Setting initial MTC/A thresholds...");
  }
  program_thresholds();

}


void loop() {
  if (threshold_updated) {
    Serial.println("Thresholds have been changed... updating MTC/A");
    print_thresholds();
    program_thresholds();
    threshold_updated = false;
  }
    if (k < 10) {
      print_levels();
      k++;
    }
}


