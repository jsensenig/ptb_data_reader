 /**
  * Code written by Nuno Barros to program the MTC/A thresholds 
  * all at once.
  * Based on the original code of David Rivera
  */
  
/// -- Declare a couple of constants to set the threshold
// Number of thresholds to set
const int ndacs = 3;


/// -- Declare some global counters for execution control
int i=0,j=0,bit=0;
  //code written by David Rivera to program the MTC/A board. 
  // 09/16
// Modified by nbarros
  const int edges = 16; 
  int i=0;
  int j = 0;
  int status;
  // pinout
  const  int program = 7; // not used

  const inst ndacs = 3;
  // 0 : LO
  // 1 : ME
  // 2 : HI
  char tnames[ndacs][2] = {"LO","ME","HI"};
  const int dports[ndacs] = {13,14,15}
  //int datapulse = 13; 
  const int clk = 12; 
  const int syncbar = 11;
  

  // Configurations to be loaded into the DAC. 
  const int thresholds[3][16] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
                           {0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0},
                           {0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0}};
  
  // configuration to be loaded to DAC
  //int array[16] = {1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0};

  void setup() {
    pinMode( program , INPUT );
    pinMode(syncbar,OUTPUT);
    pinMode(clk , OUTPUT); //clock pinMode(datapulse, OUTPUT); //data pinMode(syncbar, OUTPUT);
    for (j = 0; j < ndacs; j++) {
      pinMode(dports[j],OUTPUT);
      digitalWrite(dports[j],LOW);
    }
    //digitalWrite (13 ,LOW);
    // -- Start the serial monitor... it helps understand what is going on
    Serial.begin(9600);
    Serial.println("Setting up arduino ..."); 
    Serial.print("Thresholds : ");
    for (j = 0; j < ndacs; j++) {
      Serial.print(tnames[j]);
      Serial.print(" : ");
      Serial.print(thresholds[j]);
      Serial.print(" , ");
    }
    status = 0;
    Serial.print("SyncBar:");
  }

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
        Serial.print(”\\ ”);
        delay (5);
      } else {
        // -- clock rise
        // Where all the work is done
        digitalWrite ( clk ,HIGH); 
        Serial.print(” /”);
        ///////////Data Line
        for (j = 0; j < ndacs; ++j) {
          if (index < 4) {
            Serial.print("[X]");
          } else
          if (thresholds[j][index] == 1) {
            digitalWrite(dports[j], HIGH);
            Serial.print("[1]");
          } else {
            digitalWrite(dports[j], LOW);
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
