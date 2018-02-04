  //code written by David Rivera to program the MTC/A board. 
  // 09/16
  int edges = 16; 
  int i=0;
  // pinout
  int program = 7;
  int datapulse = 11; 
  int clockpulse = 10; 
  int syncbar = 9;
  
  // configuration to be loaded to DAC
  int array[16] = {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0};

  void setup() {
    pinMode( program , INPUT );
    pinMode(clockpulse , OUTPUT); //clock pinMode(datapulse, OUTPUT); //data pinMode(syncbar, OUTPUT);
    digitalWrite (datapulse ,LOW);
    Serial.begin(9600);
    Serial.println("Beginning .."); 
    Serial.print("SyncBar:");
  }

  void loop() {
    //pull syncbar High, to commence programming
    while ( i <32) { 
      digitalWrite(syncbar ,HIGH); 
      Serial.print(1);
      delay (50); 
      digitalWrite(syncbar ,LOW); 
      Serial.println(0);
      delay (50);
      Serial.println("Clock and Data"); 
      Serial.print("MSB: ");
      break ; 
    }
    int index = 0;
    for (i; i<32; i++){
      /////////Clock Line
      if (i % 2 ==1){
        digitalWrite(clockpulse ,LOW); 
        Serial.print("\\ ");
        delay (5);
      } else {
        digitalWrite ( clockpulse ,HIGH); 
        Serial.print(" /");
        ///////////Data Line
        if (index<4){
          Serial.print("[X]");
        }
        else if (array[index] == 1){
          digitalWrite(datapulse , HIGH); 
          Serial.print("[1]");
        } else {
          digitalWrite(datapulse , LOW);
          Serial.print("[0]");
        }
        delay (5);
        index+=1;
      }
      delay (5);
    }
    while ( i ==32) {
      digitalWrite(syncbar , HIGH); 
      Serial.print(" :LSB");
      delay (1000);
      i =33;
      break ; 
    }
  }
