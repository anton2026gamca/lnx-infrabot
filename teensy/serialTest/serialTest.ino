void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Waiting for input...");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // reading line
    //String input = Serial8.readStringUntil('\n'); // reading line
    // main lenght and format chcek
    if (input.length() != 31 || input[0] != '{' || input[1] != '"' || input[2] != 'a' || input[3] != '"' || input[4] != '=' || input[5] != '"'
                    || input[29] != '"' || input[30] != '}' ) {
      Serial.println("Invalid string!");
      return;
    }
    String data = input.substring(6, 29); // deleting start end ending of string
    // comma check
    if (data[5] != ',' || data[11] != ',' || data[17] != ',') {
      Serial.println("Wrong comma structure!");
      return;
    }
    // sign chcek
    for (int i = 0; i <= 18; i += 6) { 
      if (data[i] != '+' && data[i] != '-') {
        Serial.println("Wrong sign!");
        return;
      }
    }
    // numbers check
    for (int i = 0; i < 4; i++) {
      for (int j = 1; j < 5; j++) {
        if (data[i*6 + j] < '0' || data[i*6 + j] > '9') {
          Serial.print("Wrong number at:");
          Serial.println(i*6 + j);
          return;
        }
      }
    }
    int values[4];
    values[0] = data.substring(0, 5).toInt();
    values[1] = data.substring(6, 11).toInt();
    values[2] = data.substring(12, 17).toInt();
    values[3] = data.substring(18, 23).toInt();

    Serial.print("Default string: ");
    Serial.println(input);
    Serial.println("Extracted values:");
    for (int i = 0; i < 4; i++) {
      Serial.println(values[i]);
    }
  }
}
