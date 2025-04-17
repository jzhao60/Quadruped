int relayPin = 7;

void setup() {
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW); // Start with motor off (depends on relay logic)
}

void loop() {
  digitalWrite(relayPin, HIGH); // Turn motor ON
  delay(2000);
  digitalWrite(relayPin, LOW);  // Turn motor OFF
  delay(2000);
}