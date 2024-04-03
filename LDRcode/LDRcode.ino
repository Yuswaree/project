int ldr = A7;

void setup() {
  Serial.begin(9600);
  pinMode(ldr,INPUT);
}

void loop() {
  Serial.print("Light sensor value : ");
  int val = analogRead(ldr);
  Serial.println(val);
  delay(500);
}