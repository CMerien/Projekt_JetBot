#define echoPin 2   // Pin pre Echo (D2)
#define trigPin 3   // Pin pre Trig (D3)


long duration; // Čas návratu pulzu
int distance; // Vypočítaná vzdialenosť

void setup()
{
    pinMode(trigPin, OUTPUT); // Trig pin ako výstup
    pinMode(echoPin, INPUT); // Echo pin ako vstup

    // Serial Communication is starting with 9600 of
    // baudrate speed
    Serial.begin(9600);

    // The text to be printed in serial monitor
    Serial.println("Meranie vzdialenosti cez Arduino nano");
    delay(500);
}

void loop()
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(
        trigPin,
        HIGH); // turn on the Trigger to generate pulse
    delayMicroseconds(
        10); // keep the trigger "ON" for 10 ms to generate
             // pulse for 10 ms.

    digitalWrite(trigPin, LOW); 

    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.0344 / 2; 

    Serial.print("Distance: ");
    Serial.print(distance); 
    Serial.println(" cm");
    delay(100);
}

