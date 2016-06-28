

// Control pins for the right half of the H-bridge
const int enable2 = 9; // PWM pin for speed control
const int in3 = 8;
const int in4 = 7;

//other half
const int enable1 = 6; // PWM pin for speed control
const int in1 = 3;
const int in2 = 2;

//Control pins for ultrasonic distance sensor
const int trigPin = 12;
const int echoPin = 11;

/*
  Color Sensor      Arduino
  -----------      --------
  VCC               5V
  GND               GND
  s0                8
  s1                9
  s2                12
  s3                11
  OUT               10
  OE                GND
*/
const int s0 = A0;
const int s1 = A1;
const int s2 = A2;
const int s3 = A3;
const int out = A4;
// LED pins connected to Arduino
int redLed = 2;
int greenLed = 3;
int blueLed = 4;
// Variables
int red = 0;
int green = 0;
int blue = 0;

//Set Minimum Distance required for Bot to maintain from obstacles (in cm)
const int minimumDistance = 20;

void setup() {

  //control pins for ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //control pins for the left motor
  pinMode( enable1, OUTPUT);
  pinMode( in1, OUTPUT);
  pinMode( in2, OUTPUT);

  //control pins for the right motor
  pinMode( enable2, OUTPUT);
  pinMode( in3, OUTPUT);
  pinMode( in4, OUTPUT);

  Serial.begin(9600);
  //control for color sensor
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(blueLed, OUTPUT);
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);
}

void loop() {

  long distance;
  distance = measureDistance ();

  // check validity of distance
  if (distance >= 15 || distance <= 0) {
    stopRobot (2000);
    color();
    Serial.print("R Intensity:");
    Serial.print(red, DEC);
    Serial.print(" G Intensity: ");
    Serial.print(green, DEC);
    Serial.print(" B Intensity : ");
    Serial.print(blue, DEC);
    //Serial.println();

    if (red < blue && red < green && red < 15)
    {
      Serial.println(" - (Red Color)");
      digitalWrite(redLed, HIGH); // Turn RED LED ON
      digitalWrite(greenLed, LOW);
      digitalWrite(blueLed, LOW);
    }

    else if (blue > red && blue < green)
    {
      Serial.println(" - (Purple Color)");
      digitalWrite(redLed, LOW);
      digitalWrite(greenLed, LOW);
      digitalWrite(blueLed, HIGH); // Turn BLUE LED ON
    }

    else if (blue < red && blue < green)
    {
      Serial.println(" - (Blue Color)");
      digitalWrite(redLed, LOW);
      digitalWrite(greenLed, LOW);
      digitalWrite(blueLed, HIGH); // Turn BLUE LED ON
    }

    else if (green < red && green < blue)
    {
      Serial.println(" - (Green Color)");
      digitalWrite(redLed, LOW);
      digitalWrite(greenLed, HIGH); // Turn GREEN LED ON
      digitalWrite(blueLed, LOW);
    }

    else if (green > red && green < blue)
    {
      Serial.println(" - (Yellow Color)");
      digitalWrite(redLed, LOW);
      digitalWrite(greenLed, HIGH); // Turn GREEN LED ON
      digitalWrite(blueLed, LOW);
    }

    else {
      Serial.println();
    }
    delay(100);
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, LOW);
    digitalWrite(blueLed, LOW);
  }

  else  {
    if (distance < minimumDistance) {
      //turn left, take measurement
      //turn right, take measurement
      //go towards greater distance

      turnLeft (100);
      int leftDistance = measureDistance();
      turnRight (200);
      int rightDistance = measureDistance();
      turnLeft (100); //return to forward position

      if (leftDistance >= 200
          || leftDistance <= 0
          || rightDistance >= 200
          || rightDistance <= 0)
      {

      }
      else if (leftDistance > rightDistance) {
        turnLeft (100);
      }
      else {
        turnRight (100);
      }
    } //end of case for distance less than minimumDistance

    //begin case for valid range with distance greater than minimumDistance
    else  {
      goForward (500);
    }
  } //end valid range case

}

//The section that follows is used to define the functions used in this code loop

void color()
{
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  //count OUT, pRed, RED
  red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  digitalWrite(s3, HIGH);
  //count OUT, pBLUE, BLUE
  blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  digitalWrite(s2, HIGH);
  //count OUT, pGreen, GREEN
  green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
}

void goForward (int timeToMove) {
  //Go Forward for a certain amount of time
  Serial.print ("Move forward for ");
  Serial.println (timeToMove); //both motors go forward
  digitalWrite (in1, HIGH);
  digitalWrite (in2, LOW);
  analogWrite (enable1, 150);
  digitalWrite (in3, HIGH);
  digitalWrite (in4, LOW);
  analogWrite (enable2, 150);
  delay (timeToMove); // wait for humans to enjoy the bot going forward
}

void turnLeft (int timeToMove) {
  Serial.print ("turn left for ");
  Serial.println (timeToMove);
  digitalWrite (in1, HIGH);
  digitalWrite (in2, LOW);
  analogWrite (enable1, 150);
  digitalWrite (in3, HIGH);
  digitalWrite (in4, LOW);
  analogWrite (enable2, 100);
  delay (timeToMove);
}

void turnRight (int timeToMove) {
  Serial.print ("turn right for ");
  Serial.println (timeToMove);
  digitalWrite (in1, HIGH);
  digitalWrite (in2, LOW);
  analogWrite (enable1, 100);
  digitalWrite (in3, HIGH);
  digitalWrite (in4, LOW);
  analogWrite (enable2, 150);
  delay (timeToMove);
}

void stopRobot (int timeToMove) {
  Serial.print ("stop for ");
  Serial.println (timeToMove); //both motors stop
  digitalWrite (in1, LOW);
  digitalWrite (in2, LOW);
  digitalWrite (enable1, LOW);
  digitalWrite (in3, LOW);
  digitalWrite (in4, LOW);
  digitalWrite (enable2, LOW);
  delay (timeToMove); // wait for humans to enjoy the bot stopping
}

void goReverse (int timeToMove) {
  //Go backward for a certain abmount of time
  Serial.print ("Move backwards for ");
  Serial.println (timeToMove); //both motors go in reverse
  digitalWrite (in1, LOW);
  digitalWrite (in2, HIGH);
  digitalWrite (enable1, HIGH);
  digitalWrite (in3, LOW);
  digitalWrite (in4, HIGH);
  digitalWrite (enable2, HIGH);
  delay (timeToMove); // wait for humans to enjoy the bot going in reverse
}

long measureDistance () {
  long duration, distance;

  // send the pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2); // low for 2 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // high for 10 microseconds
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH); // measure the time to the echo
  distance = (duration / 2) / 29.1; // calculate the distance in cm
  return distance;
}

