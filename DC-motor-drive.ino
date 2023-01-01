/* Pin configurations for the left side motor */
const int enableLeft = 5;
const int highLeft = 6;       
const int lowLeft = 7;

/* Pin configurations for the right side motor */
const int enableRight = 10;
const int highRight = 8;      
const int lowRight = 9;

/* Pin configurations for the input pins that receive signals from Raspberry Pi*/
const int in0 = 0;
const int in1 = 1;
const int in2 = 2;
const int in3 = 3;  

int d0, d1, d2, d3, data;
bool rightLane = true; 

void setup() 
{
  pinMode(enableLeft, OUTPUT);
  pinMode(highLeft, OUTPUT);
  pinMode(lowLeft, OUTPUT);

  pinMode(enableRight, OUTPUT);
  pinMode(highRight, OUTPUT);
  pinMode(lowRight, OUTPUT);
  
  pinMode(in0, INPUT_PULLUP);
  pinMode(in1, INPUT_PULLUP);
  pinMode(in2, INPUT_PULLUP);
  pinMode(in3, INPUT_PULLUP);
}

/* Calculates the decimal value of the received data */
void calculateValue()
{
  d0 = digitalRead(in0);
  d1 = digitalRead(in1);
  d2 = digitalRead(in2);
  d3 = digitalRead(in3);
  data = (8*d3) + (4*d2) + (2*d1) + d0;  
}

void driveForward()
{
  digitalWrite(highLeft, LOW);
  digitalWrite(lowLeft, HIGH);
  analogWrite(enableLeft,255);

  digitalWrite(highRight, LOW);
  digitalWrite(lowRight, HIGH);
  analogWrite(enableRight,255);
}

void stop()
{
  digitalWrite(highLeft, LOW);
  digitalWrite(lowLeft, HIGH);
  analogWrite(enableLeft,0);

  digitalWrite(highRight, LOW);
  digitalWrite(lowRight, HIGH);
  analogWrite(enableRight,0);
}

/* According to the level specified, it adjusts the turning angle by changing the provided PWM duty cycle */
void turnLeft(int level)
{
  if(level == 1)
    analogWrite(enableLeft,150);
  else if(level == 2)
    analogWrite(enableLeft,100);
  else if(level == 3)
    analogWrite(enableLeft,50);
  else 
    return;
  digitalWrite(highLeft, LOW);
  digitalWrite(lowLeft, HIGH);
  digitalWrite(highRight, LOW);
  digitalWrite(lowRight, HIGH);
  analogWrite(enableRight,255);
}

/* According to the level specified, it adjusts the turning angle by changing the provided PWM duty cycle */
void turnRight(int level)
{
  if(level == 1)
    analogWrite(enableRight,150);
  else if(level == 2)
    analogWrite(enableRight,100);
  else if(level == 3)
    analogWrite(enableRight,50);
  else 
    return;
  digitalWrite(highLeft, LOW);
  digitalWrite(lowLeft, HIGH);
  analogWrite(enableLeft,255);
  digitalWrite(highRight, LOW);
  digitalWrite(lowRight, HIGH);
}

/* According to the current lane, sends the necessary signals to change lane */ 
void changeLane()
{
  analogWrite(enableLeft,0);
  analogWrite(enableRight,0);	
  delay(1000);
  if(rightLane)
  {
    digitalWrite(highLeft, HIGH);
    digitalWrite(lowLeft, LOW);
    digitalWrite(highRight, LOW);
    digitalWrite(lowRight, HIGH); 
    analogWrite(enableLeft,255);  
    analogWrite(enableRight,255);
  }
  else
  {
	digitalWrite(highLeft, LOW);
    digitalWrite(lowLeft, HIGH);
    digitalWrite(highRight, HIGH);
    digitalWrite(lowRight, LOW); 
    analogWrite(enableLeft,255);  
    analogWrite(enableRight,255);  
  }
  delay(500);
  analogWrite(enableLeft,0);
  analogWrite(enableRight,0);
  delay(250);
  driveForward();
  delay(1000);
  analogWrite(enableLeft,0);
  analogWrite(enableRight,0);	
  delay(250);
  if(rightLane)
  {
    digitalWrite(highLeft, LOW);
    digitalWrite(lowLeft, HIGH);
    digitalWrite(highRight, HIGH);
    digitalWrite(lowRight, LOW); 
    analogWrite(enableLeft,255);  
    analogWrite(enableRight,255);
  }
  else
  {
	digitalWrite(highLeft, HIGH);
    digitalWrite(lowLeft, LOW);
    digitalWrite(highRight, LOW);
    digitalWrite(lowRight, HIGH); 
    analogWrite(enableLeft,255);  
    analogWrite(enableRight,255);  
  }
  delay(500);
  analogWrite(enableLeft,0);
  analogWrite(enableRight,0);	
  delay(1000);
  digitalWrite(highLeft, LOW);
  digitalWrite(lowLeft, HIGH);
  digitalWrite(highRight, LOW);
  digitalWrite(lowRight, HIGH); 
  analogWrite(enableLeft,150);  
  analogWrite(enableRight,150);
  delay(500);
  rightLane = !rightLane; // toggle  
}

void loop() 
{
  calculateValue(); 	
  if(data == 0)
	driveForward();
  else if(data == 1 || data == 2 || data == 3)
    turnRight(data);
  else if(data == 4 || data == 5 || data == 6)
    turnLeft(data-3); 
  else if(data == 7)
	changeLane(); 
  else 
	stop(); 
}
