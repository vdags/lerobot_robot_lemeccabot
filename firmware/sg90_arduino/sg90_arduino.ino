#include <Servo.h>

const byte N_SERVOS = 6;


//==============================================
//  Pin layout
//==============================================

byte servoPins[N_SERVOS] = {3, 5, 7, 9, 10, 11};

// Multiplexer pins (S3=4, S2=6, S1=8, S0=12)
const byte MUX_S0 = 12;
const byte MUX_S1 = 8;
const byte MUX_S2 = 6;
const byte MUX_S3 = 4;
const byte MUX_OUT = A0;

// ==============================================
// End Pin layout
//==============================================

// ==============================================
// Servo parameters
// ==============================================
byte servoStartPos[N_SERVOS] = {10,10,10,10,10,10}; // Start position for each servo.

// Servo min and max accepted position commands both for safety and servo's limit detection.
byte servoMinCmd[N_SERVOS] = {10,10,10,10,10,10};
byte servoMaxCmd[N_SERVOS] = {170,170,170,170,170,170};

// Servo min and max potentiometer feedback
int servoMinFeedback[N_SERVOS] = {0,0,0,0,0,0};
int servoMaxFeedback[N_SERVOS] = {1023,1023,1023,1023,1023,1023};

//PID variables
int KP[N_SERVOS] = {10,10,10,10,10,10};
int KD[N_SERVOS] = {0,0,0,0,0,0};
int KI[N_SERVOS] = {0,0,0,0,0,0};

// ==============================================
//  End Servo parameters
// ==============================================

// ==============================================
// Controller mode
// ==============================================

char PID[] = "PIDPOS";
// Values: 'OFF' (proportional position control from servo controller), 
//         'PIDPOS' (PID control on position)

// ==============================================
// End Controller mode
// ==============================================

// ==============================================
// Servo objects and state variables
// ==============================================

Servo servos[N_SERVOS];

byte servoPos[N_SERVOS]; // Current command sent to servo
byte targetServoPos[N_SERVOS]; // Desired position to be written to servo 
// targetServoPos and servoPos are different when using PID control, as servoPos will be updated based on feedback while targetServoPos is the desired position we want to achieve

int servoFeedbackPos[N_SERVOS];  // Position feedback from potentiometer
int servoCurrentmA[N_SERVOS];    // Current in mA from 1Ω resistor

// ==============================================
// End Servo objects and state variables
// ==============================================


// ==============================================
// Serial communication variables
// ==============================================

const byte buffSize = 80;
char inputBuffer[buffSize];

const char startMarker = '<';
const char endMarker = '>';

byte bytesRecvd = 0;

boolean readInProgress = false;
boolean newDataFromPC = false;

char messageFromPC[buffSize] = {0};

unsigned long curMillis;

// ==============================================
// End Serial communication variables
// ==============================================

void setup()
{
  Serial.begin(9600);

  // Initialize multiplexer control pins
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_OUT, INPUT);

  for(byte i=0;i<N_SERVOS;i++)
  {
    servos[i].attach(servoPins[i]);
    servoPos[i] = servoStartPos[i];
    targetServoPos[i] = servoStartPos[i];
    servos[i].write(servoStartPos[i]);
  }

  Serial.println("<Arduino is ready>");
}



void loop()
{
  curMillis = millis();
  getDataFromPC();
  if (strcmp(PID, "PIDPOS") == 0){ // If using PID control, we need to continuously update servo positions based on feedback
    for (byte i = 0; i < N_SERVOS; i++)
      writeServoPosPIDPos(i);
  }
  // DEBUG
  
  // END DEBUG
}



void getDataFromPC()
{
  if(Serial.available() > 0)
  {
    char x = Serial.read();

    if (x == endMarker)
    {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      parseData();
    }

    if(readInProgress)
    {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd++;

      if (bytesRecvd == buffSize)
        bytesRecvd = buffSize - 1;
    }

    if (x == startMarker)
    {
      bytesRecvd = 0;
      readInProgress = true;
    }
  }
}



void parseData()
{
  char *token;

  token = strtok(inputBuffer, ",");
  if(token == NULL) return;

  char command = token[0];

  // -------- HANDSHAKE --------
  if(command == 'C')
  {
    for(byte i = 0; i < N_SERVOS; i++)
      servos[i].attach(servoPins[i]);
    Serial.println("<C,OK>");
    return;
  }

  if(command == 'D')
  {
    for(byte i = 0; i < N_SERVOS; i++)
      servos[i].detach();
    Serial.println("<D,OK>");
    return;
  }

  // -------- ENABLE MULTIPLE --------
  if(command == 'E')
  {
    Serial.print("<E");
    while((token = strtok(NULL, ",")) != NULL)
    {
      int id = atoi(token);
      Serial.print(",");
      Serial.print(id);

      if(id >= 0 && id < N_SERVOS)
      {
        enableTorque(id);
        Serial.print(",OK");
      }
      else
      {
        Serial.print(",KO");
      }
    }
    Serial.println(">");
    return;
  }

  // -------- DISABLE MULTIPLE --------
  if(command == 'T')
  {
    Serial.print("<T");
    while((token = strtok(NULL, ",")) != NULL)
    {
      int id = atoi(token);
      Serial.print(",");
      Serial.print(id);

      if(id >= 0 && id < N_SERVOS)
      {
        disableTorque(id);
        Serial.print(",OK");
      }
      else
      {
        Serial.print(",KO");
      }
    }
    Serial.println(">");
    return;
  }

  // -------- READ COMMAND POSITION (SERVO POS) MULTIPLE --------
  if(command == 'A')
  {
    Serial.print("<A");
    while((token = strtok(NULL, ",")) != NULL)
    {
      int id = atoi(token);
      Serial.print(",");
      Serial.print(id);

      if(id >= 0 && id < N_SERVOS)
      {
        int pos = readServoPos(id);
        Serial.print(",");
        Serial.print(pos);
      }
      else
      {
        Serial.print(",KO");
      }
    }
    Serial.println(">");
    return;
  }

  // -------- WRITE MULTIPLE --------
  if(command == 'W')
  {
    Serial.print("<W");
    while((token = strtok(NULL, ",")) != NULL)
    {
      int id = atoi(token);
      token = strtok(NULL, ",");
      if(token == NULL) break;

      int value = atoi(token);
      Serial.print(",");
      Serial.print(id);

      if(id >= 0 && id < N_SERVOS)
      {
        targetServoPos[id] = value;
        writeServoPos(id);
        Serial.print(",");
        Serial.print(targetServoPos[id]);
      }
      else
      {
        Serial.print(",KO");
      }
    }
    Serial.println(">");
    return;
  }

  // -------- READ FEEDBACK POSITION MULTIPLE --------
  if(command == 'F')
  {
    Serial.print("<F");
    while((token = strtok(NULL, ",")) != NULL)
    {
      int id = atoi(token);
      Serial.print(",");
      Serial.print(id);

      if(id >= 0 && id < N_SERVOS)
      {
        int posFeedback = readServoFeedbackPos(id);
        Serial.print(",");
        Serial.print(posFeedback);
      }
      else
      {
        Serial.print(",KO");
      }
    }
    Serial.println(">");
    return;
  }

  // -------- READ CURRENT MULTIPLE --------
  if(command == 'I')
  {
    Serial.print("<I");
    while((token = strtok(NULL, ",")) != NULL)
    {
      int id = atoi(token);
      Serial.print(",");
      Serial.print(id);

      if(id >= 0 && id < N_SERVOS)
      {
        int current = readServoCurrent(id);
        Serial.print(",");
        Serial.print(current);
      }
      else
      {
        Serial.print(",KO");
      }
    }
    Serial.println(">");
    return;
  }

  // -------- READ ALL (FEEDBACK + CURRENT + COMMAND POS) MULTIPLE --------
  if(command == 'R')
  {
    Serial.print("<R");
    while((token = strtok(NULL, ",")) != NULL)
    {
      int id = atoi(token);
      Serial.print(",(");
      Serial.print(id);

      if(id >= 0 && id < N_SERVOS)
      {
        servoFeedbackPos[id] = readServoFeedbackPos(id);
        servoCurrentmA[id] = readServoCurrent(id);
        Serial.print(",");
        Serial.print(servoFeedbackPos[id]);
        Serial.print(",");
        Serial.print(servoCurrentmA[id]);
        Serial.print(",");
        Serial.print(targetServoPos[id]);
        Serial.print(")");
      }
      else
      {
        Serial.print(",KO");
      }
    }
    Serial.println(">");
    return;
  }

}



void writeServoPos(byte id)
{
  if(targetServoPos[id] < servoMinCmd[id])
      targetServoPos[id] = servoMinCmd[id];

  if(targetServoPos[id] > servoMaxCmd[id])
      targetServoPos[id] = servoMaxCmd[id];

  
  if (strcmp(PID, "PIDPOS") == 0)
    writeServoPosPIDPos(id);
  if (strcmp(PID, "OFF") == 0){
    servoPos[id] = targetServoPos[id];
    servos[id].write(servoPos[id]);
  }
}

void writeServoPosPIDPos(byte id)
{
  int feedbackDeg = map(readServoFeedbackPos(id), 
                      servoMinFeedback[id], servoMaxFeedback[id], 
                      servoMinCmd[id], servoMaxCmd[id]);
  int error = targetServoPos[id] - feedbackDeg;
  int derivative = error - (servoPos[id] - feedbackDeg); // This is a very basic derivative term, consider using a proper derivative calculation
  int integral = error; // This is a very basic integral term, consider using a proper integral accumulation

  int output = KP[id] * error + KD[id] * derivative + KI[id] * integral;

  if(output < servoMinCmd[id])
      output = servoMinCmd[id];

  if(output > servoMaxCmd[id])
      output = servoMaxCmd[id];

  servoPos[id] = output;

  servos[id].write(servoPos[id]);
}


int readServoPos(byte id)
{
  return servos[id].read();
}



void enableTorque(byte id)
{
  servos[id].attach(servoPins[id]);

  /*
  Serial.print("<E,");
  Serial.print(id);
  Serial.println(",OK>");
  */
}



void disableTorque(byte id)
{
  servos[id].detach();
  /*
  Serial.print("<T,");
  Serial.print(id);
  Serial.println(",OK>");
  */
}



void selectMuxChannel(byte channel)
{
  digitalWrite(MUX_S0, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(MUX_S1, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(MUX_S2, (channel & 0x04) ? HIGH : LOW);
  digitalWrite(MUX_S3, (channel & 0x08) ? HIGH : LOW);
  delayMicroseconds(50);
}



int readServoFeedbackPos(byte id)
{
  byte muxChannel = id * 2;
  selectMuxChannel(muxChannel);
  return analogRead(MUX_OUT);
}



int readServoCurrent(byte id)
{
  byte muxChannel = id * 2 + 1;
  selectMuxChannel(muxChannel);
  int rawValue = analogRead(MUX_OUT);
  
  return (rawValue * 5 * 1000) / (1023 * 1);
}
