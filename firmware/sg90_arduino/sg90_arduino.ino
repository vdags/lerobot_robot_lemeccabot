#include <Servo.h>
int BAUDRATE = 9600;
const byte N_SERVOS = 6;
// TODO Add overwritable parameters for servo limits, PID gains, etc.

//==============================================
//  Pin layout
//==============================================

byte servoPins[N_SERVOS] = {3, 5, 6, 9, 10, 11};

// Multiplexer pins (S3=4, S2=6, S1=8, S0=12)
const byte MUX_S0 = 12;
const byte MUX_S1 = 8;
const byte MUX_S2 = 7;
const byte MUX_S3 = 4;
const byte MUX_OUT = A0;

// ==============================================
// End Pin layout
//==============================================

// ==============================================
// Servo parameters
// ==============================================
byte servoStartPos[N_SERVOS] = {100,170,100,100,90,90}; // Start position for each servo.

// Servo min and max accepted position commands both for safety and servo's limit detection.
byte servoMinCmd[N_SERVOS] = {0,40,0,0,0,0};
byte servoMaxCmd[N_SERVOS] = {180,180,170,180,90,180};

// Servo min and max potentiometer feedback
int servoMinFeedback[N_SERVOS] = {50,50,50,50,50,50};
int servoMaxFeedback[N_SERVOS] = {960,950,950,960,830,1023};

//PID variables
int KP[N_SERVOS] = {1,1,1,1,45,1}; // Do not vibrate at 225
int KD[N_SERVOS] = {0,0,0,0,0,0};
int KI[N_SERVOS] = {0,0,0,0,5,0};

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
  Serial.begin(BAUDRATE);

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
    // Check if next token is a register name (config) or motor ID
    token = strtok(NULL, ",");
    if(token == NULL) return;
    
    // Try to determine if it's a register or motor ID
    int firstVal = atoi(token);
    boolean isMotorId = (firstVal >= 0 && firstVal < N_SERVOS && strlen(token) <= 2);
    
    if(!isMotorId)
    {
      // Configuration register write
      if(strcmp(token, "STP") == 0)
      {
        Serial.print("<W,STP");
        for(byte i = 0; i < N_SERVOS; i++)
        {
          token = strtok(NULL, ",");
          if(token == NULL) break;
          int value = atoi(token);
          servoStartPos[i] = value;
          Serial.print(",");
          Serial.print(servoStartPos[i]);
        }
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "MINCMD") == 0)
      {
        Serial.print("<W,MINCMD");
        for(byte i = 0; i < N_SERVOS; i++)
        {
          token = strtok(NULL, ",");
          if(token == NULL) break;
          int value = atoi(token);
          servoMinCmd[i] = value;
          Serial.print(",");
          Serial.print(servoMinCmd[i]);
        }
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "MAXCMD") == 0)
      {
        Serial.print("<W,MAXCMD");
        for(byte i = 0; i < N_SERVOS; i++)
        {
          token = strtok(NULL, ",");
          if(token == NULL) break;
          int value = atoi(token);
          servoMaxCmd[i] = value;
          Serial.print(",");
          Serial.print(servoMaxCmd[i]);
        }
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "MINFB") == 0)
      {
        Serial.print("<W,MINFB");
        for(byte i = 0; i < N_SERVOS; i++)
        {
          token = strtok(NULL, ",");
          if(token == NULL) break;
          int value = atoi(token);
          servoMinFeedback[i] = value;
          Serial.print(",");
          Serial.print(servoMinFeedback[i]);
        }
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "MAXFB") == 0)
      {
        Serial.print("<W,MAXFB");
        for(byte i = 0; i < N_SERVOS; i++)
        {
          token = strtok(NULL, ",");
          if(token == NULL) break;
          int value = atoi(token);
          servoMaxFeedback[i] = value;
          Serial.print(",");
          Serial.print(servoMaxFeedback[i]);
        }
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "KP") == 0)
      {
        Serial.print("<W,KP");
        for(byte i = 0; i < N_SERVOS; i++)
        {
          token = strtok(NULL, ",");
          if(token == NULL) break;
          int value = atoi(token);
          KP[i] = value;
          Serial.print(",");
          Serial.print(KP[i]);
        }
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "KD") == 0)
      {
        Serial.print("<W,KD");
        for(byte i = 0; i < N_SERVOS; i++)
        {
          token = strtok(NULL, ",");
          if(token == NULL) break;
          int value = atoi(token);
          KD[i] = value;
          Serial.print(",");
          Serial.print(KD[i]);
        }
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "KI") == 0)
      {
        Serial.print("<W,KI");
        for(byte i = 0; i < N_SERVOS; i++)
        {
          token = strtok(NULL, ",");
          if(token == NULL) break;
          int value = atoi(token);
          KI[i] = value;
          Serial.print(",");
          Serial.print(KI[i]);
        }
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "CMOD") == 0)
      {
        token = strtok(NULL, ",");
        if(token != NULL)
        {
          strcpy(PID, token);
          Serial.print("<W,CMOD,");
          Serial.print(PID);
          Serial.println(">");
        }
        else
        {
          Serial.println("<W,CMOD,KO>");
        }
        return;
      }
      else if(strcmp(token, "BAUDRATE") == 0)
      { 
        if(token == NULL) break;
          int value = atoi(token);
        BAUDRATE = value;
        Serial.println("<W,BAUDRATE," + String(BAUDRATE) + ">");
        Serial.end();
        Serial.begin(BAUDRATE);
        return;
      }
      else
      {
        // Unknown register
        Serial.println("<W,KO>");
        return;
      }
    }
    else
    {
      // Motor position write
      Serial.print("<W");
      
      // Handle the first motor (already have the ID in firstVal)
      int id = firstVal;
      token = strtok(NULL, ",");
      if(token == NULL) {
        Serial.println(">");
        return;
      }
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
      
      // Handle remaining (id, value) pairs
      while((token = strtok(NULL, ",")) != NULL)
      {
        id = atoi(token);
        token = strtok(NULL, ",");
        if(token == NULL) break;

        value = atoi(token);
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
    }
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
    // Check if next token is a register name (config) or motor ID (telemetry)
    token = strtok(NULL, ",");
    if(token == NULL) return;
    
    // Try to determine if it's a register or motor ID
    int firstVal = atoi(token);
    boolean isMotorId = (firstVal >= 0 && firstVal < N_SERVOS && strlen(token) <= 2);
    
    if(!isMotorId)
    {
      // Configuration register read
      if(strcmp(token, "STP") == 0)
      {
        Serial.print("<R,STP");
        for(byte i = 0; i < N_SERVOS; i++)
        {
          Serial.print(",");
          Serial.print(servoStartPos[i]);
        }
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "MINCMD") == 0)
      {
        Serial.print("<R,MINCMD");
        for(byte i = 0; i < N_SERVOS; i++)
        {
          Serial.print(",");
          Serial.print(servoMinCmd[i]);
        }
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "MAXCMD") == 0)
      {
        Serial.print("<R,MAXCMD");
        for(byte i = 0; i < N_SERVOS; i++)
        {
          Serial.print(",");
          Serial.print(servoMaxCmd[i]);
        }
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "MINFB") == 0)
      {
        Serial.print("<R,MINFB");
        for(byte i = 0; i < N_SERVOS; i++)
        {
          Serial.print(",");
          Serial.print(servoMinFeedback[i]);
        }
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "MAXFB") == 0)
      {
        Serial.print("<R,MAXFB");
        for(byte i = 0; i < N_SERVOS; i++)
        {
          Serial.print(",");
          Serial.print(servoMaxFeedback[i]);
        }
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "KP") == 0)
      {
        Serial.print("<R,KP");
        for(byte i = 0; i < N_SERVOS; i++)
        {
          Serial.print(",");
          Serial.print(KP[i]);
        }
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "KD") == 0)
      {
        Serial.print("<R,KD");
        for(byte i = 0; i < N_SERVOS; i++)
        {
          Serial.print(",");
          Serial.print(KD[i]);
        }
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "KI") == 0)
      {
        Serial.print("<R,KI");
        for(byte i = 0; i < N_SERVOS; i++)
        {
          Serial.print(",");
          Serial.print(KI[i]);
        }
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "CMOD") == 0)
      {
        Serial.print("<R,CMOD,");
        Serial.print(PID);
        Serial.println(">");
        return;
      }
      else if(strcmp(token, "BAUDRATE") == 0)
      {
        Serial.print("<R,BAUDRATE,");
        Serial.print(BAUDRATE);
        Serial.println(">");
        return;
      }
    }
    else
    {
      // Telemetry read (original behavior)
      Serial.print("<R");
      int id = firstVal;
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
      
      // Read remaining motor IDs
      while((token = strtok(NULL, ",")) != NULL)
      {
        id = atoi(token);
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
    }
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
  
  return (rawValue * 5 * 1000) / (1023 * 1); // Resistor used is 1 ohm.
}
