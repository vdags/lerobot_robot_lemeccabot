#include <Servo.h>

const byte N_SERVOS = 6;

Servo servos[N_SERVOS];

byte servoPins[N_SERVOS] = {3,5,6,9,10,11};

byte servoMin = 10;
byte servoMax = 170;

byte servoPos[N_SERVOS];
byte newServoPos[N_SERVOS];

int servoIndex;

const byte buffSize = 40;
char inputBuffer[buffSize];

const char startMarker = '<';
const char endMarker = '>';

byte bytesRecvd = 0;

boolean readInProgress = false;
boolean newDataFromPC = false;

char messageFromPC[buffSize] = {0};

unsigned long curMillis;



void setup()
{
  Serial.begin(9600);

  for(byte i=0;i<N_SERVOS;i++)
  {
    servos[i].attach(servoPins[i]);
    servoPos[i] = servoMin;
    newServoPos[i] = servoMin;
    servos[i].write(servoMin);
  }

  Serial.println("<Arduino is ready>");
}



void loop()
{
  curMillis = millis();
  getDataFromPC();
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
  char * strtokIndx;

  strtokIndx = strtok(inputBuffer,",");
  strcpy(messageFromPC, strtokIndx);

  if(messageFromPC[0] == 'C')
  {
    Serial.println("<C,OK>");
    return;
  }

  if(messageFromPC[0] == 'D')
  {
    for(byte i=0;i<N_SERVOS;i++)
      servos[i].detach();

    Serial.println("<D,OK>");
    return;
  }

  strtokIndx = strtok(NULL,",");
  servoIndex = atoi(strtokIndx);

  if(servoIndex >= N_SERVOS)
  {
    Serial.println("<ERR,ID>");
    return;
  }


  if(messageFromPC[0] == 'E')
  {
    enableTorque(servoIndex);
    return;
  }

  if(messageFromPC[0] == 'T')
  {
    disableTorque(servoIndex);
    return;
  }

  if(messageFromPC[0] == 'R')
  {
    int pos = readServoPos(servoIndex);

    Serial.print("<R,");
    Serial.print(servoIndex);
    Serial.print(",");
    Serial.print(pos);
    Serial.println(">");

    return;
  }

  if(messageFromPC[0] == 'W')
  {
    strtokIndx = strtok(NULL,",");
    newServoPos[servoIndex] = atoi(strtokIndx);

    writeServoPos(servoIndex);

    Serial.print("<W,");
    Serial.print(servoIndex);
    Serial.print(",");
    Serial.print(servoPos[servoIndex]);
    Serial.println(">");

    return;
  }

}



void writeServoPos(byte id)
{
  if(newServoPos[id] < servoMin)
      newServoPos[id] = servoMin;

  if(newServoPos[id] > servoMax)
      newServoPos[id] = servoMax;

  servoPos[id] = newServoPos[id];

  servos[id].write(servoPos[id]);
}



int readServoPos(byte id)
{
  return servos[id].read();
}



void enableTorque(byte id)
{
  servos[id].attach(servoPins[id]);

  Serial.print("<E,");
  Serial.print(id);
  Serial.println(",OK>");
}



void disableTorque(byte id)
{
  servos[id].detach();

  Serial.print("<T,");
  Serial.print(id);
  Serial.println(",OK>");
}
