#include "Functions.h"


#include "MeccaBrain.h"
MeccaBrain servo(5);//servo data pin is connected to digital pin 5


int row;
int x;

const unsigned int TRIG_PIN = 7;
const unsigned int ECHO_PIN = 6;
char receivedChar;

// calibration
// If you have already determined the coefficents for the duration to distance equation, set calibration to false and set the coeffiecnmts to that value
bool calibration = false;//if this is false you can declare offset and slope as const
//the equation: y = mx + b, distance = slope * durtion + offset
double offset = 1.101;//the intercept from the least squares regression analysis
double slope = 0.0342 / 2.0;//the slope from the least squares regression analysis, divided by 2.0 because the duration is to the object and back but distance to the object would only be time to get there or back.

//room scan variables
int pstart = 60;
int pend = 120;
int ystart = 60;
int yend = 120;
int pstep = 5;
int ystep = 5;
//scanner variables(Scanner setup geometry)
//{pitch_x, pitch_y, pitch_z, yaw_x, yaw_y, yaw_z, x, y, z}
servoGeometry geo1 = { -0.48, 0.52, 6.63, 0.48, 0.0, 7.87, 0.0, 0.0, 6.83};
servoGeometry geo2 = geo1;

//functions
unsigned long analog_read()//fucntion to read in vlues from the ultrasonic sensor, if you are using a different sensor than the HY-SRF05 sensor, replace what is in this function
{
  //trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  //recive duration from the sensor
  unsigned long duration = pulseIn(ECHO_PIN, HIGH);
  return duration;
}

void calibrate(double start, double step, unsigned int size, unsigned int sample_size)
{
  //initialize arrays
  double darr[size];// array of pre-measured distances
  double val[size];// array of average distance values from the sensor
  double durations[size]; //array of average durations, in ms

  // initialize the array of pre-measured distances
  for (unsigned int i = 0; i < size; i++)
  {
    darr[i] = start + step * i;
  }


  for (unsigned int position = 0; position < size; position++)
  {
    bool newData = false;//reset newData flag
    Serial.println("Waiting for input to advance");
    Serial.println("Next measurment distance is ");
    Serial.println(darr[position], 5);
    while (!newData)//wait for user to enter 'a' into the serial monitor before running for this distance; done for each distance measurment
    {
      if (Serial.available() > 0)
      {
        receivedChar = Serial.read();
        if (receivedChar == 'a')
        {
          newData = true;
          receivedChar = ' ';
        }
      }
    }

    unsigned long total = 0;//initialize total

    Serial.println("Ready");//acknowledges that input has been recieved
    delay(1000);
    Serial.println("Go");

    //takes an average of the values collected for this measurment distance
    for (unsigned int num = 0; num < sample_size; num++)
    {

      unsigned long duration = analog_read();
      duration /= 2; //divided by 2 because the duration is to the object and back but distance to the object would only be time to get there or back.

      if (duration <= 1 || duration >= 200000)
      {
        num -= 1;//don't count this run if the value doesn't make sense
      }
      else
      {
        Serial.println(num, DEC);//print the current iterarion
        total += duration;// add the duration to the total
        delay(20);
      }
    }
    Serial.println("Done");

    double average = total / sample_size * 1.0;// compute the average
    durations[position] = average;// add this average durtion to the array
    double distance = average * 0.0342 + 1.101;//calculate the predicted distance
    val[position] = distance; // store the predicted distance to the array
  }

  //print the data (to 5 decimal places) as three(3) columns (measured on left, sensor in the middle, and durations on right) sperated by commas
  Serial.println();
  for (unsigned int count = 0; count < size; count++)
  {
    Serial.print(darr[count], 3);
    Serial.print(", ");
    Serial.print(val[count], 5);
    Serial.print(", ");
    Serial.print(durations[count], 5);
    Serial.println();
  }
  //helpful site for viewing: http://www.alcula.com/calculators/statistics/scatter-plot/
  //values can be broken up by pasting into excel and using the text splitter

  double line_vals[3];
  lsrl(durations, darr, size, line_vals);
  Serial.println();//print a space

  //print the least squares regression line coefficents and correlation coefficent (R Squared), to 7 decimal places
  Serial.print("A0(Intercept): ");
  Serial.print(line_vals[0], 7);
  Serial.print(", A1(slope): ");
  Serial.print(line_vals[1], 7);
  Serial.print(", R Squared: ");
  Serial.print(line_vals[2], 7);
  Serial.println();
  //print equation
  Serial.println("New Equation:");
  Serial.println();
  Serial.print("distance = duration / 2.0 * ");
  Serial.print(line_vals[1], 4);
  Serial.print(" + ");
  Serial.print(line_vals[0], 4);
  Serial.println();
  offset = line_vals[0];
  slope = line_vals[1];
}

//end functions
void setup() {

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  Serial.println("Initialized"); //comment out this line if you are going to use the seril plotter

  delay(500);
  if (calibration)
  {

    //if coefficents have not been determined, set these values
    const double start = 4.0; //distance to start the interval from
    const double interval = 1.5; // the distance(cm) between each measurement
    const unsigned int size = 8;// number of data points that will be collected for calibration
    const unsigned int sample_size = 10; //the number of samples to take for each average; larger numbers for more accuracy, smaller numbers for faster run times

    calibrate(start, interval, size, sample_size);

  }
  for (int iter = 0; iter < 20; iter++)
  {
    servo.setServoPosition(0, map(ystart, 0, 180, 0x18, 0xE8));
    servo.setServoPosition(1, map(pstart, 0, 180, 0x18, 0xE8));
    servo.communicate();//initiate servo communication

    delay(15);
  }
}



void loop() {

  Serial.println("");
  Serial.print("[");
  for (int pitch = pstart; pitch < pend; pitch += pstep)
  {
    double theta = pitch * DEG_TO_RAD;
    servo.setServoPosition(1, map(pitch, 180, 0, 0x18, 0xE8));

    servo.communicate();//initiate servo communication
    delay(100);
    Serial.println("");
    for (int yaw = ystart; yaw < yend; yaw += ystep)
    {
      double azimuth = yaw * DEG_TO_RAD;
      servo.setServoPosition(0, map(yaw, 0, 180, 0x18, 0xE8));
      //servo.setServoPosition(1, map(pitch, 0, 180, 0x18, 0xE8));

      servo.communicate();//initiate servo communication
      if (yaw == ystart)
      {
        delay(1000);
      }
      delay(100);
      Vector v_offsets;
      calculateOffset(geo1, theta, azimuth, v_offsets);
      /*Serial.println("");
      Serial.println("");
      Serial.print("Pitch: ");
      Serial.print(pitch, DEC);
      Serial.print(", Yaw: ");
      Serial.print(yaw, DEC);
      printVector(v_offsets);*/

      unsigned long duration = analog_read();
      double distance = slope * duration + offset;//compute distance
      geo2.pitch_y = geo1.pitch_y - distance;
      Vector v_signal;
      calculateOffset(geo2, theta, azimuth, v_signal);
      
     /* Serial.print("Pitch: ");
      Serial.print(pitch, DEC);
      Serial.print(", Yaw: ");
      Serial.print(yaw, DEC);
      printVector(v_signal);*/
      printVectorDifference(v_offsets, v_signal);
      //printSphereToVector(distance, theta, azimuth, v);
      //Serial.print(distance, 5); //print distacne to 5 decimal places
      delay(20);
      if (yaw < yend - ystep)
      {
        Serial.print("; ");
      }

    }
    if (pitch < pend - pstep)
    {
      Serial.println("; ");
    }
  }
  Serial.println("]");
  Serial.println("");
  delay(500000);
}
