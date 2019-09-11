#ifndef Functions_h
#define Functions_h
#define swap(a, b) { double t = a; a = b; b = t; }
struct vector
{
  double x;
  double y;
  double z;
};
typedef struct vector Vector;
void printVector(Vector &vec)
{
  Serial.println("");
  Serial.print("X: ");
  Serial.print(vec.x, 5);
  Serial.print("; Y: ");
  Serial.print(vec.y, 5);
  Serial.print("; Z: ");
  Serial.print(vec.z, 5);
  Serial.println("");
}
struct sphere
{
  double r;
  double theta;
  double azimuth;
};
typedef struct sphere Sphere;
struct servoGeometry
{
  //coordinates of a vector starting at the center of the axis of rotation of the servo controlling pitch(m1) and ending at the sensor where measurments are taken from
  double pitch_x;
  double pitch_y;
  double pitch_z;
  //coordinates of a vector starting at the center of the axis of rotation of the servo controlling yaw(m2) and ending at the center of the axis of rotation of the servo controlling pitch
  double yaw_x;
  double yaw_y;
  double yaw_z;
  //the coordinates of the center of the axis of rotation of the servo controlling yaw relative to space
  double x;
  double y;
  double z;
};
typedef struct servoGeometry ServoGeometry;

void calculateOffset(ServoGeometry &geo, double theta, double azimuth, vector &offsets)
{
  theta *= -1;
  theta += (HALF_PI);
  azimuth -= HALF_PI;

  //https://stackoverflow.com/questions/14607640/rotating-a-vector-in-3d-space
  //calculate the new vector of m1 after rotating to the given theta
  //uses a rotation matrix rotating about the x-axis
  double tempx = geo.pitch_x;
  double tempy = geo.pitch_y * cos(theta) - geo.pitch_z * sin(theta);
  double tempz = geo.pitch_y * sin(theta) + geo.pitch_z * cos(theta);
  //combine vectors
  tempx += geo.yaw_x;
  tempy += geo.yaw_y;
  tempz += geo.yaw_z;
  //calculate the new vector of m2 after rotating to the given azimuth
  //uses a rotation matrix rotating about the z-axis
  double temp = tempx * cos(azimuth) - tempy * sin(azimuth);
  double temp2 = tempx * sin(azimuth) - tempy * cos(azimuth);
  tempx = temp;
  tempy = temp2;
  //add hard offsets from reference frame and store it to the vector offsets
  offsets.x = (tempx + geo.x);
  offsets.y = tempy + geo.y;
  offsets.z = (tempz + geo.z);
}
vector sphereToVector(double r, double theta, double azimuth, vector &offsets)
{
  Vector data;
  theta *= -1;
  theta += (HALF_PI);
  azimuth -= HALF_PI;
  double temp1 = sin(theta) * r;
  data.x = temp1 * cos(azimuth) + offsets.x;
  data.y = temp1 * sin(azimuth) + offsets.z;
  data.z = r * cos(theta) + offsets.y;
  return data;
}
void printSphereToVector(double r, double theta, double azimuth, vector &offsets)
{
  theta *= -1;
  theta += (HALF_PI);
  azimuth -= HALF_PI;
  double temp1 = sin(theta) * r;
  double xtemp = temp1 * cos(azimuth) + offsets.x; //converet spherical coordinates to parametric coordintes, https://en.wikipedia.org/wiki/Spherical_coordinate_system#Cartesian_coordinates
  double ytemp = temp1 * sin(azimuth) + offsets.y;
  double ztemp = r * cos(theta) + offsets.z;
  Serial.print(xtemp, 5);
  Serial.print(", ");
  Serial.print(ytemp, 5);
  Serial.print(", ");
  Serial.print(ztemp, 5);

}
void printVectorDifference(vector &offsets, vector &signal)
{
  double xtemp = -1 * (signal.x - offsets.x);
  double ytemp = (signal.y);
  double ztemp = -1 * (signal.z - offsets.z);
  Serial.print(xtemp, 5);
  Serial.print(", ");
  Serial.print(ztemp, 5);
  Serial.print(", ");
  Serial.print(ytemp, 5);
}
void lsrl(double x[], double y[], int dim, double values[])
{
  double b = 0.0;
  double d = 0.0;
  for (int i = 0; i < dim; i++)
  {
    b += x[i];
    d += x[i] * x[i];
  }
  double det = 1.0;
  double a = (double)dim;
  det /= (a * d - b * b);
  swap(a, d);

  a *= det;
  b *= -1 * det;
  d *= det;

  double y0 = 0.0;
  double y1 = 0.0;
  for (int i = 0; i < dim; i++)
  {
    y0 += y[i];
    y1 += y[i] * x[i];
  }

  double b0 = a * y0;
  b0 += b * y1;
  double b1 = b * y0;
  b1 += d * y1;

  double y_mean = y0;
  y_mean /= dim;
  double error = 0.0;
  double sst = 0.0;
  for (int i = 0; i < dim; i++)
  {
    double val = b1 * x[i];
    val += b0;
    val -= y[i];
    val *= val;
    error += val;

    val = y[i] - y_mean;
    val *= val;
    sst += val;
  }
  double r = 1 - (error / sst);

  values[0] = b0;
  values[1] = b1;
  values[2] = r;

}

#endif
