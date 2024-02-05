#include <Adafruit_Sensor.h>

struct Vector3{
  float x,y,z;
  Vector3(float x, float y, float z): x{x}, y{y}, z{z}{}
  Vector3(): x(0), y(0), z(0){}
  Vector3(sensors_vec_t vec) : x{vec.x}, y{vec.y}, z{vec.z} {}
  Vector3* operator += (Vector3 V){
      x += V.x;
      y += V.y;
      z += V.z;
      return this;
    }
  Vector3 operator * (float d){
      Vector3 P;
      P.x = x * d;
      P.y = y * d;
      P.z = z * d;
    return P;
    }
};