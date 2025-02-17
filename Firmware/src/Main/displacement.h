#ifndef DISPLACEMENT_H
#define DISPLACEMENT_H

void readEncoder1();

void readEncoder6();

int get_heading(float acc[3], float mag[3], float p[3], float magdec);

void get_scaled_IMU(float Axyz[3], float Mxyz[3]);

void vector_cross(float a[3], float b[3], float out[3]);

float vector_dot(float a[3], float b[3]);

void vector_normalize(float a[3]);



#endif