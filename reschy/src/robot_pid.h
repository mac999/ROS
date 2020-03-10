#pragma once

void initPID(float Kp = 4.0, float Ki = 1.0, float Kd = 1.0);
float calcPID(float targetValue, float currentValue, float samplingTime);
float convertValue(float controlPID, float controlRange, float min1, float max2);


