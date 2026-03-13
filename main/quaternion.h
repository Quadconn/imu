#ifndef QUATERNION_H
#define QUATERNION_H

#define QUAT_IDENTITY (Quaternion){1.0f, 0.0f, 0.0f, 0.0f}

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

void quat_multiply(Quaternion* q_new, const Quaternion* q1, const Quaternion* q2);

float quat_magnitude(const Quaternion* q);

void quat_normalize(Quaternion* q);

void quat_from_angle_axis(Quaternion* q_new, float angle, float x, float y, float z);

#endif
