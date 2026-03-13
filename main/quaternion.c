#include "quaternion.h"

#include <math.h>


void quat_multiply(Quaternion* q_new, const Quaternion* q1, const Quaternion* q2) {
    float w = (q1->w * q2->w) - (q1->x * q2->x) - (q1->y * q2->y) - (q1->z * q2->z);

    float x = (q1->w * q2->x) + (q1->x * q2->w) + (q1->y * q2->z) - (q1->z * q2->y);

    float y = (q1->w * q2->y) - (q1->x * q2->z) + (q1->y * q2->w) + (q1->z * q2->x);

    float z = (q1->w * q2->z) + (q1->x * q2->y) - (q1->y * q2->x) + (q1->z * q2->w);

    q_new->w = w;
    q_new->x = x;
    q_new->y = y;
    q_new->z = z;

    quat_normalize(q_new);
}

float quat_magnitude(const Quaternion* q) {
    return sqrtf(powf(q->w, 2.0f) + powf(q->x, 2.0f) + powf(q->y, 2.0f) + powf(q->z, 2.0f));
}

void quat_normalize(Quaternion* q) {
    float magnitude = quat_magnitude(q);

    q->w /= magnitude; 
    q->x /= magnitude;
    q->y /= magnitude;
    q->z /= magnitude;
}


void quat_from_angle_axis(Quaternion* q_new, float angle, float x, float y, float z) {
    float half_angle = angle / 2.0f;

    // Normalize axis
    float axis_magnitude = sqrtf(powf(x, 2.0f) + powf(y, 2.0f) + powf(z, 2.0f));
    x /= axis_magnitude;
    y /= axis_magnitude;
    z /= axis_magnitude;


    // Create quaternion
    q_new->w = cosf(half_angle);
    q_new->x = x * sinf(half_angle);
    q_new->y = y * sinf(half_angle);
    q_new->z = z * sinf(half_angle);

    quat_normalize(q_new);
}
