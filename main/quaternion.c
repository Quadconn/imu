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
    return sqrtf((q->w * q->w) + (q->x * q->x) + (q->y * q->y) + (q->z * q->z));
}

void quat_normalize(Quaternion* q) {
    float magnitude = quat_magnitude(q);

    // Division by zero protection
    if (magnitude < 1E-6f) return;

    q->w /= magnitude; 
    q->x /= magnitude;
    q->y /= magnitude;
    q->z /= magnitude;
}


void quat_from_angle_axis(Quaternion* q_new, float angle, float x, float y, float z) {
    float half_angle = angle / 2.0f;

    // Normalize axis
    float axis_magnitude = sqrtf((x * x) + (y * y) + (z * z));
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

void quat_to_euler(Euler* e, const Quaternion* q) {
    e->roll = atan2f(2.0f * (q->w * q->x + q->y * q->z), 
                     (q->w * q->w) - (q->x * q->x) - (q->y * q->y) + (q->z * q->z));

    e->pitch = asinf(2.0f * (q->w * q->y - q->x * q->z));

    e->yaw = atan2f(2 * (q->w * q->z + q->x * q->y), 
                     (q->w * q->w) + (q->x * q->x) - (q->y * q->y) - (q->z * q->z));

    // Avoid gimbal lock (when pitch = +/- pi/2 arguments to roll and pitch become 0 which 
    // is undefined for atan2())

    if (fabsf(e->pitch - ((float)M_PI / 2.0f)) <= 1E-3f) {
        e->roll = 0.0f;
        e->yaw = -2.0f * atan2f(q->x, q->w);

    } else if (fabsf(e->pitch - ((float)-M_PI / 2.0f)) <= 1E-3f) {
        e->roll = 0.0f;
        e->yaw = 2.0f * atan2f(q->x, q->w);

    }
}
