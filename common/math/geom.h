// vec3, quat, mat3 implementations

#ifndef GEOM_H
#define GEOM_H

#include <types.h>

typedef struct {
    double x;
    double y;
    double z;
} vec3;

vec3 vec3_sum(vec3 *v1, vec3 *v2);
vec3 vec3_sub(vec3 *v1, vec3 *v2);
vec3 vec3_dot(vec3 *v1, vec3 *v2);
vec3 vec3_cross(vec3 *v1, vec3 *v2);
vec3 vec3_scale(vec3 *v, f64 f);
vec3 vec3_div(vec3 *v1, vec3 *v2);
vec3 vec3_abs(vec3 *v);

typedef struct {
    double r;
    double i;
    double j;
    double k;
} quat;

quat quat_mul(quat *q1, quat *q2);
void quat_norm(quat *q);
vec3 vec3_rotate_quat(vec3 *v, quat *q);
vec3 quat_to_euler(quat *q);

#endif