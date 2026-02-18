#include "geom.h"
#include <consts.h>
#include <math.h>
#include <assert.h>

// --- 3D vector ---

vec3 vec3_sum(vec3 *v1, vec3 *v2) {
    return (vec3) {
        .x = v1->x + v2->x,
        .y = v1->y + v2->y,
        .z = v1->z + v2->z
    };
}

vec3 vec3_sub(vec3 *v1, vec3 *v2) {
    return (vec3) {
        .x = v1->x - v2->x,
        .y = v1->y - v2->y,
        .z = v1->z - v2->z
    };
}

vec3 vec3_dot(vec3 *v1, vec3 *v2) {
    return (vec3) {
        .x = v1->x * v2->x,
        .y = v1->y * v2->y,
        .z = v1->z * v2->z
    };
}

vec3 vec3_cross(vec3 *v1, vec3 *v2) {
    return (vec3) {
        .x = v1->y * v2->z - v1->z * v2->y,
        .y = v1->z * v2->x - v1->x * v2->z,
        .z = v1->x * v2->y - v1->y * v2->x
    };
}

vec3 vec3_scale(vec3 *v, f64 f) {
    return (vec3) {
        .x = v->x * f,
        .y = v->y * f,
        .z = v->z * f
    };
}

vec3 vec3_div(vec3 *v1, vec3 *v2) {
    return (vec3) {
        .x = (v2->x != 0.0) ? v1->x / v2->x : 0.0,
        .y = (v2->y != 0.0) ? v1->y / v2->y : 0.0,
        .z = (v2->z != 0.0) ? v1->z / v2->z : 0.0
    };
}

vec3 vec3_abs(vec3 *v) {
    return (vec3) {
        .x = fabs(v->x),
        .y = fabs(v->y),
        .z = fabs(v->z)
    };
}

vec3 vec3_norm(vec3 *v) {
    f64 norm = sqrt(v->x*v->x + v->y*v->y + v->z*v->z);
    
    return (vec3) {
        .x = v->x/norm,
        .y = v->y/norm,
        .z = v->z/norm,
    };
}

// --- Quaternions ---
// https://eater.net/quaternions
// https://stackoverflow.com/questions/49790453/enu-ned-frame-conversion-using-quaternions
// https://lisyarus.github.io/blog/posts/introduction-to-quaternions.html

quat quat_mul(quat *q1, quat *q2) {
    // q1 * q2 = | q1r q2r - q2i q2i - q1j q2j - q1j q2k |
    //           | q1r q2i + q1i q2r + q1j q2k - q1k q2j |
    //           | q1r q2j - q1i q2k + q1j q2r + q1k q2i |
    //           | q1r q2k + q1i q2j - q1j q2i + q1k q2r |
    // Quaterion multiplication is non-commutative: q1 q2 != q2 q1

    quat q;
    q.r = q1->r * q2->r - q1->i * q2->i - q1->j * q2->j - q1->k * q2->k;
    q.i = q1->r * q2->i + q1->i * q2->r + q1->j * q2->k - q1->k * q2->j;
    q.j = q1->r * q2->j - q1->i * q2->k + q1->j * q2->r + q1->k * q2->i;
    q.k = q1->r * q2->k + q1->i * q2->j - q1->j * q2->i + q1->k * q2->r;

    return q;
}

void quat_norm(quat *q) {
    f64 norm = sqrt(
        q->r*q->r + \
        q->i*q->i + \
        q->j*q->j + \
        q->k*q->k \
    );

    assert(norm > 1e-15);

    q->r /= norm;
    q->i /= norm;
    q->j /= norm;
    q->k /= norm;
}


vec3 vec3_rotate_by_quat(vec3 *v, quat *q) {
    // v_r = q v q^-1

    quat q_inv = { q->r, -q->i, -q->j, -q->k };

    quat vq = { 0.0, v->x, v->y, v->z };

    quat qv  = quat_mul(q, &vq);
    quat qvq = quat_mul(&qv, &q_inv);

    vec3 result = { qvq.i, qvq.j, qvq.k };
    return result;
}

vec3 quat_to_euler_zyx(quat *q) {
    // ZYX sequence
    // Angles are between [-PI .. PI] (atan2)
    // https://en.wikipedia.org/wiki/Atan2
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    vec3 angles = {0};

    // roll (x-axis rotation)
    f64 sinr_cosp = 2 * (q->r * q->i + q->j * q->k);
    f64 cosr_cosp = 1 - 2 * (q->i * q->i + q->j * q->j);
    angles.x = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    f64 sinp = sqrt(1 + 2 * (q->r * q->j - q->i * q->k));
    f64 cosp = sqrt(1 - 2 * (q->r * q->j - q->i * q->k));
    angles.y = 2 * atan2(sinp, cosp) - PI / 2;

    // yaw (z-axis rotation)
    f64 siny_cosp = 2 * (q->r * q->k + q->i * q->j);
    f64 cosy_cosp = 1 - 2 * (q->j * q->j + q->k * q->k);
    angles.z = atan2(siny_cosp, cosy_cosp);

    return angles;
}
