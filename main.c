// https://winter.dev/articles/physics-engine
// Ian Millington's Game Physics Engine Development: https://www.r-5.org/files/books/computers/algo-list/realtime-3d/Ian_Millington-Game_Physics_Engine_Development-EN.pdf

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <time.h>

#include "raylib.h"
#include "raymath.h"
#include <raygui.h>
#include "rlgl.h"

#define CAM_MIN_RADIUS 0.5
#define CAM_SCROLL_SPEED 1.0

// App state
float c_radius = 15.0f;
float c_yaw_rad = 45.0f * DEG2RAD;
float c_pitch_rad = 30.0f * DEG2RAD;

// Physics
// TODO: Implement fixed timestep (ex. 1khz).
//       We should consider moving physics to a different thread.

// The physics world assumes Z up

typedef struct {
    double x;
    double y;
    double z;
} p_vec3;

typedef struct {
    double r;
    double i;
    double j;
    double k;
} p_quat;

void p_quat_to_mtx4(float mtx[16], p_quat q) {
    float r = (float)q.r;
    float i = (float)q.i;
    float j = (float)q.j;
    float k = (float)q.k;

    mtx[0]  = 1 - 2*j*j - 2*k*k;
    mtx[1]  = 2*i*j + 2*k*r;
    mtx[2]  = 2*i*k - 2*j*r;
    mtx[3]  = 0;

    mtx[4]  = 2*i*j - 2*k*r;
    mtx[5]  = 1 - 2*i*i - 2*k*k;
    mtx[6]  = 2*j*k + 2*i*r;
    mtx[7]  = 0;

    mtx[8]  = 2*i*k + 2*j*r;
    mtx[9]  = 2*j*k - 2*i*r;
    mtx[10] = 1 - 2*i*i - 2*j*j;
    mtx[11] = 0;

    mtx[12] = 0;
    mtx[13] = 0;
    mtx[14] = 0;
    mtx[15] = 1;
}

void p_set_quat_to_axis_angle(p_quat *quat, p_vec3 *axis, double angle_rad) {
    quat->r = cosf(angle_rad / 2.0f);
    quat->i = axis->x * sinf(angle_rad / 2.0f);
    quat->j = axis->y * sinf(angle_rad / 2.0f);
    quat->k = axis->z * sinf(angle_rad / 2.0f);
}

p_quat p_quat_mul(p_quat *q1, p_quat *q2) {
    p_quat q;
    q.r = q1->r * q2->r - q1->i * q2->i - q1->j * q2->j - q1->k * q2->k;
    q.i = q1->r * q2->i + q1->i * q2->r + q1->j * q2->k - q1->k * q2->j;
    q.j = q1->r * q2->j - q1->i * q2->k + q1->j * q2->r + q1->k * q2->i;
    q.k = q1->r * q2->k + q1->i * q2->j - q1->j * q2->i + q1->k * q2->r;

    return q;
}

void p_quat_norm(p_quat *q) {
    double norm = sqrt(
        q->r*q->r + \
        q->i*q->i + \
        q->j*q->j + \
        q->k*q->k \
    );

    q->r /= norm;
    q->i /= norm;
    q->j /= norm;
    q->k /= norm;
}

p_vec3 p_vec_rotate_quat(p_vec3 *v, p_quat *q) {
    // v_r = q * v * q^-1

    p_quat q_inv = { q->r, -q->i, -q->j, -q->k };

    p_quat vq = { 0.0, v->x, v->y, v->z };

    p_quat qv  = p_quat_mul(q, &vq);
    p_quat qvq = p_quat_mul(&qv, &q_inv);

    p_vec3 result = { qvq.i, qvq.j, qvq.k };
    return result;
}

p_vec3 p_vec_sum(p_vec3 *v1, p_vec3 *v2) {
    return (p_vec3) {
        .x = v1->x + v2->x,
        .y = v1->y + v2->y,
        .z = v1->z + v2->z
    };
}

p_vec3 p_vec_dot(p_vec3 *v1, p_vec3 *v2) {
    return (p_vec3) {
        .x = v1->x * v2->x,
        .y = v1->y * v2->y,
        .z = v1->z * v2->z
    };
}

p_vec3 p_vec_cross(p_vec3 *v1, p_vec3 *v2) {
    return (p_vec3) {
        .x = v1->y * v2->z - v1->z * v2->y,
        .y = v1->z * v2->x - v1->x * v2->z,
        .z = v1->x * v2->y - v1->y * v2->x
    };
}

p_vec3 p_vec_scale(p_vec3 *v, double f) {
    return (p_vec3) {
        .x = v->x * f,
        .y = v->y * f,
        .z = v->z * f
    };
}

p_vec3 p_vec_div(p_vec3 *v1, p_vec3 *v2) {
    return (p_vec3) {
        .x = (v2->x != 0.0) ? v1->x / v2->x : 0.0,
        .y = (v2->y != 0.0) ? v1->y / v2->y : 0.0,
        .z = (v2->z != 0.0) ? v1->z / v2->z : 0.0
    };
}

typedef struct {
    p_vec3 pos;     // m
    p_vec3 vel;     // m/s

    p_quat ori;     // Orientation (quaternion)
                    // Quaternion are prefered in order to avoid gimbal lock
                    // It is similar to axis-angle representation
    p_vec3 rot;     // rad/s
    
    double mass;    // Kg
    p_vec3 inertia; // Kg*m^2. It assumes a symmetric body
} p_rigid_body;

// Constants
const float g = 9.81; // m/s^2
uint64_t p_last_mc = 0;

// Quadcopter state
const double kf = 1e-2;     // Thrust coefficient: (N per (rad/s)^2)
const double arm_l = 0.2;   // Arm length (m)

float base_w = 25.0f;

double rot_w[4] = {
    0.0, 0.0, 0.0, 0.0
};  // Rotor angular velocities (rad/s)

p_rigid_body obj = {
    .pos = { 0.0, 0.0, 0.0},
    .vel = { 0.0, 0.0, 0.0},
    .ori = { 1.0, 0.0, 0.0, 0.0},   // (this should be normalized to 1)
    .rot = { 0.0, 0.0, 0.0},
    .mass = 2.0,
    .inertia = { 0.2, 0.2, 0.4 }
};

uint64_t get_micros() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
}

void p_init() {
    p_last_mc = get_micros();
}

void p_update() {
    uint64_t mc = get_micros();
    float dt = (float)(mc - p_last_mc) / 1e6; // s
    p_last_mc = mc;

    // Compute motors thrust
    // TODO: rotor dynamics, turbulence, ground effect
    // TODO: Consider using RK4 integrator
    //
    //         ^
    //       x |
    //    
    //    cw        ccw
    //    ->        <-
    //    M0        M1
    //       \    /
    //        \  /        y
    //         ||         ->
    //        /  \ 
    //       /    \ 
    //    M3        M2
    //    ->        <-
    //    ccw       cw
    
    double s = sqrt(arm_l);
    p_vec3 arm_dir[4] = {
        {.x = s,  .y = -s, .z = 0.0 }, // M0
        {.x = s, . y = s,  .z = 0.0 }, // M1
        {.x = -s, .y = s,  .z = 0.0 }, // M2
        {.x = -s, .y = -s, .z = 0.0 }  // M3
    };
    
    // For now we assume ideal thrust model: f = kf * rot_w^2
    p_vec3 mot_f[4] = {0};
    for (size_t i=0; i<4; i++) {
        mot_f[i].z = kf * rot_w[i] * rot_w[i];
    }

    p_vec3 tot_mot_f = { 0.0, 0.0, 0.0 };
    for (size_t i=0; i<4; i++) {
        tot_mot_f = p_vec_sum(&tot_mot_f, &mot_f[i]);
    }

    // Rotate the thrust to the object orientation
    p_vec3 tot_f = p_vec_rotate_quat(&tot_mot_f, &obj.ori);
    p_vec3 obj_acc = p_vec_scale(&tot_f, 1.0/obj.mass);
    obj_acc.z -= g;

    // Linear integrator
    // v = v_0 + a*dt
    // x = x_0 + v*dt
    p_vec3 d_vel = p_vec_scale(&obj_acc, dt);
    obj.vel = p_vec_sum(&obj.vel, &d_vel);
    p_vec3 d_pos = p_vec_scale(&obj.vel, dt);
    obj.pos = p_vec_sum(&obj.pos, &d_pos);
    
    // torque = r x F (Nm = Kg*m^2/s^2)
    p_vec3 mot_trq[4] = {0};
    for (size_t i=0; i<4; i++) {
        mot_trq[i] = p_vec_cross(&arm_dir[i], &mot_f[i]);
    }

    p_vec3 tot_trq = { 0.0, 0.0, 0.0 };
    for (size_t i=0; i<4; i++) {
        tot_trq = p_vec_sum(&tot_trq, &mot_trq[i]);
    }

    // The change in angular velocity depends on two things:
    //     we have torque (rather than force),
    //     and the moment of inertia (rather than mass)
    // The moment of inertia depends on the mass of the object and the distance
    // of the mass from the axis of rotation.
    
    // ang_acc = torque / inertia
    // Kg*m^2/s^2 / Kg*m^2 = 1/s^2 = rad/s^2
    p_vec3 obj_ang_acc = p_vec_div(&tot_trq, &obj.inertia); 

    // rot += ang_acc * dt
    p_vec3 d_rot = p_vec_scale(&obj_ang_acc, dt);
    obj.rot = p_vec_sum(&obj.rot, &d_rot);
    
    // Rotational integrator
    p_quat wq = { 0.0, obj.rot.x, obj.rot.y, obj.rot.z };
    p_quat dq = p_quat_mul(&obj.ori, &wq);
    dq.r *= 1.0/2 * dt;
    dq.i *= 1.0/2 * dt;
    dq.j *= 1.0/2 * dt;
    dq.k *= 1.0/2 * dt;

    obj.ori.r += dq.r;
    obj.ori.i += dq.i;
    obj.ori.j += dq.j;
    obj.ori.k += dq.k;

    p_quat_norm(&obj.ori);  // Crazy things happens if you don't normalize this

    //printf("%lf %lf %lf %lf\n", obj.ori.r, obj.ori.i, obj.ori.j, obj.ori.k);
}

int main(void) {
    const int screenWidth = 1280;
    const int screenHeight = 720;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "Awning");

    Camera3D camera = { 0 };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera mode type

    //SetTargetFPS(60);

    p_init();

    // Main game loop
    while (!WindowShouldClose()) {
        for (size_t i=0; i<4; i++) {
            rot_w[i] = base_w;
        }
        
        if (IsKeyDown(KEY_W)) {
            rot_w[0] -= 1.0;
            rot_w[1] -= 1.0;
            rot_w[2] += 1.0;
            rot_w[3] += 1.0;
        }
        
        if (IsKeyDown(KEY_D)) {
            rot_w[0] += 1.0;
            rot_w[1] -= 1.0;
            rot_w[2] -= 1.0;
            rot_w[3] += 1.0;
        }
        
        if (IsKeyDown(KEY_A)) {
            rot_w[0] -= 1.0;
            rot_w[1] += 1.0;
            rot_w[2] += 1.0;
            rot_w[3] -= 1.0;
        }
        
        if (IsKeyDown(KEY_S)) {
            rot_w[0] += 1.0;
            rot_w[1] += 1.0;
            rot_w[2] -= 1.0;
            rot_w[3] -= 1.0;
        }

        p_update();

        int win_w = GetScreenWidth();
        int win_h = GetScreenHeight();

        if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
            Vector2 mouse_delta = GetMouseDelta();

            c_yaw_rad   += mouse_delta.x / win_w * 2 * PI;
            c_pitch_rad += mouse_delta.y / win_h * PI;

            if (c_pitch_rad > PI/2.0f - 0.1f) c_pitch_rad = PI/2.0f - 0.1f;
            if (c_pitch_rad < -PI/2.0f + 0.1f) c_pitch_rad = -PI/2.0f + 0.1f;
        }

        // if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        //     Vector2 mouse_delta = GetMouseDelta();
        //     
        //     Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
        //     Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, camera.up));
        //     Vector3 up = Vector3Normalize(Vector3CrossProduct(right, forward));

        //     float pan_scale = (c_radius * tanf(camera.fovy * 0.5f * DEG2RAD)) / (GetScreenHeight() * 0.5f);

        //     Vector3 pan = Vector3Add(
        //         Vector3Scale(right, -mouse_delta.x * pan_scale),
        //         Vector3Scale(up, mouse_delta.y * pan_scale)
        //     );

        //     camera.target = Vector3Add(camera.target, pan);
        //     camera.position = Vector3Add(camera.position, pan);
        // }

        c_radius -= GetMouseWheelMove() * CAM_SCROLL_SPEED;
        if (c_radius < CAM_MIN_RADIUS) c_radius = CAM_MIN_RADIUS;
        
        camera.target.x = obj.pos.x;
        camera.target.y = obj.pos.z;
        camera.target.z = obj.pos.y;

        camera.position.x = camera.target.x + c_radius * cosf(c_pitch_rad) * cosf(c_yaw_rad);
        camera.position.y = camera.target.y + c_radius * sinf(c_pitch_rad);
        camera.position.z = camera.target.z + c_radius * cosf(c_pitch_rad) * sinf(c_yaw_rad);
        
        BeginDrawing();
            ClearBackground((Color){ 30, 30, 30, 255 });

            BeginMode3D(camera);
                rlPushMatrix();
                    rlTranslatef(obj.pos.x, obj.pos.z, obj.pos.y);

                    p_quat rl_obj_rot = {
                        .r = obj.ori.r,
                        .i = -obj.ori.i,
                        .j = -obj.ori.k,
                        .k = -obj.ori.j
                    };
                    float rot_mtx[16];
                    p_quat_to_mtx4(rot_mtx, rl_obj_rot);

                    rlMultMatrixf(rot_mtx);

                    DrawCube((Vector3){ 0.0f, 0.0f, 0.0f }, 2.0f, 1.0f, 2.0f, RED);
                    DrawCubeWires((Vector3){ 0.0f, 0.0f, 0.0f }, 2.0f, 1.0f, 2.0f, MAROON);
                rlPopMatrix();
                
                // Floor tiles
                const int floor_extent = 25;
                const float tile_size = 1.0f;
                for (int y = -floor_extent; y < floor_extent; y++) {
                    for (int x = -floor_extent; x < floor_extent; x++) {
                        if ((y + x) & 1) {
                            DrawPlane((Vector3){ x*tile_size, 0.0f, y*tile_size}, (Vector2){ tile_size, tile_size }, WHITE);
                        }
                        else {
                            DrawPlane((Vector3){ x*tile_size, 0.0f, y*tile_size}, (Vector2){ tile_size, tile_size }, LIGHTGRAY);
                        }
                    }
                }

                DrawGrid(10, 1.0f);
            EndMode3D();

            GuiSlider((Rectangle){ 0.0, 0.0, 300.0, 20.0 }, "Min", "Max", (float *)&base_w, 0.0, 50.0);
            
            DrawFPS(win_w - 100, 10);

            char pos_txt[64];
            sprintf(pos_txt, "X: %10.2lf", obj.pos.x);
            DrawText(pos_txt, 10, 30, 24, WHITE);
            sprintf(pos_txt, "Y: %10.2lf", obj.pos.y);
            DrawText(pos_txt, 210, 30, 24, WHITE);
            sprintf(pos_txt, "Z: %10.2lf", obj.pos.z);
            DrawText(pos_txt, 410, 30, 24, WHITE);
        EndDrawing();
    }

    CloseWindow();        // Close window and OpenGL context

    return 0;
}