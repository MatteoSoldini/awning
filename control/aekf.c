
//Mat J(Mat Xp, vec3 acc_meas, vec3 gyro_meas) {
//    vec3 omega = {
//        gyro_meas.x - Xp.data[S_OMEGA_BIAS_X],
//        gyro_meas.y - Xp.data[S_OMEGA_BIAS_Y],
//        gyro_meas.z - Xp.data[S_OMEGA_BIAS_Z]
//    };
//    
//    vec3 body_acc = {
//        .x = acc_meas.x, // - Xp.data[S_ACC_BIAS_X],
//        .y = acc_meas.y, // - Xp.data[S_ACC_BIAS_Y],
//        .z = acc_meas.z, // - Xp.data[S_ACC_BIAS_Z],
//    };
//
//    Mat J = { .r=S_STATE_DIM, .c=S_STATE_DIM };
//    
//    // --- Position ---
//    // p = p + v dt + 1/2 a dt^2
//    
//    // 1/2 a dt^2 = 1/2 R ba dt^2
//    //            = 1/2 dt^2 R [0 0 baz]
//    //            = 1/2 dt^2 |    2 baz (qi qk + qj qr)      |
//    //                       |    2 baz (qj qk - qi qr)      |
//    //                       | baz - 2 baz (qi^2 + qj^2) - G |
//    //            = dt^2 |          (az - bz) (qi qk + qj qr)          |
//    //                   |          (az - bz) (qj qk - qi qr)          |
//    //                   | 1/2 (az - bz) - (az - bz) (qi^2 + qj^2) - G |
//
//    MAT_AT(J, S_POS_X, S_POS_X) =       1;
//    MAT_AT(J, S_POS_X, S_VEL_X) =       c_dt;
//    MAT_AT(J, S_POS_X, S_QUAT_R) =      body_acc.z*Xp.data[S_QUAT_J]*c_dt*c_dt;
//    MAT_AT(J, S_POS_X, S_QUAT_I) =      body_acc.z*Xp.data[S_QUAT_K]*c_dt*c_dt;
//    MAT_AT(J, S_POS_X, S_QUAT_J) =      body_acc.z*Xp.data[S_QUAT_R]*c_dt*c_dt;
//    MAT_AT(J, S_POS_X, S_QUAT_K) =      body_acc.z*Xp.data[S_QUAT_I]*c_dt*c_dt;
//    //MAT_AT(J, S_POS_X, S_ACC_BIAS_Z) = -(Xp.data[S_QUAT_I]*Xp.data[S_QUAT_K] + Xp.data[S_QUAT_J]*Xp.data[S_QUAT_R])*c_dt*c_dt;
//
//    MAT_AT(J, S_POS_Y, S_POS_Y) =       1;
//    MAT_AT(J, S_POS_Y, S_VEL_Y) =       c_dt;
//    MAT_AT(J, S_POS_Y, S_QUAT_R) =     -body_acc.z*Xp.data[S_QUAT_I]*c_dt*c_dt;
//    MAT_AT(J, S_POS_Y, S_QUAT_I) =     -body_acc.z*Xp.data[S_QUAT_R]*c_dt*c_dt;
//    MAT_AT(J, S_POS_Y, S_QUAT_J) =      body_acc.z*Xp.data[S_QUAT_K]*c_dt*c_dt;
//    MAT_AT(J, S_POS_Y, S_QUAT_K) =      body_acc.z*Xp.data[S_QUAT_J]*c_dt*c_dt;
//    //MAT_AT(J, S_POS_Y, S_ACC_BIAS_Z) = -(Xp.data[S_QUAT_J]*Xp.data[S_QUAT_K] - Xp.data[S_QUAT_I]*Xp.data[S_QUAT_R])*c_dt*c_dt;
//    
//    MAT_AT(J, S_POS_Z, S_POS_Z) =       1;
//    MAT_AT(J, S_POS_Z, S_VEL_Z) =       c_dt;
//    MAT_AT(J, S_POS_Z, S_QUAT_I) =     -2.0*body_acc.z*Xp.data[S_QUAT_I]*c_dt*c_dt;
//    MAT_AT(J, S_POS_Z, S_QUAT_J) =     -2.0*body_acc.z*Xp.data[S_QUAT_J]*c_dt*c_dt;
//    //MAT_AT(J, S_POS_Z, S_ACC_BIAS_Z) = (-0.5 + Xp.data[S_QUAT_I]*Xp.data[S_QUAT_I] + Xp.data[S_QUAT_J]*Xp.data[S_QUAT_J])*c_dt*c_dt;
//
//    // --- Velocity ---
//    // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix
//    
//    // az = t/m
//
//    // wa = R [0, 0, baz]
//    //    = |    2 baz (qi qk + qj qr)     |
//    //      |    2 baz (qj qk - qi qr)     |
//    //      | baz - 2 baz (qi^2 + qj^2) - G |
//    
//    // v = v + wa dt
//    //   = |        vx + (2 (az - bz) (qi qk + qj qr)) dt        |
//    //     |        vy + (2 (bz - bz) (qj qk - qi qr)) dt        |
//    //     | vz + ((az - bz) - 2 (az - bz) (qi^2 + qj^2) - G) dt |
//
//    MAT_AT(J, S_VEL_X, S_VEL_X) =       1;
//    MAT_AT(J, S_VEL_X, S_QUAT_R) =      2.0*body_acc.z*Xp.data[S_QUAT_J]*c_dt;
//    MAT_AT(J, S_VEL_X, S_QUAT_I) =      2.0*body_acc.z*Xp.data[S_QUAT_K]*c_dt;
//    MAT_AT(J, S_VEL_X, S_QUAT_J) =      2.0*body_acc.z*Xp.data[S_QUAT_R]*c_dt;
//    MAT_AT(J, S_VEL_X, S_QUAT_K) =      2.0*body_acc.z*Xp.data[S_QUAT_I]*c_dt;
//    //MAT_AT(J, S_VEL_X, S_ACC_BIAS_Z) = -2.0*(Xp.data[S_QUAT_I]*Xp.data[S_QUAT_K] + Xp.data[S_QUAT_J]*Xp.data[S_QUAT_R])*c_dt;
//    
//    MAT_AT(J, S_VEL_Y, S_VEL_Y) =       1;
//    MAT_AT(J, S_VEL_Y, S_QUAT_R) =     -2.0*body_acc.z*Xp.data[S_QUAT_I]*c_dt;
//    MAT_AT(J, S_VEL_Y, S_QUAT_I) =     -2.0*body_acc.z*Xp.data[S_QUAT_R]*c_dt;
//    MAT_AT(J, S_VEL_Y, S_QUAT_J) =      2.0*body_acc.z*Xp.data[S_QUAT_K]*c_dt;
//    MAT_AT(J, S_VEL_Y, S_QUAT_K) =      2.0*body_acc.z*Xp.data[S_QUAT_J]*c_dt;
//    //MAT_AT(J, S_VEL_Y, S_ACC_BIAS_Z) = -2.0*(Xp.data[S_QUAT_J]*Xp.data[S_QUAT_K] - Xp.data[S_QUAT_I]*Xp.data[S_QUAT_R])*c_dt;
//    
//    // vz + (az - 2 az qi^2 - 2 az qj^2 - G) dt
//    MAT_AT(J, S_VEL_Z, S_VEL_Z) =       1;
//    MAT_AT(J, S_VEL_Z, S_QUAT_I) =     -4.0*body_acc.z*Xp.data[S_QUAT_I]*c_dt;
//    MAT_AT(J, S_VEL_Z, S_QUAT_J) =     -4.0*body_acc.z*Xp.data[S_QUAT_J]*c_dt;
//    //MAT_AT(J, S_VEL_Z, S_ACC_BIAS_Z) = -c_dt + 2.0*(Xp.data[S_QUAT_I]*Xp.data[S_QUAT_I] + Xp.data[S_QUAT_J]*Xp.data[S_QUAT_J])*c_dt;
//
//    //MAT_AT(J, S_ACC_BIAS_Z, S_ACC_BIAS_Z) = 1.0;
//    
//    // --- Orientation ---
//    // wq = [0, rx, ry, rz]
//    // q = q + 1/2 q wq dt
//    //     | qr + (-qi (rx - bx) - qj (ry - by) - qk (rz - bz)) 0.5 dt |
//    //   = | qi + ( qr (rx - bx) + qj (rz - bz) - qk (ry - by)) 0.5 dt |
//    //     | qj + ( qr (ry - by) - qi (rz - bz) + qk (rx - bx)) 0.5 dt |
//    //     | qk + ( qr (rz - bz) + qi (ry - by) - qj (rx - bx)) 0.5 dt |
//
//    // q.r
//    MAT_AT(J, S_QUAT_R, S_QUAT_R) =        1;
//    MAT_AT(J, S_QUAT_R, S_QUAT_I) =       -0.5*c_dt*omega.x;
//    MAT_AT(J, S_QUAT_R, S_QUAT_J) =       -0.5*c_dt*omega.y;
//    MAT_AT(J, S_QUAT_R, S_QUAT_K) =       -0.5*c_dt*omega.z;
//    MAT_AT(J, S_QUAT_R, S_OMEGA_BIAS_X) =  0.5*c_dt*Xp.data[S_QUAT_I];
//    MAT_AT(J, S_QUAT_R, S_OMEGA_BIAS_Y) =  0.5*c_dt*Xp.data[S_QUAT_J];
//    MAT_AT(J, S_QUAT_R, S_OMEGA_BIAS_Z) =  0.5*c_dt*Xp.data[S_QUAT_K];
//
//    // q.i
//    MAT_AT(J, S_QUAT_I, S_QUAT_R) =        0.5*c_dt*omega.x;
//    MAT_AT(J, S_QUAT_I, S_QUAT_I) =        1;
//    MAT_AT(J, S_QUAT_I, S_QUAT_J) =        0.5*c_dt*omega.z;
//    MAT_AT(J, S_QUAT_I, S_QUAT_K) =       -0.5*c_dt*omega.y;
//    MAT_AT(J, S_QUAT_I, S_OMEGA_BIAS_X) = -0.5*c_dt*Xp.data[S_QUAT_R];
//    MAT_AT(J, S_QUAT_I, S_OMEGA_BIAS_Y) =  0.5*c_dt*Xp.data[S_QUAT_K];
//    MAT_AT(J, S_QUAT_I, S_OMEGA_BIAS_Z) = -0.5*c_dt*Xp.data[S_QUAT_J];
//
//    // q.j
//    MAT_AT(J, S_QUAT_J, S_QUAT_R) =        0.5*c_dt*omega.y;
//    MAT_AT(J, S_QUAT_J, S_QUAT_I) =       -0.5*c_dt*omega.z;
//    MAT_AT(J, S_QUAT_J, S_QUAT_J) =        1;
//    MAT_AT(J, S_QUAT_J, S_QUAT_K) =        0.5*c_dt*omega.x;
//    MAT_AT(J, S_QUAT_J, S_OMEGA_BIAS_X) = -0.5*c_dt*Xp.data[S_QUAT_K];
//    MAT_AT(J, S_QUAT_J, S_OMEGA_BIAS_Y) = -0.5*c_dt*Xp.data[S_QUAT_R];
//    MAT_AT(J, S_QUAT_J, S_OMEGA_BIAS_Z) =  0.5*c_dt*Xp.data[S_QUAT_I];
//
//    // q.k
//    MAT_AT(J, S_QUAT_K, S_QUAT_R) =        0.5*c_dt*omega.z;
//    MAT_AT(J, S_QUAT_K, S_QUAT_I) =        0.5*c_dt*omega.y;
//    MAT_AT(J, S_QUAT_K, S_QUAT_J) =       -0.5*c_dt*omega.x;
//    MAT_AT(J, S_QUAT_K, S_QUAT_K) =        1;
//    MAT_AT(J, S_QUAT_K, S_OMEGA_BIAS_X) =  0.5*c_dt*Xp.data[S_QUAT_J];
//    MAT_AT(J, S_QUAT_K, S_OMEGA_BIAS_Y) = -0.5*c_dt*Xp.data[S_QUAT_I];
//    MAT_AT(J, S_QUAT_K, S_OMEGA_BIAS_Z) = -0.5*c_dt*Xp.data[S_QUAT_R];
//
//    // --- Omega ---
//    SET_XYZ(J, S_OMEGA_BIAS_X, S_OMEGA_BIAS_X, 1);    // Constant
//
//    return J;
//}


//// State covariance matrix
//Mat P = { .r=S_STATE_DIM, .c=S_STATE_DIM };
//
//// Process covariance matrix
//Mat Q = { .r=S_STATE_DIM, .c=S_STATE_DIM };


//void ekf_predict(vec3 acc_meas, vec3 gyro_meas) {
//    // P = J(X) P J(X)' + Q
//    Mat Jx = J(X, acc_meas, gyro_meas);
//    P = mat_mul(&Jx, &P);
//
//    Mat Jx_t = mat_trans(&Jx);
//    P = mat_mul(&P, &Jx_t);
//
//    P = mat_sum(&P, &Q);
//    
//    // X = F(X)
//    Mat new_X = F(X, acc_meas, gyro_meas);
//   
//#ifndef NDEBUG
//    Mat I = mat_sub(&new_X, &X);
//    for (i32 i = 0; i < X.r; i++) {
//        f64 vi = fabs(MAT_AT(I, i, 0));
//        
//        if (vi > 1.0) {
//            DBG_BREAK();
//        }
//
//        f64 sigma = sqrt(MAT_AT(P, i, i));   // expected stddev of residual
//        if (sigma > 1e-12 && vi > 5.0 * sigma) {
//            //DBG_BREAK();
//        }
//    }
//#endif
//
//    X = new_X;
//}
//void ekf_correct(
//    Mat *Z,     // Measurements matrix
//    Mat_fn h,   // Observation matrix
//    Mat_fn H,   // Jacobian observation matrix
//    Mat *R      // Measurement noise
//) {
//    //assert(Z->r == H->r && R->r == R->c && R->r == H->r && "Wrong arguments dimensions");
//    
//    // Compute innovation
//    // I = Z - h(X)
//    Mat hX = h(&X);
//    Mat I = mat_sub(Z, &hX);   // This is in measurement space
//
//    // Compute innovation covariance
//    // S = H(X) P H(X)' + R
//    Mat HX = H(&X);
//    Mat HX_t = mat_trans(&HX);
//
//    Mat S = mat_mul(&HX, &P);
//    S = mat_mul(&S, &HX_t);
//    S = mat_sum(&S, R);
//    
//    for (i32 i = 0; i < Z->r; i++) {
//        f64 vi = fabs(MAT_AT(I, i, 0));
//        
//        f64 sigma = sqrt(MAT_AT(S, i, i));   // expected stddev of residual
//        if (sigma > 1e-12 && vi > 5.0 * sigma) {
//#ifndef NDEBUG
//            printf("INNOV gate: i=%d I=%lf sigma=%lf (%.2f sigma)\n",
//                   i, vi, sigma, vi/sigma);
//            
//            printf("DEBUG DUMP\n");
//            printf("I: ");
//            mat_print(&I);
//            
//            printf("Z: ");
//            mat_print(Z);
//            
//            printf("X: ");
//            mat_print(&X);
//            
//            printf("P: ");
//            mat_print(&P);
//            
//            DBG_BREAK();
//#endif
//
//            return; // reject update
//        }
//    }
//
//    // Compute Kalman gain
//    // K = P H(X)' S^-1
//    Mat S_inv = mat_inv(&S);
//    
//    Mat K = mat_mul(&P, &HX_t);
//    K = mat_mul(&K, &S_inv);
//
//#ifndef NDEBUG
//    f64 K_max = 0.0;
//    for (i32 r=0; r<K.r; r++) {
//        for (i32 c=0; c<K.c; c++) {
//            if (MAT_AT(K, r, c) > fabs(K_max)) {
//                K_max = fabs(MAT_AT(K, r, c));
//            }
//        }
//    }
//
//    if (K_max >= 1.5) {
//        //printf("K_max: %lf\n", K_max);
//    }
//#endif
//
//    // Update state
//    // X = X + K I
//    Mat X_corr = mat_mul(&K, &I);
//    X = mat_sum(&X, &X_corr);
//
//    // Normalize state quaterion
//    quat q = QUAT_FROM_STATE(X);
//    quat_norm(&q);
//    
//    MAT_AT(X, S_QUAT_R, 0) = q.r;
//    MAT_AT(X, S_QUAT_I, 0) = q.i;
//    MAT_AT(X, S_QUAT_J, 0) = q.j;
//    MAT_AT(X, S_QUAT_K, 0) = q.k;
//
//    // Update covariance
//    // P = P - K S K'
//    //Mat K_t = mat_trans(&K);
//    //Mat KS = mat_mul(&K, &S);
//    //Mat KSK_t = mat_mul(&KS, &K_t);
//    //P = mat_sub(&P, &KSK_t);
//    
//    // This formula seems to be mathematically unstable and produces bad P after
//    // a while (~5 min with 12 state variables). The solution seems to be to use Joseph-form
//    // https://kalman-filter.com/joseph-form/
//    
//    // P = (Id - K H(X)) P (Id - K H(X))' + K R K'
//    // A = (Id - K H(X))
//    // P = A P A' + K R K'
//
//    Mat KHX = mat_mul(&K, &HX);
//    Mat Id = mat_identity(KHX.r);
//    Mat A = mat_sub(&Id, &KHX);
//    Mat A_t = mat_trans(&A);
//
//    Mat AP = mat_mul(&A, &P);
//    Mat APA_t = mat_mul(&AP, &A_t);
//
//    Mat K_t = mat_trans(&K);
//    Mat KR = mat_mul(&K, R);
//    Mat KRK_t = mat_mul(&KR, &K_t);
//    
//    P = mat_sum(&APA_t, &KRK_t);
//
//    // Check P
//    // * Symmetric
//    // * Semi-positive (positive diagonal)
//    for (u64 r=0; r<P.r; r++) {
//        for (u64 c=r+1; c<P.c; c++) {
//            f64 diff = MAT_AT(P, r, c) - MAT_AT(P, c, r);
//            assert(fabs(diff) < 1e-10);
//        }
//    }
//
//    for (u64 r=0; r<P.r; r++) {
//        assert(MAT_AT(P, r, r) >= 0.0);
//    }
//}