
//void kf_predict(
//    Mat *B, // Control transition matrix
//    Mat *U  // Control matrix
//) {
//    // Predict new state
//    // X = F*X + B*U
//    Mat FX = mat_mul(&F, &X);
//    Mat BU = mat_mul(B, U);
//    X = mat_sum(&FX, &BU);
//    
//    // Predict state covariance
//    // P(n+1) = F*P(n)*F_t + Q
//    Mat F_t = mat_trans(&F);
//    P = mat_mul(&F, &P);
//    P = mat_mul(&P, &F_t);
//    P = mat_sum(&P, &Q);
//}
//
//void kf_correct(
//    Mat *Z, // Measurements matrix
//    Mat *H, // Observation matrix
//    Mat *R  // Measurement noise
//) {
//    assert(Z->r == H->r && R->r == R->c && R->r == H->r && "Wrong arguments dimensions");
//
//    // Map state domain to measurements domain
//    Mat HX = mat_mul(H, &X);
//
//    // Compute innovation
//    Mat I = mat_sub(Z, &HX);   // This is in measurement space
//
//    // Compute innovation covariance
//    // S = H * P_pred * H_t + R
//    Mat H_t = mat_trans(H);
//    Mat HP = mat_mul(H, &P);
//    Mat S = mat_mul(&HP, &H_t);
//
//    S = mat_sum(&S, R);
//    
//    Mat S_inv = mat_inv(&S);
//
//    // Compute Kalman gain
//    // K = P_pred*H_t*S^-1
//    Mat K = mat_mul(&P, &H_t);
//    K = mat_mul(&K, &S_inv);
//    
//    // Update state
//    // X = X_pred + K * I
//    Mat X_corr = mat_mul(&K, &I);
//    X = mat_sum(&X, &X_corr);
//
//    // Update covariance
//    // P = (Id - K * H) * P_pred
//    Mat KH = mat_mul(&K, H);   // Map Kalman Gain from measurement domain to state domain
//    Mat Id = mat_identity(KH.r);
//    Mat IdminKH = mat_sub(&Id, &KH);
//    P = mat_mul(&IdminKH, &P);
//}