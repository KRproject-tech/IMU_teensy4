
MatrixXf  I_mat = MatrixXf::Identity(3,3);          // Identity matrix
MatrixXf  I_mat_77 = MatrixXf::Identity(7,7);       // Identity matrix



// EKF parameters
VectorXf g_vec = VectorXf::Zero(3);                 // Gravity vector g [m/s^2]
VectorXf m_vec = VectorXf::Zero(3);                 // Magnetic vector m [a.u.]
MatrixXf A_omega_b = MatrixXf::Zero(3,3);           // System matrix for the bias model
MatrixXf W_mat = MatrixXf::Zero(6,6);               // Process noise
MatrixXf V_mat = MatrixXf::Zero(6,6);               // Measurement noise




//---------------------- Initialization ----------------------
void init_EKF(){

  int i;

  // Euler parameter [-]
  VectorXf eps_vec0(4);  
  // Initialization of Euler parameter: eps0
  eps_vec0 <<  1, 
               0, 
               0, 
               0;

  // 角速度バイアス [rad/s]
  float Omega_b_x0 = -0.00409;
  float Omega_b_y0 = -0.00194;
  float Omega_b_z0 = -0.00021;            

  // initialize EKF parameters.
  // Gravity vector g [m/s^2]
  g_vec <<  0,
            0,
            -G_ACC;
            
  // Magnetic vector m [a.u.]    
  // 地磁気 [a.u.]
  float Mag_x = 0.0;
  float Mag_y = 0.0;
  float Mag_z = 0.0;
  // 平均
  float Mag_x_mean = 0.0;
  float Mag_y_mean = 0.0;
  float Mag_z_mean = 0.0;

  for(i=0;i<5;i++){
      digitalWrite(ledPin, HIGH);   // set the LED on
      read_Mag( &Mag_x, &Mag_y, &Mag_z);
      delay( 50);
      
      digitalWrite(ledPin, LOW);   // set the LED on
      Mag_x_mean += Mag_x/5.0;
      Mag_y_mean += Mag_y/5.0;
      Mag_z_mean += Mag_z/5.0;
      delay( 50);
  }  
  m_vec <<  -(Mag_y_mean - MAG_Y_C),
             (Mag_x_mean - MAG_X_C),
             (Mag_z_mean - MAG_Z_C);      
  m_vec.normalize();
             
            
  // Initial error covariance matrix P_0
  P_mat.diagonal() <<  1e-8, 1e-8, 1e-8, 1e-8, 
                       1e-6, 1e-6, 1e-6;
  // System matrix for the bias model
  A_omega_b.diagonal() << -1e-4, -1e-4, -1e-4;


  // 加速度センサの分散 [(m/s^2)^2]
  float sigma2_Ax = 0.000729;
  float sigma2_Ay = 0.000979;
  float sigma2_Az = 0.002046;
  // ジャイロセンサの分散 [(rad/s)^2]
  float sigma2_omega_x = 2.06e-6;
  float sigma2_omega_y = 2.28e-6;
  float sigma2_omega_z = 1.34e-6;

  // Process noise
  W_mat.diagonal() <<  sigma2_omega_x, sigma2_omega_y, sigma2_omega_z,  // Angular velocity noise
                       1e-6, 1e-6, 1e-6;                                // Angular velocity bias
  // Measurement noise
  V_mat.diagonal() <<  sigma2_Ax, sigma2_Ay, sigma2_Az,
                       1e-4, 1e-4, 1e-4;                     

  // Initialize of state vector: x = x0
  x_vec <<  eps_vec0,
            Omega_b_x0,
            Omega_b_y0,
            Omega_b_z0;
}



//---------------------- Attitude estimation of EKF  -------------------------
void EKF( VectorXf &x_v, VectorXf &h_vec, MatrixXf &P_mat_v, const VectorXf &omega_m_vec, const VectorXf &acc_m_vec, const VectorXf &mag_m_vec){
  
  // Euler parameter [-]
  VectorXf eps_vec(4); 
  
  // x_hat_k-1
  VectorXf x_hat_km1(7);  
  // x_hat-_k
  VectorXf x_hat_m_k(7);
  // x_hat_k
  VectorXf x_hat_k(7);
  // b_k
  MatrixXf b_mat_v = MatrixXf::Zero(7,6);
  // A_k-1
  MatrixXf A_mat_v = MatrixXf::Zero(7,7);
  // P-_k
  MatrixXf Pmk_mat_v = MatrixXf::Zero(7,7);  
  // c_k^T
  MatrixXf cT_mat_v = MatrixXf::Zero(6,7);
  // K_k
  MatrixXf K_k(7,6);
  // c_k^T*P-_k*c_k + V_k
  MatrixXf c_P_ck_p_Vk(6,6);
  // y_k
  VectorXf y_k(6);


  // x_hat_k-1
  x_hat_km1 = x_v;

  // x-_hat_k = fd( x_hat_k-1, u_k-1)
  x_hat_m_k = x_v;
  fd_func( x_hat_m_k, omega_m_vec);  
  
  // bd_k := b*d_T
  b_func( b_mat_v, x_hat_km1);

  // A_k-1 := dx_f
  A_func( A_mat_v, x_hat_km1, omega_m_vec);
  
  // c_k^T := dx_h
  cT_func( cT_mat_v, x_hat_m_k, g_vec, m_vec);
  
  // P-_k = A_k-1*P_k-1*A_k-1^T + b*W*b^T
  Pmk_mat_v = A_mat_v*P_mat_v*A_mat_v.transpose() + b_mat_v*W_mat*b_mat_v.transpose();

  // K_k = P-_k*c_k(c_k^T*P-_k*c_k + v_k*v_k^T)^-1
  c_P_ck_p_Vk = cT_mat_v*Pmk_mat_v*cT_mat_v.transpose() + V_mat;
  K_k = Pmk_mat_v*cT_mat_v.transpose()*( c_P_ck_p_Vk.inverse() );

  // x_hat_k = x-_hat_k + K_k*(y_k - h(x-_hat_k))
  h_func( h_vec, x_hat_m_k, g_vec, m_vec);
  y_k <<  acc_m_vec,
          mag_m_vec;
  x_hat_k = x_hat_m_k + K_k*(y_k - h_vec);

  // Normalization ( eps/||eps|| )
  eps_vec = x_hat_k.segment(0,4);

  x_v <<  eps_vec.normalized(),
          x_hat_k.segment(4,3);
            

  // P_k = (I - K_k*c_k^T)*P-_k
  P_mat_v = Pmk_mat_v - K_k*cT_mat_v*Pmk_mat_v;

  
  
}




//---------------------- EKF 関数 ----------------------
// L( eps)
void L_func( MatrixXf &L_mat, const VectorXf &eps_v){

  MatrixXf p(3,1);
  p = eps_v.segment(1,3);
  MatrixXf tild_p(3,3);
  tild_p <<       0,  -p(2,0),   p(1,0),
             p(2,0),        0,  -p(0,0),
            -p(1,0),   p(0,0),        0;

  L_mat << (-p), eps_v(0)*I_mat - tild_p;
}


// f( x, u)
void f_func( VectorXf &f_v, const VectorXf &x_v, const VectorXf &omega_m_v){

  // Euler parameter [-]
  Vector4f eps_v = x_v.segment(0,4);    
  // Bias angular velocity [rad/s]    
  Vector3f omega_b_v = x_v.segment(4,3); 
  // L(eps)
  MatrixXf L_mat = MatrixXf::Zero(3,4);
  

  L_func( L_mat, eps_v);

  f_v <<  1/2.0*( L_mat.transpose() )*(omega_m_v - omega_b_v),
          A_omega_b*omega_b_v                               ;

}

// fd( x, u)
void fd_func( VectorXf &x_v, const VectorXf &omega_m_v){

  // x + k1*dT
  VectorXf x_p_k1dT = VectorXf::Zero(7);
  // Euler parameter [-]
  VectorXf eps_vec(4); 
  
  // f(x,u)
  VectorXf k1_v = VectorXf::Zero(7);
  VectorXf k2_v = VectorXf::Zero(7);
  

  // k1
  f_func( k1_v, x_v, omega_m_v);
  
  // k2
  x_p_k1dT = x_v + k1_v*INT_T*1e-3;
  
  //Normalize
  eps_vec = x_p_k1dT.segment(0,4);
  x_p_k1dT <<   eps_vec.normalized(),
                x_p_k1dT.segment(4,3);
          
  f_func( k2_v, x_p_k1dT, omega_m_v);
 

  x_v = x_v + 1/2.0*(k1_v + k2_v)*INT_T*1e-3;        
}



// bd_k := b*d_T
void b_func( MatrixXf &b_mat_v, const VectorXf &x_v){
  
  // Euler parameter [-]
  Vector4f eps_v = x_v.segment(0,4); 
  // L(eps)
  MatrixXf L_mat = MatrixXf::Zero(3,4);  
  // 0
  MatrixXf zero_mat1 = MatrixXf::Zero(4,3);
  // 0
  MatrixXf zero_mat2 = MatrixXf::Zero(3,3);

  L_func( L_mat, eps_v);

  b_mat_v <<  -1/2.0*L_mat.transpose(), zero_mat1,
                             zero_mat2,     I_mat; 

  // b_k := b*d_T
  b_mat_v = INT_T*1e-3*b_mat_v;                           
}



// h( x)
void h_func( VectorXf &h_v , const VectorXf &x_v, const VectorXf &g_v, const VectorXf &m_v){

  // Euler parameter [-]
  Vector4f eps_v = x_v.segment(0,4); 
  // p_vec 
  Vector3f p = eps_v.segment(1,3);
  // ~p_vec
  MatrixXf tild_p(3,3);
  tild_p <<     0,  -p(2),   p(1),
             p(2),      0,  -p(0),
            -p(1),   p(0),      0;
  // A( eps)
  MatrixXf A_mat(3,3);

  A_mat = I_mat + 2.0*tild_p*(tild_p + eps_v(0)*I_mat);

  // h = [ A^T*g
  //       A^T*m]
  h_v <<  A_mat.transpose()*g_v,
          A_mat.transpose()*m_v;  
}



// d_eps( L(eps)^T*(omega_m - omega_b) )
void de_L_omega( MatrixXf &de_L_omega_mat, const VectorXf &x_v, const VectorXf &omega_m_v){

  // Bias angular velocity [rad/s]    
  Vector3f omega_b_v = x_v.segment(4,3); 

  de_L_omega_mat <<                           0,  -(omega_m_v(0) - omega_b_v(0)), -(omega_m_v(1) - omega_b_v(1)), -(omega_m_v(2) - omega_b_v(2)), 
                    omega_m_v(0) - omega_b_v(0),                               0,    omega_m_v(2) - omega_b_v(2), -(omega_m_v(1) - omega_b_v(1)),
                    omega_m_v(1) - omega_b_v(1),  -(omega_m_v(2) - omega_b_v(2)),                              0,    omega_m_v(0) - omega_b_v(0),
                    omega_m_v(2) - omega_b_v(2),     omega_m_v(1) - omega_b_v(1), -(omega_m_v(0) - omega_b_v(0)),                              0;
}

// a := dx_f
void a_func( MatrixXf &A_mat, const VectorXf &x_v, const VectorXf &omega_m_v){

  // Euler parameter [-]
  Vector4f eps_v = x_v.segment(0,4);  
  // d_eps( L(eps)^T*(omega_m - omega_b) )
  MatrixXf de_L_omega_mat = MatrixXf::Zero(4,4); 
  // L(eps)
  MatrixXf L_mat = MatrixXf::Zero(3,4);  
  // 0
  MatrixXf zero_mat = MatrixXf::Zero(3,4);  
  

  L_func( L_mat, eps_v);
  de_L_omega( de_L_omega_mat, x_v, omega_m_v);
  

  A_mat << 1/2.0*de_L_omega_mat, -1/2.0*L_mat.transpose(),
                       zero_mat,                A_omega_b; 
}

// A_k-1 := dx_fd
void A_func( MatrixXf &A_mat, const VectorXf &x_v, const VectorXf &omega_m_v){

  // x + k1*dT
  VectorXf x_p_k1dT = VectorXf::Zero(7);
  // Euler parameter [-]
  VectorXf eps_vec(4); 

  // f(x,u)
  VectorXf k1_v = VectorXf::Zero(7);
  // A: = dx_f
  MatrixXf k1_mat = MatrixXf::Zero(7,7); 
  MatrixXf k2_mat = MatrixXf::Zero(7,7); 

  // k1
  f_func( k1_v, x_v, omega_m_v);
  
  // K1
  a_func( k1_mat, x_v, omega_m_v);
  
  // K2
  x_p_k1dT = x_v + k1_v*INT_T*1e-3;
  
  //Normalize
  eps_vec = x_p_k1dT.segment(0,4);
  x_p_k1dT <<   eps_vec.normalized(),
                x_p_k1dT.segment(4,3);
          
  a_func( k2_mat, x_p_k1dT, omega_m_v);

  A_mat = I_mat_77 + 1/2.0*(k1_mat + k2_mat)*INT_T*1e-3;
}



// c_k^T := dx_h
void cT_func( MatrixXf &cT_mat, const VectorXf &x_v, const VectorXf &g_v, const VectorXf &m_v){

  // 0
  MatrixXf zero_mat = MatrixXf::Zero(3,3);
  // Euler parameter [-]
  Vector4f eps_v = x_v.segment(0,4); 
  // p_vec
  Vector3f p = eps_v.segment(1,3); 
  
  // u_g = g X p
  MatrixXf u_g(3,1);
  // VectorXfのままだと外積でエラーが出るのでVector3fに変換．
  u_g = static_cast<Vector3f>(g_v).cross( p);
  // u_g + p_0*g
  VectorXf ug_p_p0g = u_g + eps_v(0)*g_v;
  // ~(u_g + p_0*g)
  MatrixXf ug_p_p0g_mat(3,3);
  ug_p_p0g_mat <<              0,  -ug_p_p0g(2),   ug_p_p0g(1),
                     ug_p_p0g(2),             0,  -ug_p_p0g(0),
                    -ug_p_p0g(1),   ug_p_p0g(0),             0;
  
  // u_m = m X p
  MatrixXf u_m(3,1);
  // VectorXfのままだと外積でエラーが出るのでVector3fに変換．
  u_m = static_cast<Vector3f>(m_v).cross( p);
  // u_m + p_0*m
  VectorXf um_p_p0m = u_m + eps_v(0)*m_v;
  // ~(u_m + p_0*m)
  MatrixXf um_p_p0m_mat(3,3);
  um_p_p0m_mat <<              0,  -um_p_p0m(2),   um_p_p0m(1),
                     um_p_p0m(2),             0,  -um_p_p0m(0),
                    -um_p_p0m(1),   um_p_p0m(0),             0;                  
                    

  cT_mat << u_g, ug_p_p0g_mat + (p.dot( g_v))*I_mat - g_v*p.transpose(), zero_mat,
            u_m, um_p_p0m_mat + (p.dot( m_v))*I_mat - m_v*p.transpose(), zero_mat;  
}
