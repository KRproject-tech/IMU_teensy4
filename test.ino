/* LED Blink, Teensyduino Tutorial #1
   http://www.pjrc.com/teensy/tutorial.html
 
   This example code is in the public domain.
*/

#include <FlexiTimer2.h>
#include <Wire.h>
#include <ArduinoEigen.h>


using namespace Eigen;


// EKF Variables
MatrixXf P_mat = MatrixXf::Zero(7,7);  // Error covariance matrix P_k
VectorXf x_vec = VectorXf::Zero(7);     // x := [ eps_vec, omega_bias_vec]




#define INT_T 2                // 割り込み時間 [ms]
#define G_ACC 9.80665           // 重力加速度 [m/s^2]


// BMX055　加速度センサのI2Cアドレス  
#define ADDR_ACC  0x19          // (JP1,JP2,JP3 = Openの時)
#define ADDR_GYR  0x69          // (JP1,JP2,JP3 = Openの時)
#define ADDR_MAG  0x13          // (JP1,JP2,JP3 = Openの時)
// 磁気センサ平均値 [a.u.]
#define MAG_X_C (-23.5083)
#define MAG_Y_C (-42.2819)
#define MAG_Z_C (-29.2886)



// Teensy 2.0 has the LED on pin 11
// Teensy++ 2.0 has the LED on pin 6
// Teensy 3.x / Teensy LC have the LED on pin 13
const int ledPin = 13;

int global_time_ms = 0;         // 時刻 [ms]





// the setup() method runs once, when the sketch starts
void setup() {  
  
  // initialize the digital pin as an output.
  pinMode( ledPin, OUTPUT);

  // USB serial
  Serial.begin( 921600); 

  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  Wire.setClock(2000000);     // タイマー割り込み関数内では，BMX055との通信が時間が掛かる．

  //BMX055 初期化
  init_BMX055(); 

  // initialization
  init_EKF();  

  // timer 割り込み
  FlexiTimer2::set( INT_T, flip); 
  FlexiTimer2::start();
}




//---------------------- timer 割り込み関数 ----------------------
void flip(){


  // 加速度 [m/s^2]
  float A_x = 0.0;
  float A_y = 0.0;
  float A_z = 0.0;
  // 角速度 [rad/s]
  float Omega_x = 0.0;
  float Omega_y = 0.0;
  float Omega_z = 0.0;
  // 地磁気 [a.u.]
  float Mag_x = 0.0;
  float Mag_y = 0.0;
  float Mag_z = 0.0;
  

  // 角速度 [rad/s]
  VectorXf omega_m_vec = VectorXf::Zero(3);
  // 加速度 [m/s^2]
  VectorXf acc_m_vec = VectorXf::Zero(3);
  // 地磁気(正規化後) [a.u.]
  VectorXf mag_m_vec = VectorXf::Zero(3); 

  // Euler parameter [-]
  VectorXf eps_vec = VectorXf::Zero(4);   
  // h
  VectorXf h_vec = VectorXf::Zero(6);
  

  
  int time_us_calc = 0; // [us]
  
  
  //------------- 時刻 [ms] -------------------------
  global_time_ms += INT_T;
  global_time_ms = (global_time_ms < 1000*1000) ? global_time_ms : 0; 


  // BMX055 
  // 加速度 [m/s^2]
  read_Acc( &A_x, &A_y, &A_z);
  // 角速度 [rad/s]
  read_Gyro( &Omega_x, &Omega_y, &Omega_z);
  // 地磁気 [a.u.]
  read_Mag( &Mag_x, &Mag_y, &Mag_z);
  


  //------------- EKF ------------------------------
  time_us_calc = micros();

  
  // Measured angular velocity: omega_m_vec [rad/s]
  omega_m_vec <<  Omega_x, 
                  Omega_y, 
                  Omega_z;
  // Measured acceralation: acc_m_vec [m/s^2]
  acc_m_vec <<  A_x, 
                A_y, 
                A_z;
  // Measured magnetic field: mag_m_vec [a.u.]
  mag_m_vec <<  -(Mag_y - MAG_Y_C),
                 (Mag_x - MAG_X_C),
                 (Mag_z - MAG_Z_C);
  mag_m_vec.normalize();                


  // Extended Kalman filter
  EKF( x_vec, h_vec, P_mat, omega_m_vec, acc_m_vec, mag_m_vec);      

  // eps
  eps_vec = x_vec.segment(0,4);

  
  
  time_us_calc = micros() - time_us_calc;

  


  //---------- send to PC -----------------------------  
//  Serial.printf("AA,%f[s],%f[-],%f[-],%f[-],%f[-],%f[-],%f[-]\r\n", (float)global_time_ms/1000.0, (float)A_x, (float)A_y, (float)A_z, 
//                                                                                                  (float)Mag_x, (float)Mag_y, (float)Mag_z);  
//  Serial.printf("AA,%f[s],%f[-],%f[-],%f[-],%f[-],%f[-],%f[-]\r\n", (float)global_time_ms/1000.0, (float)A_x, (float)A_y, (float)A_z, 
//                                                                                                  (float)Omega_x, (float)Omega_y, (float)Omega_z);  
//  Serial.printf("AA,%f[s],%f[-],%f[-],%f[-],%f[-],%f[-],%f[-]\r\n", (float)global_time_ms/1000.0, (float)A_x, (float)A_z, (float)h_vec(0), 
//                                                                                                    (float)h_vec(2), (float)eps_vec.norm(), (float)time_us_calc);   
//  Serial.printf("AA,%f[s],%f[-],%f[-],%f[-],%f[-],%f[-],%f[-]\r\n", (float)global_time_ms/1000.0, (float)x_vec(0), (float)x_vec(1), (float)x_vec(2), 
//                                                                                                  (float)x_vec(3), (float)h_vec(2), (float)time_us_calc);       
  // For Matlab plot
  Serial.printf("%f %f %f %f %f %f %f %f\n", (float)global_time_ms/1000.0, (float)x_vec(0), (float)x_vec(1), (float)x_vec(2), (float)x_vec(3),
                                                                           (float)mag_m_vec(0), (float)mag_m_vec(1), (float)mag_m_vec(2)); 

}



// the loop() methor runs over and over again,
// as long as the board has power

void loop() {
  digitalWrite(ledPin, HIGH);   // set the LED on
  delay(100);                  // wait for a second
  digitalWrite(ledPin, LOW);    // set the LED off
  delay(100);                  // wait for a second
}
