

//---------------------- BMX055 初期化 --------------------------
void init_BMX055(){
  
  //-------------------加速度センサ----------------------------------//
  // Range
  Wire.beginTransmission( ADDR_ACC);
  Wire.write(0x0F);  // PMU_Rangeレジスタを選択
  Wire.write(0x03);  // Range = +/- 2g
  Wire.endTransmission();
  delay( 100);
  // Sampling frequency
  Wire.beginTransmission( ADDR_ACC);
  Wire.write(0x10);  // PMU_BWレジスタを選択
  Wire.write(0x0E);  // Bandwidth = 500 Hz
  Wire.endTransmission();
  delay( 100);
  // Sleep setting
  Wire.beginTransmission( ADDR_ACC);
  Wire.write(0x11);  // PMU_LPWレジスタを選択
  Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
  Wire.endTransmission();
  delay( 100);

  //--------------------ジャイロセンサ -----------------------------//
  // Range
  Wire.beginTransmission( ADDR_GYR);
  Wire.write(0x0F);  // Select Range register
  Wire.write(0x01);  // Full scale = +/- 1000 degree/s
//  Wire.write(0x02);  // Full scale = +/- 500 degree/s
//  Wire.write(0x03);  // Full scale = +/- 250 degree/s
  Wire.endTransmission();
  delay( 100);
  // Sampling frequency
  Wire.beginTransmission( ADDR_GYR);
  Wire.write(0x10);  // Select Bandwidth register
//  Wire.write(0x07);  // ODR = 100 Hz
  Wire.write(0x02);  // ODR = 1000 Hz
  Wire.endTransmission();
  delay( 100);
  // Sleep setting
  Wire.beginTransmission( ADDR_GYR);
  Wire.write(0x11);  // Select LPM1 register
  Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay( 100);
  
  //--------------------磁気センサ --------------------------------//
  // (通常設定値)
  Wire.beginTransmission( ADDR_MAG);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x83);  // Soft reset
  Wire.endTransmission();
  delay( 100);
  // 
  Wire.beginTransmission( ADDR_MAG);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x01);  // Soft reset
  Wire.endTransmission();
  delay( 100);
  // Output Data Rate [Hz]
  Wire.beginTransmission( ADDR_MAG);
  Wire.write(0x4C);  // Select Mag register
  Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
//  Wire.write(0x07);  // Normal Mode, ODR = 30 Hz
  Wire.endTransmission();
  delay( 100);
  // 
  Wire.beginTransmission( ADDR_MAG);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-Axis enabled
  Wire.endTransmission();
  delay( 100);
  // 
  Wire.beginTransmission( ADDR_MAG);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
  delay( 100);
  // 
  Wire.beginTransmission( ADDR_MAG);
  Wire.write(0x52);  // Select Mag register
  Wire.write(0x16);  // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}


// BMX055 
//-------------------加速度センサ----------------------------------//
void read_Acc( float* A_x, float* A_y, float* A_z){

  int i = 0;
  int data[6] = { 0 };

  for(i=0; i<6; i++){
    
    Wire.beginTransmission( ADDR_ACC);
    Wire.write((2 + i));// データレジスタを選択
    Wire.endTransmission();
    Wire.requestFrom( ADDR_ACC, 1);// 1バイトのデータを要求する

    // 6バイトのデータを読み取る
    // A_x lsb, A_x msb, A_y lsb, A_y msb, A_z lsb, A_z msb
    if(Wire.available() == 1)
      data[i] = Wire.read();
  }

  // データを12ビットに変換
  *A_x = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
  if (*A_x > 2047)  *A_x -= 4096;
  *A_y = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
  if (*A_y > 2047)  *A_y -= 4096;
  *A_z = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
  if (*A_z > 2047)  *A_z -= 4096;
  
  *A_x = *A_x*G_ACC*-0.00098; // [m/s^2] @ range +-2g
  *A_y = *A_y*G_ACC*-0.00098; // [m/s^2] @ range +-2g
  *A_z = *A_z*G_ACC*-0.00098; // [m/s^2] @ range +-2g

}

// BMX055
//-------------------ジャイロセンサ----------------------------------//


void read_Gyro( float* Omega_x, float* Omega_y, float* Omega_z){

  int i = 0;
  int data[6] = { 0 };

  for (i=0; i<6; i++)
  {
    Wire.beginTransmission( ADDR_GYR);
    Wire.write((2 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom( ADDR_GYR, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // Omega_x lsb, Omega_x msb, Omega_y lsb, Omega_y msb, Omega_z lsb, Omega_z msb
    if(Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  *Omega_x = (data[1] * 256) + data[0];
  if (*Omega_x > 32767)  *Omega_x -= 65536;
  *Omega_y = (data[3] * 256) + data[2];
  if (*Omega_y > 32767)  *Omega_y -= 65536;
  *Omega_z = (data[5] * 256) + data[4];
  if (*Omega_z > 32767)  *Omega_z -= 65536;

  *Omega_x = *Omega_x*PI/180.0*0.0305; //  [rad/s] @ Full scale = +/- 1000 degree/s
  *Omega_y = *Omega_y*PI/180.0*0.0305; //  [rad/s] @ Full scale = +/- 1000 degree/s
  *Omega_z = *Omega_z*PI/180.0*0.0305; //  [rad/s] @ Full scale = +/- 1000 degree/s

//  *Omega_x = *Omega_x*PI/180.0*0.0153; //  [rad/s] @ Full scale = +/- 500 degree/s
//  *Omega_y = *Omega_y*PI/180.0*0.0153; //  [rad/s] @ Full scale = +/- 500 degree/s
//  *Omega_z = *Omega_z*PI/180.0*0.0153; //  [rad/s] @ Full scale = +/- 500 degree/s

//  *Omega_x = *Omega_x*PI/180.0*0.0076; //  [rad/s] @ Full scale = +/- 250 degree/s
//  *Omega_y = *Omega_y*PI/180.0*0.0076; //  [rad/s] @ Full scale = +/- 250 degree/s
//  *Omega_z = *Omega_z*PI/180.0*0.0076; //  [rad/s] @ Full scale = +/- 250 degree/s

  // 補正係数
//  *Omega_x = *Omega_x*1.5;
//  *Omega_y = *Omega_y*1.5;
//  *Omega_z = *Omega_z*1.5;

}

// BMX055
//-------------------磁気センサ----------------------------------//
void read_Mag( float* Mag_x, float* Mag_y, float* Mag_z){

  int i = 0;
  int data[6] = { 0 };

  for (i=0; i<6; i++)
  {
    Wire.beginTransmission( ADDR_MAG);
    Wire.write((0x42 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom( ADDR_MAG, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // Mag_x lsb, Mag_x msb, Mag_y lsb, Mag_y msb, Mag_z lsb, Mag_z msb
    if(Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  *Mag_x = ((data[1] << 8) | data[0]) >> 3;
  if(*Mag_x > 4095) *Mag_x -= 8192;
  *Mag_y = ((data[3] << 8) | data[2]) >> 3;
  if(*Mag_y > 4095) *Mag_y -= 8192;
  *Mag_z = ((data[5] << 8) | data[4]) >> 1;
  if(*Mag_z > 16383) *Mag_z -= 32768;
}
