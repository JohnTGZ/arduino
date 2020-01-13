#include <SparkFunLSM6DS3.h>
#include <Wire.h>
#include <math.h>

float Acc_roll, Acc_pitch, Acc_yaw;
float Gyro_roll_rate, Gyro_pitch_rate, Gyro_yaw_rate, Gyro_roll, Gyro_pitch, Gyro_yaw;
float roll_temp, pitch_temp, yaw_temp, roll, pitch, yaw;
float dt = 0.002403; //sampling time at 416Hz for the gyro
float W_acc = 0.05 , W_gyro = 0.95, W_prime; //Weights for acc and gyro
float z = 1;

int i, itr = 1000;
float Acc_x_error , Acc_y_error , Acc_z_error , Gyro_x_error , Gyro_y_error , Gyro_z_error ;
float Acc_x_error_f = 0, Acc_y_error_f = 0 , Acc_z_error_f = 0 , Gyro_x_error_f = 0 , Gyro_y_error_f = 0 , Gyro_z_error_f = 0 ;

LSM6DS3 myIMU( I2C_MODE, 0x6A );  //I2C device address 0x6A

void setup() {
  Serial.begin(9600);
  //Call .begin() to configure the IMUs
  if ( myIMU.begin() != 0 )
  {
    Serial.println("Device error");
  }
  else
  {
    Serial.println("Device OK!");
  }
  W_prime = z/(z-0.98);
  calc_error();
}


void loop()
{
    roll = int(Est_roll());
    pitch = int(Est_pitch());
    yaw = int(Est_yaw());
    
    Serial.print("Est_roll: ");
    Serial.println(Est_roll);
    Serial.print("Est_Pitch: ");
    Serial.println(pitch);
    Serial.print("Est_Yaw: ");
    Serial.println(Est_yaw);
}




//Obtain roll,pitch and yaw euler angles from accelerometer
float Acc_x() {
  Acc_roll = (atan2( myIMU.readFloatAccelY() , sqrt( pow(myIMU.readFloatAccelX(), 2) + pow(myIMU.readFloatAccelZ(), 2) ) )) * 180 / PI - Acc_x_error_f;
  return Acc_roll;
}
float Acc_y() {
  Acc_pitch = ( atan2( -1 * myIMU.readFloatAccelX() , sqrt( pow(myIMU.readFloatAccelY(), 2) + pow(myIMU.readFloatAccelZ(), 2) ) )) * 180 / PI - Acc_y_error_f ;
  return Acc_pitch;
}
float Acc_z() {
  Acc_yaw = myIMU.readFloatAccelZ() - Acc_z_error_f;
  return Acc_yaw;
}


//Obtain Angular Rate
float Gyro_x_rate() { 
  Gyro_roll_rate = myIMU.readFloatGyroX() - Gyro_x_error_f;
  return Gyro_roll_rate;
}
float Gyro_y_rate() {
  Gyro_pitch_rate = myIMU.readFloatGyroY() - Gyro_y_error_f;
  return Gyro_pitch_rate;
}
float Gyro_z_rate() {
  Gyro_yaw_rate = myIMU.readFloatGyroZ() - Gyro_z_error_f;
  return Gyro_yaw_rate;
}

//Obtain roll,pitch and yaw euler angles from gyro

float Gyro_x_angle() {
  Gyro_roll = W_gyro * Gyro_x_rate() * dt;
  return Gyro_roll; 
}
float Gyro_y_angle() {
  Gyro_pitch = W_gyro * Gyro_y_rate() * dt;
  return Gyro_pitch; 
}
float Gyro_z_angle() {
  Gyro_yaw = W_gyro * Gyro_z_rate() * dt;
  return Gyro_yaw; 
}



float Est_roll() {
  roll_temp = (0.02 * Acc_x() + Gyro_x_angle()) * W_prime;
  if (roll_temp > 90) {
    return 90;
  }
  else if (roll_temp < -90) {
    return -90;
  }
  else {
    return roll_temp; 
  }
}
float Est_pitch() {
  pitch_temp  = (0.02 * Acc_y() + Gyro_y_angle()) * W_prime;
  if (pitch_temp >90 ) {
    return 90;
  }
  else if (pitch_temp < -90) {
    return -90;
  }
  else{
    return pitch_temp; 
  }
}
float Est_yaw() {
  yaw_temp = (0.02 * Acc_z() + Gyro_z_angle()) * W_prime;
  if (yaw_temp > 90) {
    return 90;
  }
  else if (yaw_temp < -90) {
    return -90;
  }
  else{
    return yaw_temp; 
  }
}


void calc_error() {
  Serial.print("Start error calculation...");
  for (i = 0; i < itr; i++) {
    Acc_x_error += Acc_x();
    Acc_y_error += Acc_y();
    Acc_z_error += Acc_z();
  }
  Acc_x_error /= itr;
  Acc_y_error /= itr;
  Acc_z_error /= itr;

  for (i = 0; i < itr; i++) {
    Gyro_x_error += Gyro_x_rate();
    Gyro_y_error += Gyro_x_rate();
    Gyro_z_error += Gyro_x_rate();
  }
  Gyro_x_error /= itr;
  Gyro_y_error /= itr;
  Gyro_z_error /= itr;

  Acc_x_error_f = Acc_x_error;
  Acc_y_error_f = Acc_y_error;
  Acc_z_error_f = Acc_z_error;

  Gyro_x_error_f = Gyro_x_error;
  Gyro_y_error_f = Gyro_y_error;
  Gyro_z_error_f = Gyro_z_error;

}
