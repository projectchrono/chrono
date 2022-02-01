IMU Sensor Model {#IMU_sensor}
=================================

\tableofcontents

Chrono::Sensor supports three sensors commonly used collectively as an IMU. These are accelerometer, gyroscope, and magnetometer.

#### Accelerometer Creation
~~~{.cpp}
// create a noise model
auto
noise_model=chrono_types::make_shared<ChNoiseNormalDrift>(
    100.f,                  // float updateRate,
    {0,0,0},                // float mean,
    {0.001,0.001,0.001},    // float stdev,
    .01f,                   // float bias_drift,
    .1f                     // float tau_drift
);

auto acc= chrono_types::make_shared<ChAccelerometerSensor>(
    parent_body,        // body to which the IMU is attached
    imu_update_rate,    // update rate
    imu_offset_pose,    // offset pose from body
    noise_model         // noise model
);

acc->SetName("Accelerometer");

acc->SetLag(.001); //1 millisecond lag
acc->SetCollectionWindow(.001);  //1 millisecond collection time

// Accelerometer data access filter
acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());

// Addsensorto manager
manager->AddSensor(acc);
~~~

#### Gyroscope Creation
~~~{.cpp}
// create a noise model
auto
noise_model=chrono_types::make_shared<ChNoiseNormalDrift>(
    100.f,                  // float updateRate,
    {0,0,0},                // float mean,
    {0.001,0.001,0.001},    // float stdev,
    .01f,                   // float bias_drift,
    .1f                     // float tau_drift
);

auto gyro= chrono_types::make_shared<ChGyroscopeSensor>(
    my_body,            // body to which the IMU is attached
    imu_update_rate,    // update rate
    imu_offset_pose,    // offset pose from body
    noise_model         // noise model
);

gyro->SetName("Gyroscope");

gyro->SetLag(.001);                 // 1 millisecond lag
gyro->SetCollectionWindow(.001);    // 1 millisecond collection time

// Gyroscope data access filter
gyro->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());

// Addsensorto manager
manager->AddSensor(gyro);
~~~

#### Magnetometer Creation
~~~{.cpp}
// create a noise model
auto
noise_model=chrono_types::make_shared<ChNoiseNormal>(
    {0,0,0},                // float mean,
    {0.001,0.001,0.001},    // float stdev
);

auto mag= chrono_types::make_shared<ChMagnetometerSensor>(
    my_body,            // body to which the IMU is attached
    100.f,              // update rate of 100 Hz
    imu_offset_pose,    // offset pose from body
    noise_model         // noise model
);

mag->SetName("Magnetometer");

mag->SetLag(.001);                  // 1 millisecond lag
mag->SetCollectionWindow(.001);     // 1 millisecond collection time

// Gyroscope data access filter
mag->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());

// Addsensorto manager
manager->AddSensor(mag);
~~~

<br>

#### IMU Data Access
~~~{.cpp}
utils::CSV_writer imu_csv(" ");
UserAccelBufferPtr bufferAcc;
UserGyroBufferPtr bufferGyro;
UserMagnetBufferPtr bufferMag;
int imu_last_launch = 0;
while () {
  bufferAcc = acc->GetMostRecentBuffer<UserAccelBufferPtr>();
  bufferGyro = gyro->GetMostRecentBuffer<UserGyroBufferPtr>();
  bufferMag = mag->GetMostRecentBuffer<UserMagnetBufferPtr>();
  if (bufferAcc->Buffer && bufferGyro->Buffer && bufferMag->Buffer &&
      bufferMag->LaunchedCount > imu_last_launch) {
      // Save the imu data to file
      AccelData acc_data = bufferAcc->Buffer[0];
      GyroData gyro_data = bufferGyro->Buffer[0];
      MagnetData mag_data = bufferMag->Buffer[0];
      imu_csv << std::fixed << std::setprecision(6);
      imu_csv << acc_data.X;
      imu_csv << acc_data.Y;
      imu_csv << acc_data.Z;
      imu_csv << gyro_data.Roll;
      imu_csv << gyro_data.Pitch;
      imu_csv << gyro_data.Yaw;
      imu_csv << mag_data.H;
      imu_csv << mag_data.X;
      imu_csv << mag_data.Y;
      imu_csv << mag_data.Z;
      imu_csv << std::endl;
      imu_last_launch = bufferMag->LaunchedCount;
  }
}
imu_csv.write_to_file(imu_file);
~~~
