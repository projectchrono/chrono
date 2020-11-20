IMU Sensor Model {#IMU_sensor}
=================================

\tableofcontents

Details of the IMU sensor implemented in Chrono::Sensor.


#### IMU Creation
~~~{.cpp}
// create a noise model
auto
imu_noise_model=chrono_types::make_shared<ChIMUNoiseNormalDrift>(
    100.f,   // float updateRate,
    0.f,     // float g_mean,
    .001f,   // float g_stdev,
    .0f,     // float g_bias_drift,
    .1f,     // float g_tau_drift,
    .0f,     // float a_mean,
    .0075f,  // float a_stdev,
    .001f,   // float a_bias_drift,
    .1f      // floata_tau_drift);
);

auto imu= chrono_types::make_shared<ChIMUSensor>(
    my_body,    // body to which the IMU is attached
    imu_update_rate,   // update rate
    imu_offset_pose,   // offset pose from bodyimu_noise_model
);  
// IMU noise model
imu->SetName("IMU");

imu->SetLag(imu_lag);
imu->SetCollectionWindow(imu_collection_time);

// IMU data accessfilter
imu->PushFilter(chrono_types::make_shared<ChFilterIMUAccess>());

// Addsensorto manager
manager->AddSensor(imu);
~~~

<br>

#### IMU Data Access
~~~{.cpp}
utils::CSV_writer imu_csv(" ");
UserIMUBufferPtr bufferIMU;
while(){
    bufferIMU=imu->GetMostRecentBuffer<UserIMUBufferPtr>();       
    if(bufferIMU->Buffer) {
        // Save the imudata to file
        IMUDataimu_data = bufferIMU->Buffer[0];
        imu_csv<<std::fixed <<std::setprecision(6);
        imu_csv<<imu_data.Accel[0];  // Acc X
        imu_csv<<imu_data.Accel[1];  // Acc Y
        imu_csv<<imu_data.Accel[2];  // Acc Z
        imu_csv<<imu_data.Roll;      // Roll
        imu_csv<<imu_data.Pitch;     // Pitch
        imu_csv<<imu_data.Yaw;       // Yaw
        imu_csv<<std::endl;
    }
}
imu_csv.write_to_file(imu_file);
~~~