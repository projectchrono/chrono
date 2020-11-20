GPS Sensor Model {#GPS_sensor}
=================================

\tableofcontents

Details of the GPS sensor implemented in Chrono::Sensor.

#### GPS Creation
~~~{.cpp}
// create a noise model
auto gps_noise_model=chrono_types::make_shared<ChGPSNoiseNormal>(
    ChVector<float>(1.f, 1.f, 1.f),  // Mean
    ChVector<float>(2.f, 3.f, 1.f)   // Standard Deviation
);
auto gps= chrono_types::make_shared<ChGPSSensor>(
    my_body,   // body to which the GPS is attached
    gps_update_rate,  // update rate
    gps_offset_pose,  // offset pose from body
    gps_reference,    // reference GPS location (GPS coordinates of simulation origin)
    gps_noise_model// noise model to use for adding GPS noise
);

gps->SetName("GPS");
gps->SetLag(gps_lag);
gps->SetCollectionWindow(gps_collection_time);

// GPS data accessfilter
gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());

// Addsensorto manager
manager->AddSensor(gps);
~~~

<br>

#### GPS data access
~~~{.cpp}
utils::CSV_writergps_csv(" ");
UserGPSBufferPtr bufferGPS;
while(){
    bufferGPS=gps->GetMostRecentBuffer<UserGPSBufferPtr>();
    if(bufferGPS->Buffer) {
        // Save the gpsdata to file
        GPSDatagps_data= bufferGPS->Buffer[0];
        gps_csv<<std::fixed <<std::setprecision(6);
        gps_csv<<gps_data.Latitude;   // Latitude
        gps_csv<<gps_data.Longitude;  // Longitude
        gps_csv<<gps_data.Altitude;   // Altitude
        gps_csv<<gps_data.Time;       // Time
        gps_csv<<std::endl;
    }
}

gps_csv.write_to_file(gps_file);
~~~