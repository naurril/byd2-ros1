
#include "inu_codec.h"

int decode_latlon(const uint8_t *buffer, size_t buffer_size, LatLon &pos) {

    inu_latitude_longitude_t latlon;
    int ret = inu_latitude_longitude_unpack(
        &latlon,
        buffer, 
        buffer_size        
    );

    pos.lat = inu_latitude_longitude_pos_lat_decode(latlon.pos_lat);
    pos.lon = inu_latitude_longitude_pos_lon_decode(latlon.pos_lon);
    
    return ret;
}

int decode_alt(const uint8_t *buffer, size_t buffer_size, double &alt) {
    
    inu_altitude_t obj;
    int ret = inu_altitude_unpack(
        &obj,
        buffer, 
        buffer_size        
    );

    alt = inu_altitude_pos_alt_decode(obj.pos_alt);
    return ret;
}


int decode_accel(const uint8_t *buffer, size_t buffer_size, vector3d &obj) {
    
    inu_accel_imu_raw_t accel;
    int ret = inu_accel_imu_raw_unpack(
        &accel,
        buffer, 
        buffer_size        
    );

    obj.x = inu_accel_imu_raw_accel_raw_x_decode(accel.accel_raw_x);
    obj.y = inu_accel_imu_raw_accel_raw_y_decode(accel.accel_raw_y);
    obj.z = inu_accel_imu_raw_accel_raw_z_decode(accel.accel_raw_z);

    return ret;
}

int decode_angle_rate(const uint8_t *buffer, size_t buffer_size, vector3d &obj) {
    
    inu_ang_rate_raw_imu_t angle_rate;
    int ret = inu_ang_rate_raw_imu_unpack(
        &angle_rate,
        buffer, 
        buffer_size        
    );

    obj.x = inu_ang_rate_raw_imu_ang_rate_raw_x_decode(angle_rate.ang_rate_raw_x);
    obj.y = inu_ang_rate_raw_imu_ang_rate_raw_y_decode(angle_rate.ang_rate_raw_y);
    obj.z = inu_ang_rate_raw_imu_ang_rate_raw_z_decode(angle_rate.ang_rate_raw_z);

    return ret;
}

int decode_angle(const uint8_t *buffer, size_t buffer_size, vector3d &obj) {
    
    inu_heading_pitch_roll_t angle;
    int ret = inu_heading_pitch_roll_unpack(
        &angle,
        buffer, 
        buffer_size        
    );

    obj.z = inu_heading_pitch_roll_angle_heading_decode(angle.angle_heading);
    obj.y = inu_heading_pitch_roll_angle_pitch_decode(angle.angle_pitch);
    obj.x = inu_heading_pitch_roll_angle_roll_decode(angle.angle_roll);

    return ret;
}