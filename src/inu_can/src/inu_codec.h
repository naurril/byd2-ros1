#pragma once

#include "inu.h"

struct LatLon{    
    double lat;
    double lon;
};

int decode_latlon(const uint8_t *buffer, size_t buffer_size, LatLon &obj);

int decode_alt(const uint8_t *buffer, size_t buffer_size, double &alt);


struct vector3d{
    double x;
    double y;
    double z;
};


int decode_accel(const uint8_t *buffer, size_t buffer_size, vector3d &obj);
int decode_angle_rate(const uint8_t *buffer, size_t buffer_size, vector3d &obj);
int decode_angle(const uint8_t *buffer, size_t buffer_size, vector3d &obj);