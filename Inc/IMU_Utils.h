#ifndef __IMU_UTILS_H
#define __IMU_UTILS_H

/* Constants */
#define SENSORS_GRAVITY_EARTH             (9.80665F)              /* Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON              (1.6F)                  /* The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN               (275.0F)                /* The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
#define SENSORS_MAGFIELD_EARTH_MAX        (60.0F)                 /* Maximum magnetic field on Earth's surface */
#define SENSORS_MAGFIELD_EARTH_MIN        (30.0F)                 /* Minimum magnetic field on Earth's surface */
#define SENSORS_PRESSURE_SEALEVELHPA      (1013.25F)              /* Average sea level pressure is 1013.25 hPa */
#define SENSORS_DPS_TO_RADS               (0.017453293F)          /* Degrees/s to rad/s multiplier */
#define SENSORS_GAUSS_TO_MICROTESLA       (100)                   /* Gauss to micro-Tesla multiplier */
#define CONVERT_TO_SI                     (0.001F)                /* Coverte to the international System of Units */
#define PI                                (3.14159265359)
#endif // __IMU_UTILS_H