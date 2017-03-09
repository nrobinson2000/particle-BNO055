/***************************************************************************
  This is a library for the BNO055 orientation sensor

  Designed specifically to work with the Adafruit BNO055 Breakout.

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by KTOWN for Adafruit Industries.

  MIT license, all text above must be included in any redistribution

  Ported to Particle devices by Nathan Robinson
  https://github.com/nrobinson2000
  On 2017-1-21

****************************************************************************/

 #include "Particle.h"
 #include <stdlib.h>
 #include <string.h>
 #include <stdint.h>
 #include <math.h>

#ifndef _ADAFRUIT_SENSOR_H
#define _ADAFRUIT_SENSOR_H

/* Intentionally modeled after sensors.h in the Android API:
 * https://github.com/android/platform_hardware_libhardware/blob/master/include/hardware/sensors.h */

/* Constants */
#define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON              (1.6F)                  /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN               (275.0F)                /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
#define SENSORS_MAGFIELD_EARTH_MAX        (60.0F)                 /**< Maximum magnetic field on Earth's surface */
#define SENSORS_MAGFIELD_EARTH_MIN        (30.0F)                 /**< Minimum magnetic field on Earth's surface */
#define SENSORS_PRESSURE_SEALEVELHPA      (1013.25F)              /**< Average sea level pressure is 1013.25 hPa */
#define SENSORS_DPS_TO_RADS               (0.017453293F)          /**< Degrees/s to rad/s multiplier */
#define SENSORS_GAUSS_TO_MICROTESLA       (100)                   /**< Gauss to micro-Tesla multiplier */

/** Sensor types */
typedef enum
{
  SENSOR_TYPE_ACCELEROMETER         = (1),   /**< Gravity + linear acceleration */
  SENSOR_TYPE_MAGNETIC_FIELD        = (2),
  SENSOR_TYPE_ORIENTATION           = (3),
  SENSOR_TYPE_GYROSCOPE             = (4),
  SENSOR_TYPE_LIGHT                 = (5),
  SENSOR_TYPE_PRESSURE              = (6),
  SENSOR_TYPE_PROXIMITY             = (8),
  SENSOR_TYPE_GRAVITY               = (9),
  SENSOR_TYPE_LINEAR_ACCELERATION   = (10),  /**< Acceleration not including gravity */
  SENSOR_TYPE_ROTATION_VECTOR       = (11),
  SENSOR_TYPE_RELATIVE_HUMIDITY     = (12),
  SENSOR_TYPE_AMBIENT_TEMPERATURE   = (13),
  SENSOR_TYPE_VOLTAGE               = (15),
  SENSOR_TYPE_CURRENT               = (16),
  SENSOR_TYPE_COLOR                 = (17)
} sensors_type_t;

/** struct sensors_vec_s is used to return a vector in a common format. */
typedef struct {
    union {
        float v[3];
        struct {
            float x;
            float y;
            float z;
        };
        /* Orientation sensors */
        struct {
            float roll;    /**< Rotation around the longitudinal axis (the plane body, 'X axis'). Roll is positive and increasing when moving downward. -90�<=roll<=90� */
            float pitch;   /**< Rotation around the lateral axis (the wing span, 'Y axis'). Pitch is positive and increasing when moving upwards. -180�<=pitch<=180�) */
            float heading; /**< Angle between the longitudinal axis (the plane body) and magnetic north, measured clockwise when viewing from the top of the device. 0-359� */
        };
    };
    int8_t status;
    uint8_t reserved[3];
} sensors_vec_t;

/** struct sensors_color_s is used to return color data in a common format. */
typedef struct {
    union {
        float c[3];
        /* RGB color space */
        struct {
            float r;       /**< Red component */
            float g;       /**< Green component */
            float b;       /**< Blue component */
        };
    };
    uint32_t rgba;         /**< 24-bit RGBA value */
} sensors_color_t;

/* Sensor event (36 bytes) */
/** struct sensor_event_s is used to provide a single sensor event in a common format. */
typedef struct
{
    int32_t version;                          /**< must be sizeof(struct sensors_event_t) */
    int32_t sensor_id;                        /**< unique sensor identifier */
    int32_t type;                             /**< sensor type */
    int32_t reserved0;                        /**< reserved */
    int32_t timestamp;                        /**< time is in milliseconds */
    union
    {
        float           data[4];
        sensors_vec_t   acceleration;         /**< acceleration values are in meter per second per second (m/s^2) */
        sensors_vec_t   magnetic;             /**< magnetic vector values are in micro-Tesla (uT) */
        sensors_vec_t   orientation;          /**< orientation values are in degrees */
        sensors_vec_t   gyro;                 /**< gyroscope values are in rad/s */
        float           temperature;          /**< temperature is in degrees centigrade (Celsius) */
        float           distance;             /**< distance in centimeters */
        float           light;                /**< light in SI lux units */
        float           pressure;             /**< pressure in hectopascal (hPa) */
        float           relative_humidity;    /**< relative humidity in percent */
        float           current;              /**< current in milliamps (mA) */
        float           voltage;              /**< voltage in volts (V) */
        sensors_color_t color;                /**< color in RGB component values */
    };
} sensors_event_t;

/* Sensor details (40 bytes) */
/** struct sensor_s is used to describe basic information about a specific sensor. */
typedef struct
{
    char     name[12];                        /**< sensor name */
    int32_t  version;                         /**< version of the hardware + driver */
    int32_t  sensor_id;                       /**< unique sensor identifier */
    int32_t  type;                            /**< this sensor's type (ex. SENSOR_TYPE_LIGHT) */
    float    max_value;                       /**< maximum value of this sensor's value in SI units */
    float    min_value;                       /**< minimum value of this sensor's value in SI units */
    float    resolution;                      /**< smallest difference between two values reported by this sensor */
    int32_t  min_delay;                       /**< min delay in microseconds between events. zero = not a constant rate */
} sensor_t;

class Adafruit_Sensor {
 public:
  // Constructor(s)
  Adafruit_Sensor() {}
  virtual ~Adafruit_Sensor() {}

  // These must be defined by the subclass
  virtual void enableAutoRange(bool enabled) {};
  virtual bool getEvent(sensors_event_t*) = 0;
  virtual void getSensor(sensor_t*) = 0;

 private:
  bool _autoRange;
};

#endif

#ifndef IMUMATH_VECTOR_HPP
#define IMUMATH_VECTOR_HPP

namespace imu
{

template <uint8_t N> class Vector
{
public:
    Vector()
    {
        memset(p_vec, 0, sizeof(double)*N);
    }

    Vector(double a)
    {
        memset(p_vec, 0, sizeof(double)*N);
        p_vec[0] = a;
    }

    Vector(double a, double b)
    {
        memset(p_vec, 0, sizeof(double)*N);
        p_vec[0] = a;
        p_vec[1] = b;
    }

    Vector(double a, double b, double c)
    {
        memset(p_vec, 0, sizeof(double)*N);
        p_vec[0] = a;
        p_vec[1] = b;
        p_vec[2] = c;
    }

    Vector(double a, double b, double c, double d)
    {
        memset(p_vec, 0, sizeof(double)*N);
        p_vec[0] = a;
        p_vec[1] = b;
        p_vec[2] = c;
        p_vec[3] = d;
    }

    Vector(const Vector<N> &v)
    {
        for (int x = 0; x < N; x++)
            p_vec[x] = v.p_vec[x];
    }

    ~Vector()
    {
    }

    uint8_t n() { return N; }

    double magnitude() const
    {
        double res = 0;
        for (int i = 0; i < N; i++)
            res += p_vec[i] * p_vec[i];

        return sqrt(res);
    }

    void normalize()
    {
        double mag = magnitude();
        if (isnan(mag) || mag == 0.0)
            return;

        for (int i = 0; i < N; i++)
            p_vec[i] /= mag;
    }

    double dot(const Vector& v) const
    {
        double ret = 0;
        for (int i = 0; i < N; i++)
            ret += p_vec[i] * v.p_vec[i];

        return ret;
    }

    // The cross product is only valid for vectors with 3 dimensions,
    // with the exception of higher dimensional stuff that is beyond
    // the intended scope of this library.
    // Only a definition for N==3 is given below this class, using
    // cross() with another value for N will result in a link error.
    Vector cross(const Vector& v) const;

    Vector scale(double scalar) const
    {
        Vector ret;
        for(int i = 0; i < N; i++)
            ret.p_vec[i] = p_vec[i] * scalar;
        return ret;
    }

    Vector invert() const
    {
        Vector ret;
        for(int i = 0; i < N; i++)
            ret.p_vec[i] = -p_vec[i];
        return ret;
    }

    Vector& operator=(const Vector& v)
    {
        for (int x = 0; x < N; x++ )
            p_vec[x] = v.p_vec[x];
        return *this;
    }

    double& operator [](int n)
    {
        return p_vec[n];
    }

    double operator [](int n) const
    {
        return p_vec[n];
    }

    double& operator ()(int n)
    {
        return p_vec[n];
    }

    double operator ()(int n) const
    {
        return p_vec[n];
    }

    Vector operator+(const Vector& v) const
    {
        Vector ret;
        for(int i = 0; i < N; i++)
            ret.p_vec[i] = p_vec[i] + v.p_vec[i];
        return ret;
    }

    Vector operator-(const Vector& v) const
    {
        Vector ret;
        for(int i = 0; i < N; i++)
            ret.p_vec[i] = p_vec[i] - v.p_vec[i];
        return ret;
    }

    Vector operator * (double scalar) const
    {
        return scale(scalar);
    }

    Vector operator / (double scalar) const
    {
        Vector ret;
        for(int i = 0; i < N; i++)
            ret.p_vec[i] = p_vec[i] / scalar;
        return ret;
    }

    void toDegrees()
    {
        for(int i = 0; i < N; i++)
            p_vec[i] *= 57.2957795131; //180/pi
    }

    void toRadians()
    {
        for(int i = 0; i < N; i++)
            p_vec[i] *= 0.01745329251;  //pi/180
    }

    double& x() { return p_vec[0]; }
    double& y() { return p_vec[1]; }
    double& z() { return p_vec[2]; }
    double x() const { return p_vec[0]; }
    double y() const { return p_vec[1]; }
    double z() const { return p_vec[2]; }


private:
    double p_vec[N];
};


template <>
inline Vector<3> Vector<3>::cross(const Vector& v) const
{
    return Vector(
        p_vec[1] * v.p_vec[2] - p_vec[2] * v.p_vec[1],
        p_vec[2] * v.p_vec[0] - p_vec[0] * v.p_vec[2],
        p_vec[0] * v.p_vec[1] - p_vec[1] * v.p_vec[0]
    );
}

} // namespace

#endif


#ifndef IMUMATH_MATRIX_HPP
#define IMUMATH_MATRIX_HPP

namespace imu
{


template <uint8_t N> class Matrix
{
public:
    Matrix()
    {
        memset(_cell_data, 0, N*N*sizeof(double));
    }

    Matrix(const Matrix &m)
    {
        for (int ij = 0; ij < N*N; ++ij)
        {
            _cell_data[ij] = m._cell_data[ij];
        }
    }

    ~Matrix()
    {
    }

    Matrix& operator=(const Matrix& m)
    {
        for (int ij = 0; ij < N*N; ++ij)
        {
            _cell_data[ij] = m._cell_data[ij];
        }
        return *this;
    }

    Vector<N> row_to_vector(int i) const
    {
        Vector<N> ret;
        for (int j = 0; j < N; j++)
        {
            ret[j] = cell(i, j);
        }
        return ret;
    }

    Vector<N> col_to_vector(int j) const
    {
        Vector<N> ret;
        for (int i = 0; i < N; i++)
        {
            ret[i] = cell(i, j);
        }
        return ret;
    }

    void vector_to_row(const Vector<N>& v, int i)
    {
        for (int j = 0; j < N; j++)
        {
            cell(i, j) = v[j];
        }
    }

    void vector_to_col(const Vector<N>& v, int j)
    {
        for (int i = 0; i < N; i++)
        {
            cell(i, j) = v[i];
        }
    }

    double operator()(int i, int j) const
    {
        return cell(i, j);
    }
    double& operator()(int i, int j)
    {
        return cell(i, j);
    }

    double cell(int i, int j) const
    {
        return _cell_data[i*N+j];
    }
    double& cell(int i, int j)
    {
        return _cell_data[i*N+j];
    }


    Matrix operator+(const Matrix& m) const
    {
        Matrix ret;
        for (int ij = 0; ij < N*N; ++ij)
        {
            ret._cell_data[ij] = _cell_data[ij] + m._cell_data[ij];
        }
        return ret;
    }

    Matrix operator-(const Matrix& m) const
    {
        Matrix ret;
        for (int ij = 0; ij < N*N; ++ij)
        {
            ret._cell_data[ij] = _cell_data[ij] - m._cell_data[ij];
        }
        return ret;
    }

    Matrix operator*(double scalar) const
    {
        Matrix ret;
        for (int ij = 0; ij < N*N; ++ij)
        {
            ret._cell_data[ij] = _cell_data[ij] * scalar;
        }
        return ret;
    }

    Matrix operator*(const Matrix& m) const
    {
        Matrix ret;
        for (int i = 0; i < N; i++)
        {
            Vector<N> row = row_to_vector(i);
            for (int j = 0; j < N; j++)
            {
                ret(i, j) = row.dot(m.col_to_vector(j));
            }
        }
        return ret;
    }

    Matrix transpose() const
    {
        Matrix ret;
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < N; j++)
            {
                ret(j, i) = cell(i, j);
            }
        }
        return ret;
    }

    Matrix<N-1> minor_matrix(int row, int col) const
    {
        Matrix<N-1> ret;
        for (int i = 0, im = 0; i < N; i++)
        {
            if (i == row)
                continue;

            for (int j = 0, jm = 0; j < N; j++)
            {
                if (j != col)
                {
                    ret(im, jm++) = cell(i, j);
                }
            }
            im++;
        }
        return ret;
    }

    double determinant() const
    {
        // specialization for N == 1 given below this class
        double det = 0.0, sign = 1.0;
        for (int i = 0; i < N; ++i, sign = -sign)
            det += sign * cell(0, i) * minor_matrix(0, i).determinant();
        return det;
    }

    Matrix invert() const
    {
        Matrix ret;
        double det = determinant();

        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < N; j++)
            {
                ret(i, j) = minor_matrix(j, i).determinant() / det;
                if ((i+j)%2 == 1)
                    ret(i, j) = -ret(i, j);
            }
        }
        return ret;
    }

    double trace() const
    {
        double tr = 0.0;
        for (int i = 0; i < N; ++i)
            tr += cell(i, i);
        return tr;
    }

private:
    double _cell_data[N*N];
};


template<>
inline double Matrix<1>::determinant() const
{
    return cell(0, 0);
}

};

#endif


#ifndef IMUMATH_QUATERNION_HPP
#define IMUMATH_QUATERNION_HPP


namespace imu
{

class Quaternion
{
public:
    Quaternion(): _w(1.0), _x(0.0), _y(0.0), _z(0.0) {}

    Quaternion(double w, double x, double y, double z):
        _w(w), _x(x), _y(y), _z(z) {}

    Quaternion(double w, Vector<3> vec):
        _w(w), _x(vec.x()), _y(vec.y()), _z(vec.z()) {}

    double& w()
    {
        return _w;
    }
    double& x()
    {
        return _x;
    }
    double& y()
    {
        return _y;
    }
    double& z()
    {
        return _z;
    }

    double w() const
    {
        return _w;
    }
    double x() const
    {
        return _x;
    }
    double y() const
    {
        return _y;
    }
    double z() const
    {
        return _z;
    }

    double magnitude() const
    {
        return sqrt(_w*_w + _x*_x + _y*_y + _z*_z);
    }

    void normalize()
    {
        double mag = magnitude();
        *this = this->scale(1/mag);
    }

    Quaternion conjugate() const
    {
        return Quaternion(_w, -_x, -_y, -_z);
    }

    void fromAxisAngle(const Vector<3>& axis, double theta)
    {
        _w = cos(theta/2);
        //only need to calculate sine of half theta once
        double sht = sin(theta/2);
        _x = axis.x() * sht;
        _y = axis.y() * sht;
        _z = axis.z() * sht;
    }

    void fromMatrix(const Matrix<3>& m)
    {
        double tr = m.trace();

        double S;
        if (tr > 0)
        {
            S = sqrt(tr+1.0) * 2;
            _w = 0.25 * S;
            _x = (m(2, 1) - m(1, 2)) / S;
            _y = (m(0, 2) - m(2, 0)) / S;
            _z = (m(1, 0) - m(0, 1)) / S;
        }
        else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2))
        {
            S = sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;
            _w = (m(2, 1) - m(1, 2)) / S;
            _x = 0.25 * S;
            _y = (m(0, 1) + m(1, 0)) / S;
            _z = (m(0, 2) + m(2, 0)) / S;
        }
        else if (m(1, 1) > m(2, 2))
        {
            S = sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;
            _w = (m(0, 2) - m(2, 0)) / S;
            _x = (m(0, 1) + m(1, 0)) / S;
            _y = 0.25 * S;
            _z = (m(1, 2) + m(2, 1)) / S;
        }
        else
        {
            S = sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;
            _w = (m(1, 0) - m(0, 1)) / S;
            _x = (m(0, 2) + m(2, 0)) / S;
            _y = (m(1, 2) + m(2, 1)) / S;
            _z = 0.25 * S;
        }
    }

    void toAxisAngle(Vector<3>& axis, double& angle) const
    {
        double sqw = sqrt(1-_w*_w);
        if (sqw == 0) //it's a singularity and divide by zero, avoid
            return;

        angle = 2 * acos(_w);
        axis.x() = _x / sqw;
        axis.y() = _y / sqw;
        axis.z() = _z / sqw;
    }

    Matrix<3> toMatrix() const
    {
        Matrix<3> ret;
        ret.cell(0, 0) = 1 - 2*_y*_y - 2*_z*_z;
        ret.cell(0, 1) = 2*_x*_y - 2*_w*_z;
        ret.cell(0, 2) = 2*_x*_z + 2*_w*_y;

        ret.cell(1, 0) = 2*_x*_y + 2*_w*_z;
        ret.cell(1, 1) = 1 - 2*_x*_x - 2*_z*_z;
        ret.cell(1, 2) = 2*_y*_z - 2*_w*_x;

        ret.cell(2, 0) = 2*_x*_z - 2*_w*_y;
        ret.cell(2, 1) = 2*_y*_z + 2*_w*_x;
        ret.cell(2, 2) = 1 - 2*_x*_x - 2*_y*_y;
        return ret;
    }


    // Returns euler angles that represent the quaternion.  Angles are
    // returned in rotation order and right-handed about the specified
    // axes:
    //
    //   v[0] is applied 1st about z (ie, roll)
    //   v[1] is applied 2nd about y (ie, pitch)
    //   v[2] is applied 3rd about x (ie, yaw)
    //
    // Note that this means result.x() is not a rotation about x;
    // similarly for result.z().
    //
    Vector<3> toEuler() const
    {
        Vector<3> ret;
        double sqw = _w*_w;
        double sqx = _x*_x;
        double sqy = _y*_y;
        double sqz = _z*_z;

        ret.x() = atan2(2.0*(_x*_y+_z*_w),(sqx-sqy-sqz+sqw));
        ret.y() = asin(-2.0*(_x*_z-_y*_w)/(sqx+sqy+sqz+sqw));
        ret.z() = atan2(2.0*(_y*_z+_x*_w),(-sqx-sqy+sqz+sqw));

        return ret;
    }

    Vector<3> toAngularVelocity(double dt) const
    {
        Vector<3> ret;
        Quaternion one(1.0, 0.0, 0.0, 0.0);
        Quaternion delta = one - *this;
        Quaternion r = (delta/dt);
        r = r * 2;
        r = r * one;

        ret.x() = r.x();
        ret.y() = r.y();
        ret.z() = r.z();
        return ret;
    }

    Vector<3> rotateVector(const Vector<2>& v) const
    {
        return rotateVector(Vector<3>(v.x(), v.y()));
    }

    Vector<3> rotateVector(const Vector<3>& v) const
    {
        Vector<3> qv(_x, _y, _z);
        Vector<3> t = qv.cross(v) * 2.0;
        return v + t*_w + qv.cross(t);
    }


    Quaternion operator*(const Quaternion& q) const
    {
        return Quaternion(
            _w*q._w - _x*q._x - _y*q._y - _z*q._z,
            _w*q._x + _x*q._w + _y*q._z - _z*q._y,
            _w*q._y - _x*q._z + _y*q._w + _z*q._x,
            _w*q._z + _x*q._y - _y*q._x + _z*q._w
        );
    }

    Quaternion operator+(const Quaternion& q) const
    {
        return Quaternion(_w + q._w, _x + q._x, _y + q._y, _z + q._z);
    }

    Quaternion operator-(const Quaternion& q) const
    {
        return Quaternion(_w - q._w, _x - q._x, _y - q._y, _z - q._z);
    }

    Quaternion operator/(double scalar) const
    {
        return Quaternion(_w / scalar, _x / scalar, _y / scalar, _z / scalar);
    }

    Quaternion operator*(double scalar) const
    {
        return scale(scalar);
    }

    Quaternion scale(double scalar) const
    {
        return Quaternion(_w * scalar, _x * scalar, _y * scalar, _z * scalar);
    }

private:
    double _w, _x, _y, _z;
};

} // namespace

#endif


#ifndef __PARTICLE_BNO055_H__
#define __PARTICLE_BNO055_H__

#define BNO055_ADDRESS_A (0x28)
#define BNO055_ADDRESS_B (0x29)
#define BNO055_ID        (0xA0)

#define NUM_BNO055_OFFSET_REGISTERS (22)

typedef struct
{
    uint16_t accel_offset_x;
    uint16_t accel_offset_y;
    uint16_t accel_offset_z;
    uint16_t gyro_offset_x;
    uint16_t gyro_offset_y;
    uint16_t gyro_offset_z;
    uint16_t mag_offset_x;
    uint16_t mag_offset_y;
    uint16_t mag_offset_z;

    uint16_t accel_radius;
    uint16_t mag_radius;
} adafruit_bno055_offsets_t;

class Adafruit_BNO055 : public Adafruit_Sensor
{
  public:
    typedef enum
    {
      /* Page id register definition */
      BNO055_PAGE_ID_ADDR                                     = 0X07,

      /* PAGE0 REGISTER DEFINITION START*/
      BNO055_CHIP_ID_ADDR                                     = 0x00,
      BNO055_ACCEL_REV_ID_ADDR                                = 0x01,
      BNO055_MAG_REV_ID_ADDR                                  = 0x02,
      BNO055_GYRO_REV_ID_ADDR                                 = 0x03,
      BNO055_SW_REV_ID_LSB_ADDR                               = 0x04,
      BNO055_SW_REV_ID_MSB_ADDR                               = 0x05,
      BNO055_BL_REV_ID_ADDR                                   = 0X06,

      /* Accel data register */
      BNO055_ACCEL_DATA_X_LSB_ADDR                            = 0X08,
      BNO055_ACCEL_DATA_X_MSB_ADDR                            = 0X09,
      BNO055_ACCEL_DATA_Y_LSB_ADDR                            = 0X0A,
      BNO055_ACCEL_DATA_Y_MSB_ADDR                            = 0X0B,
      BNO055_ACCEL_DATA_Z_LSB_ADDR                            = 0X0C,
      BNO055_ACCEL_DATA_Z_MSB_ADDR                            = 0X0D,

      /* Mag data register */
      BNO055_MAG_DATA_X_LSB_ADDR                              = 0X0E,
      BNO055_MAG_DATA_X_MSB_ADDR                              = 0X0F,
      BNO055_MAG_DATA_Y_LSB_ADDR                              = 0X10,
      BNO055_MAG_DATA_Y_MSB_ADDR                              = 0X11,
      BNO055_MAG_DATA_Z_LSB_ADDR                              = 0X12,
      BNO055_MAG_DATA_Z_MSB_ADDR                              = 0X13,

      /* Gyro data registers */
      BNO055_GYRO_DATA_X_LSB_ADDR                             = 0X14,
      BNO055_GYRO_DATA_X_MSB_ADDR                             = 0X15,
      BNO055_GYRO_DATA_Y_LSB_ADDR                             = 0X16,
      BNO055_GYRO_DATA_Y_MSB_ADDR                             = 0X17,
      BNO055_GYRO_DATA_Z_LSB_ADDR                             = 0X18,
      BNO055_GYRO_DATA_Z_MSB_ADDR                             = 0X19,

      /* Euler data registers */
      BNO055_EULER_H_LSB_ADDR                                 = 0X1A,
      BNO055_EULER_H_MSB_ADDR                                 = 0X1B,
      BNO055_EULER_R_LSB_ADDR                                 = 0X1C,
      BNO055_EULER_R_MSB_ADDR                                 = 0X1D,
      BNO055_EULER_P_LSB_ADDR                                 = 0X1E,
      BNO055_EULER_P_MSB_ADDR                                 = 0X1F,

      /* Quaternion data registers */
      BNO055_QUATERNION_DATA_W_LSB_ADDR                       = 0X20,
      BNO055_QUATERNION_DATA_W_MSB_ADDR                       = 0X21,
      BNO055_QUATERNION_DATA_X_LSB_ADDR                       = 0X22,
      BNO055_QUATERNION_DATA_X_MSB_ADDR                       = 0X23,
      BNO055_QUATERNION_DATA_Y_LSB_ADDR                       = 0X24,
      BNO055_QUATERNION_DATA_Y_MSB_ADDR                       = 0X25,
      BNO055_QUATERNION_DATA_Z_LSB_ADDR                       = 0X26,
      BNO055_QUATERNION_DATA_Z_MSB_ADDR                       = 0X27,

      /* Linear acceleration data registers */
      BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                     = 0X28,
      BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                     = 0X29,
      BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                     = 0X2A,
      BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                     = 0X2B,
      BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                     = 0X2C,
      BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                     = 0X2D,

      /* Gravity data registers */
      BNO055_GRAVITY_DATA_X_LSB_ADDR                          = 0X2E,
      BNO055_GRAVITY_DATA_X_MSB_ADDR                          = 0X2F,
      BNO055_GRAVITY_DATA_Y_LSB_ADDR                          = 0X30,
      BNO055_GRAVITY_DATA_Y_MSB_ADDR                          = 0X31,
      BNO055_GRAVITY_DATA_Z_LSB_ADDR                          = 0X32,
      BNO055_GRAVITY_DATA_Z_MSB_ADDR                          = 0X33,

      /* Temperature data register */
      BNO055_TEMP_ADDR                                        = 0X34,

      /* Status registers */
      BNO055_CALIB_STAT_ADDR                                  = 0X35,
      BNO055_SELFTEST_RESULT_ADDR                             = 0X36,
      BNO055_INTR_STAT_ADDR                                   = 0X37,

      BNO055_SYS_CLK_STAT_ADDR                                = 0X38,
      BNO055_SYS_STAT_ADDR                                    = 0X39,
      BNO055_SYS_ERR_ADDR                                     = 0X3A,

      /* Unit selection register */
      BNO055_UNIT_SEL_ADDR                                    = 0X3B,
      BNO055_DATA_SELECT_ADDR                                 = 0X3C,

      /* Mode registers */
      BNO055_OPR_MODE_ADDR                                    = 0X3D,
      BNO055_PWR_MODE_ADDR                                    = 0X3E,

      BNO055_SYS_TRIGGER_ADDR                                 = 0X3F,
      BNO055_TEMP_SOURCE_ADDR                                 = 0X40,

      /* Axis remap registers */
      BNO055_AXIS_MAP_CONFIG_ADDR                             = 0X41,
      BNO055_AXIS_MAP_SIGN_ADDR                               = 0X42,

      /* SIC registers */
      BNO055_SIC_MATRIX_0_LSB_ADDR                            = 0X43,
      BNO055_SIC_MATRIX_0_MSB_ADDR                            = 0X44,
      BNO055_SIC_MATRIX_1_LSB_ADDR                            = 0X45,
      BNO055_SIC_MATRIX_1_MSB_ADDR                            = 0X46,
      BNO055_SIC_MATRIX_2_LSB_ADDR                            = 0X47,
      BNO055_SIC_MATRIX_2_MSB_ADDR                            = 0X48,
      BNO055_SIC_MATRIX_3_LSB_ADDR                            = 0X49,
      BNO055_SIC_MATRIX_3_MSB_ADDR                            = 0X4A,
      BNO055_SIC_MATRIX_4_LSB_ADDR                            = 0X4B,
      BNO055_SIC_MATRIX_4_MSB_ADDR                            = 0X4C,
      BNO055_SIC_MATRIX_5_LSB_ADDR                            = 0X4D,
      BNO055_SIC_MATRIX_5_MSB_ADDR                            = 0X4E,
      BNO055_SIC_MATRIX_6_LSB_ADDR                            = 0X4F,
      BNO055_SIC_MATRIX_6_MSB_ADDR                            = 0X50,
      BNO055_SIC_MATRIX_7_LSB_ADDR                            = 0X51,
      BNO055_SIC_MATRIX_7_MSB_ADDR                            = 0X52,
      BNO055_SIC_MATRIX_8_LSB_ADDR                            = 0X53,
      BNO055_SIC_MATRIX_8_MSB_ADDR                            = 0X54,

      /* Accelerometer Offset registers */
      ACCEL_OFFSET_X_LSB_ADDR                                 = 0X55,
      ACCEL_OFFSET_X_MSB_ADDR                                 = 0X56,
      ACCEL_OFFSET_Y_LSB_ADDR                                 = 0X57,
      ACCEL_OFFSET_Y_MSB_ADDR                                 = 0X58,
      ACCEL_OFFSET_Z_LSB_ADDR                                 = 0X59,
      ACCEL_OFFSET_Z_MSB_ADDR                                 = 0X5A,

      /* Magnetometer Offset registers */
      MAG_OFFSET_X_LSB_ADDR                                   = 0X5B,
      MAG_OFFSET_X_MSB_ADDR                                   = 0X5C,
      MAG_OFFSET_Y_LSB_ADDR                                   = 0X5D,
      MAG_OFFSET_Y_MSB_ADDR                                   = 0X5E,
      MAG_OFFSET_Z_LSB_ADDR                                   = 0X5F,
      MAG_OFFSET_Z_MSB_ADDR                                   = 0X60,

      /* Gyroscope Offset register s*/
      GYRO_OFFSET_X_LSB_ADDR                                  = 0X61,
      GYRO_OFFSET_X_MSB_ADDR                                  = 0X62,
      GYRO_OFFSET_Y_LSB_ADDR                                  = 0X63,
      GYRO_OFFSET_Y_MSB_ADDR                                  = 0X64,
      GYRO_OFFSET_Z_LSB_ADDR                                  = 0X65,
      GYRO_OFFSET_Z_MSB_ADDR                                  = 0X66,

      /* Radius registers */
      ACCEL_RADIUS_LSB_ADDR                                   = 0X67,
      ACCEL_RADIUS_MSB_ADDR                                   = 0X68,
      MAG_RADIUS_LSB_ADDR                                     = 0X69,
      MAG_RADIUS_MSB_ADDR                                     = 0X6A
    } adafruit_bno055_reg_t;

    typedef enum
    {
      POWER_MODE_NORMAL                                       = 0X00,
      POWER_MODE_LOWPOWER                                     = 0X01,
      POWER_MODE_SUSPEND                                      = 0X02
    } adafruit_bno055_powermode_t;

    typedef enum
    {
      /* Operation mode settings*/
      OPERATION_MODE_CONFIG                                   = 0X00,
      OPERATION_MODE_ACCONLY                                  = 0X01,
      OPERATION_MODE_MAGONLY                                  = 0X02,
      OPERATION_MODE_GYRONLY                                  = 0X03,
      OPERATION_MODE_ACCMAG                                   = 0X04,
      OPERATION_MODE_ACCGYRO                                  = 0X05,
      OPERATION_MODE_MAGGYRO                                  = 0X06,
      OPERATION_MODE_AMG                                      = 0X07,
      OPERATION_MODE_IMUPLUS                                  = 0X08,
      OPERATION_MODE_COMPASS                                  = 0X09,
      OPERATION_MODE_M4G                                      = 0X0A,
      OPERATION_MODE_NDOF_FMC_OFF                             = 0X0B,
      OPERATION_MODE_NDOF                                     = 0X0C
    } adafruit_bno055_opmode_t;

    typedef enum
    {
      REMAP_CONFIG_P0                                         = 0x21,
      REMAP_CONFIG_P1                                         = 0x24, // default
      REMAP_CONFIG_P2                                         = 0x24,
      REMAP_CONFIG_P3                                         = 0x21,
      REMAP_CONFIG_P4                                         = 0x24,
      REMAP_CONFIG_P5                                         = 0x21,
      REMAP_CONFIG_P6                                         = 0x21,
      REMAP_CONFIG_P7                                         = 0x24
    } adafruit_bno055_axis_remap_config_t;

    typedef enum
    {
      REMAP_SIGN_P0                                           = 0x04,
      REMAP_SIGN_P1                                           = 0x00, // default
      REMAP_SIGN_P2                                           = 0x06,
      REMAP_SIGN_P3                                           = 0x02,
      REMAP_SIGN_P4                                           = 0x03,
      REMAP_SIGN_P5                                           = 0x01,
      REMAP_SIGN_P6                                           = 0x07,
      REMAP_SIGN_P7                                           = 0x05
    } adafruit_bno055_axis_remap_sign_t;

    typedef struct
    {
      uint8_t  accel_rev;
      uint8_t  mag_rev;
      uint8_t  gyro_rev;
      uint16_t sw_rev;
      uint8_t  bl_rev;
    } adafruit_bno055_rev_info_t;

    typedef enum
    {
      VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
      VECTOR_MAGNETOMETER  = BNO055_MAG_DATA_X_LSB_ADDR,
      VECTOR_GYROSCOPE     = BNO055_GYRO_DATA_X_LSB_ADDR,
      VECTOR_EULER         = BNO055_EULER_H_LSB_ADDR,
      VECTOR_LINEARACCEL   = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
      VECTOR_GRAVITY       = BNO055_GRAVITY_DATA_X_LSB_ADDR
    } adafruit_vector_type_t;

    Adafruit_BNO055 ( int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A );

    bool  begin               ( adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF );
    void  setMode             ( adafruit_bno055_opmode_t mode );
    void  getRevInfo          ( adafruit_bno055_rev_info_t* );
    void  displayRevInfo      ( void );
    void  setExtCrystalUse    ( boolean usextal );
    void  getSystemStatus     ( uint8_t *system_status,
                                uint8_t *self_test_result,
                                uint8_t *system_error);
    void  displaySystemStatus ( void );
    void  getCalibration      ( uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag);

    imu::Vector<3>  getVector ( adafruit_vector_type_t vector_type );
    imu::Quaternion getQuat   ( void );
    int8_t          getTemp   ( void );

    /* Adafruit_Sensor implementation */
    bool  getEvent  ( sensors_event_t* );
    void  getSensor ( sensor_t* );

    /* Functions to deal with raw calibration data */
    bool  getSensorOffsets(uint8_t* calibData);
    bool  getSensorOffsets(adafruit_bno055_offsets_t &offsets_type);
    void  setSensorOffsets(const uint8_t* calibData);
    void  setSensorOffsets(const adafruit_bno055_offsets_t &offsets_type);
    bool  isFullyCalibrated(void);

  private:
    byte  read8   ( adafruit_bno055_reg_t );
    bool  readLen ( adafruit_bno055_reg_t, byte* buffer, uint8_t len );
    bool  write8  ( adafruit_bno055_reg_t, byte value );

    uint8_t _address;
    int32_t _sensorID;
    adafruit_bno055_opmode_t _mode;
};

#endif
