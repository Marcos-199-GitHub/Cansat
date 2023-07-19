"""
    void no_filter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q) {
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];  // variable for readability
        q[0] += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * deltaT;
        q[1] += 0.5f * (q0 * gx + q2 * gz - q3 * gy) * deltaT;
        q[2] += 0.5f * (q0 * gy - q1 * gz + q3 * gx) * deltaT;
        q[3] += 0.5f * (q0 * gz + q1 * gy - q2 * gx) * deltaT;
        float recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] *= recipNorm;
        q[1] *= recipNorm;
        q[2] *= recipNorm;
        q[3] *= recipNorm;
    }


    }
"""
import numpy as np
import math
import time 

    # // Mahony accelleration filter
    # // Mahony scheme uses proportional and integral filtering on
    # // the error between estimated reference vector (gravity) and measured one.
    # // Madgwick's implementation of Mayhony's AHRS algorithm.
    # // See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
    # // Free parameters in the Mahony filter and fusion scheme,
    # // Kp for proportional feedback, Ki for integral
    # // float Kp = 30.0;
    # // float Ki = 0.0;
    # // with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
    # // with MPU-6050, some instability observed at Kp=100 Now set to 30.

class QuaternionFilter:
    def __init__(self) -> None:
        # for mahony
        self.Kp = 30.0;
        self.Ki = 0.0;
        self.deltaT = 0.0
        self.newTime = 0.0 
        self.oldTime = 0.0
        self.q = np.zeros(4)
            # // for madgwick
        self.GyroMeasError = math.pi * (40.0 / 180.0)     #// gyroscope measurement error in rads/s (start at 40 deg/s)
        self.GyroMeasDrift = math.pi * (0.0 / 180.0)   #// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
        self.beta = math.sqrt(3.0 / 4.0) * self.GyroMeasError  #// compute beta
        self.zeta = math.sqrt(3.0 / 4.0) * self.GyroMeasDrift #// compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
        self.filter_sel = "MADGWICK"

    def update(self,ax: float,ay: float,az: float,gx: float,gy: float,gz: float,mx: float,my: float,mz: float, q: np.ndarray)->np.ndarray:
        self.newTime = time.time()
        self.deltaT = self.newTime - self.oldTime
        self.oldTime = self.newTime
        self.deltaT = abs(self.deltaT * 0.001 * 0.001)

        if (self.filter_sel == "MADGWICK"):
            return self.madgwick(ax, ay, az, gx, gy, gz, mx, my, mz, q)
        elif (self.filter_sel == "MAHONY"):
            return self.mahony(ax, ay, az, gx, gy, gz, mx, my, mz, q)
        else:
            return self.no_filter(ax, ay, az, gx, gy, gz, mx, my, mz, q)


    # Madgwick Quaternion Update
    def madgwick(self,ax:float, ay:float, az:float, gx:float, gy:float, gz:float, mx:float, my:float, mz:float, q:np.ndarray) -> np.ndarray:
        self.q = q
        #   // short name local variable for readability
        q0 = q[0]
        q1 = q[1]
        q2 = q[2]
        q3 = q[3]
        recipNorm = 0.0
        s0 = 0.0
        s1 = 0.0
        s2 = 0.0
        s3 = 0.0

        qDot1 = 0.0
        qDot2 = 0.0
        qDot3 = 0.0
        qDot4 = 0.0
        hx = 0.0
        hy = 0.0

        _2q0mx = 0.0
        _2q0my = 0.0
        _2q0mz = 0.0
        _2q1mx = 0.0
        _2bx = 0.0
        _2bz = 0.0
        _4bx = 0.0
        _4bz = 0.0
        _2q0 = 0.0
        _2q1 = 0.0
        _2q2 = 0.0
        _2q3 = 0.0
        _2q0q2 = 0.0
        _2q2q3 = 0.0
        q0q0 = 0.0
        q0q1 = 0.0
        q0q2 = 0.0
        q0q3 = 0.0
        q1q1 = 0.0
        q1q2 = 0.0
        q1q3 = 0.0
        q2q2 = 0.0
        q2q3 = 0.0
        q3q3 = 0.0

        # // Rate of change of quaternion from gyroscope
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        # // Normalise accelerometer measurement
        a_norm = ax * ax + ay * ay + az * az
        if (a_norm == 0.): return q #// handle NaN
        recipNorm = 1.0 / math.sqrt(a_norm)
        ax *= recipNorm
        ay *= recipNorm
        az *= recipNorm

        # // Normalise magnetometer measurement
        m_norm = mx * mx + my * my + mz * mz;
        if (m_norm == 0.): return q # // handle NaN
        recipNorm = 1.0 / math.sqrt(m_norm)
        mx *= recipNorm
        my *= recipNorm
        mz *= recipNorm

        # // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0 * q0 * mx
        _2q0my = 2.0 * q0 * my
        _2q0mz = 2.0 * q0 * mz
        _2q1mx = 2.0 * q1 * mx
        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q0q2 = 2.0 * q0 * q2
        _2q2q3 = 2.0 * q2 * q3
        q0q0 = q0 * q0
        q0q1 = q0 * q1
        q0q2 = q0 * q2
        q0q3 = q0 * q3
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q3q3 = q3 * q3

        #  Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz

        
        # // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        recipNorm = 1.0 / math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)  #// normalise step magnitude
        s0 *= recipNorm
        s1 *= recipNorm
        s2 *= recipNorm
        s3 *= recipNorm

        # // Apply feedback step
        qDot1 -= self.beta * s0
        qDot2 -= self.beta * s1
        qDot3 -= self.beta * s2
        qDot4 -= self.beta * s3

        # // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * self.deltaT
        q1 += qDot2 * self.deltaT
        q2 += qDot3 * self.deltaT
        q3 += qDot4 * self.deltaT

        # // Normalise quaternion
        recipNorm = 1.0 / math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm
        q1 *= recipNorm
        q2 *= recipNorm
        q3 *= recipNorm

        self.q[0] = q0
        self.q[1] = q1
        self.q[2] = q2
        self.q[3] = q3

        return self.q

    def no_filter(self,ax: float, ay: float, az: float, gx: float, gy: float, gz: float, mx: float, my: float, mz: float, q: np.ndarray) -> np.ndarray:
        # // variable for readability
        self.q = q
        q0 = self.q[0]
        q1 = self.q[1]
        q2 = self.q[2]
        q3 = self.q[3]

        self.q[0] += 0.5 * (-q1 * gx - q2 * gy - q3 * gz) * self.deltaT
        self.q[1] += 0.5 * (q0 * gx + q2 * gz - q3 * gy) * self.deltaT
        self.q[2] += 0.5 * (q0 * gy - q1 * gz + q3 * gx) * self.deltaT
        self.q[3] += 0.5 * (q0 * gz + q1 * gy - q2 * gx) * self.deltaT
        recipNorm = 1.0 / math.sqrt(self.q[0] * self.q[0] + self.q[1] * self.q[1] + self.q[2] * self.q[2] + self.q[3] * self.q[3]);
        self.q[0] *= recipNorm
        self.q[1] *= recipNorm
        self.q[2] *= recipNorm
        self.q[3] *= recipNorm
        return self.q

    def mahony(self,ax:float, ay:float, az:float, gx:float, gy:float, gz:float, mx:float, my:float, mz:float, q:np.ndarray)-> np.ndarray:
        self.q = q
        recipNorm = 0.0
        vx = 0.0
        vy = 0.0
        vz = 0.0
        #error terms
        ex = 0.0
        ey = 0.0
        ez = 0.0

        qa = 0
        qb = 0
        qc = 0
        # //integral feedback terms
        ix = 0.0
        iy = 0.0
        iz = 0.0  
        tmp = 0.0
        # Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        tmp = ax * ax + ay * ay + az * az
        if (tmp > 0.0):
            # // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
            recipNorm = 1.0 / math.sqrt(tmp)
            ax *= recipNorm
            ay *= recipNorm
            az *= recipNorm

            # // Estimated direction of gravity in the body frame (factor of two divided out)
            vx = self.q[1] * self.q[3] - self.q[0] * self.q[2]
            vy = self.q[0] * self.q[1] + self.q[2] * self.q[3]
            vz = self.q[0] * self.q[0] - 0.5 + self.q[3] * self.q[3]

            # // Error is cross product between estimated and measured direction of gravity in body frame
            # // (half the actual magnitude)
            ex = (ay * vz - az * vy)
            ey = (az * vx - ax * vz)
            ez = (ax * vy - ay * vx)

            # // Compute and apply to gyro term the integral feedback, if enabled
            if (self.Ki > 0.0):
                ix += self.Ki * ex * self.deltaT  # integral error scaled by Ki
                iy += self.Ki * ey * self.deltaT
                iz += self.Ki * ez * self.deltaT
                gx += ix  # apply integral feedback
                gy += iy
                gz += iz

            # // Apply proportional feedback to gyro term
            gx += self.Kp * ex
            gy += self.Kp * ey
            gz += self.Kp * ez
        #Integrate rate of change of quaternion, q cross gyro term
        self.deltaT = 0.5 * self.deltaT
        gx *= self.deltaT  #pre-multiply common factors
        gy *= self.deltaT
        gz *= self.deltaT
        qa = self.q[0]
        qb = self.q[1]
        qc = self.q[2]

        self.q[0] += (-qb * gx - qc * gy - self.q[3] * gz)
        self.q[1] += (qa * gx + qc * gz - self.q[3] * gy)
        self.q[2] += (qa * gy - qb * gz + self.q[3] * gx)
        self.q[3] += (qa * gz + qb * gy - qc * gx)

        #// renormalise quaternion
        recipNorm = 1.0 / math.sqrt(self.q[0] * self.q[0] + self.q[1] * self.q[1] + self.q[2] * self.q[2] + self.q[3] * self.q[3])
        self.q[0] = self.q[0] * recipNorm
        self.q[1] = self.q[1] * recipNorm
        self.q[2] = self.q[2] * recipNorm
        self.q[3] = self.q[3] * recipNorm
        return self.q