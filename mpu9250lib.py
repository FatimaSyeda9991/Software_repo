# mpu9250_complete.py
# Combined MPU9250 driver for Raspberry Pi Pico
# Merged from ak8963.py, mpu6500.py, and mpu9250.py

import ustruct
import utime
from machine import I2C, Pin
from micropython import const

# =============================================================================
# AK8963 Magnetometer Constants and Class
# =============================================================================

# AK8963 Register Addresses
_AK8963_WIA = const(0x00)
_AK8963_HXL = const(0x03)
_AK8963_HXH = const(0x04)
_AK8963_HYL = const(0x05)
_AK8963_HYH = const(0x06)
_AK8963_HZL = const(0x07)
_AK8963_HZH = const(0x08)
_AK8963_ST2 = const(0x09)
_AK8963_CNTL1 = const(0x0a)
_AK8963_ASAX = const(0x10)
_AK8963_ASAY = const(0x11)
_AK8963_ASAZ = const(0x12)

_AK8963_MODE_POWER_DOWN = 0b00000000
AK8963_MODE_SINGLE_MEASURE = 0b00000001
AK8963_MODE_CONTINOUS_MEASURE_1 = 0b00000010  # 8Hz
AK8963_MODE_CONTINOUS_MEASURE_2 = 0b00000110  # 100Hz
_AK8963_MODE_EXTERNAL_TRIGGER_MEASURE = 0b00000100
_AK8963_MODE_SELF_TEST = 0b00001000
_AK8963_MODE_FUSE_ROM_ACCESS = 0b00001111

AK8963_OUTPUT_14_BIT = 0b00000000
AK8963_OUTPUT_16_BIT = 0b00010000

_AK8963_SO_14BIT = 0.6  # μT per digit when 14bit mode
_AK8963_SO_16BIT = 0.15  # μT per digit when 16bit mode

class AK8963:
    """Class which provides interface to AK8963 magnetometer."""
    
    def __init__(
        self, i2c, address=0x0c,
        mode=AK8963_MODE_CONTINOUS_MEASURE_1, output=AK8963_OUTPUT_16_BIT,
        offset=(0, 0, 0), scale=(1, 1, 1)
    ):
        self.i2c = i2c
        self.address = address
        self._offset = offset
        self._scale = scale

        if 0x48 != self.whoami:
            raise RuntimeError("AK8963 not found in I2C bus.")

        # Sensitivity adjustement values
        self._register_char(_AK8963_CNTL1, _AK8963_MODE_FUSE_ROM_ACCESS)
        asax = self._register_char(_AK8963_ASAX)
        asay = self._register_char(_AK8963_ASAY)
        asaz = self._register_char(_AK8963_ASAZ)
        self._register_char(_AK8963_CNTL1, _AK8963_MODE_POWER_DOWN)

        # Should wait atleast 100us before next mode
        self._adjustement = (
            (0.5 * (asax - 128)) / 128 + 1,
            (0.5 * (asay - 128)) / 128 + 1,
            (0.5 * (asaz - 128)) / 128 + 1
        )

        # Power on
        self._register_char(_AK8963_CNTL1, (mode | output))

        if output is AK8963_OUTPUT_16_BIT:
            self._so = _AK8963_SO_16BIT
        else:
            self._so = _AK8963_SO_14BIT

    @property
    def magnetic(self):
        """
        X, Y, Z axis micro-Tesla (uT) as floats.
        """
        xyz = list(self._register_three_shorts(_AK8963_HXL))
        self._register_char(_AK8963_ST2)  # Enable updating readings again

        # Apply factory axial sensitivy adjustements
        xyz[0] *= self._adjustement[0]
        xyz[1] *= self._adjustement[1]
        xyz[2] *= self._adjustement[2]

        # Apply output scale determined in constructor
        so = self._so
        xyz[0] *= so
        xyz[1] *= so
        xyz[2] *= so

        # Apply hard iron ie. offset bias from calibration
        xyz[0] -= self._offset[0]
        xyz[1] -= self._offset[1]
        xyz[2] -= self._offset[2]

        # Apply soft iron ie. scale bias from calibration
        xyz[0] *= self._scale[0]
        xyz[1] *= self._scale[1]
        xyz[2] *= self._scale[2]

        return tuple(xyz)

    @property
    def adjustement(self):
        return self._adjustement

    @property
    def whoami(self):
        """Value of the whoami register."""
        return self._register_char(_AK8963_WIA)

    def calibrate(self, count=256, delay=200):
        """Calibrate magnetometer for hard and soft iron distortions."""
        self._offset = (0, 0, 0)
        self._scale = (1, 1, 1)

        reading = self.magnetic
        minx = maxx = reading[0]
        miny = maxy = reading[1]
        minz = maxz = reading[2]

        while count:
            utime.sleep_ms(delay)
            reading = self.magnetic
            minx = min(minx, reading[0])
            maxx = max(maxx, reading[0])
            miny = min(miny, reading[1])
            maxy = max(maxy, reading[1])
            minz = min(minz, reading[2])
            maxz = max(maxz, reading[2])
            count -= 1

        # Hard iron correction
        offset_x = (maxx + minx) / 2
        offset_y = (maxy + miny) / 2
        offset_z = (maxz + minz) / 2

        self._offset = (offset_x, offset_y, offset_z)

        # Soft iron correction
        avg_delta_x = (maxx - minx) / 2
        avg_delta_y = (maxy - miny) / 2
        avg_delta_z = (maxz - minz) / 2

        avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3

        scale_x = avg_delta / avg_delta_x
        scale_y = avg_delta / avg_delta_y
        scale_z = avg_delta / avg_delta_z

        self._scale = (scale_x, scale_y, scale_z)

        return self._offset, self._scale

    def _register_short(self, register, value=None, buf=bytearray(2)):
        if value is None:
            self.i2c.readfrom_mem_into(self.address, register, buf)
            return ustruct.unpack("<h", buf)[0]

        ustruct.pack_into("<h", buf, 0, value)
        return self.i2c.writeto_mem(self.address, register, buf)

    def _register_three_shorts(self, register, buf=bytearray(6)):
        self.i2c.readfrom_mem_into(self.address, register, buf)
        return ustruct.unpack("<hhh", buf)

    def _register_char(self, register, value=None, buf=bytearray(1)):
        if value is None:
            self.i2c.readfrom_mem_into(self.address, register, buf)
            return buf[0]

        ustruct.pack_into("<b", buf, 0, value)
        return self.i2c.writeto_mem(self.address, register, buf)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        pass

# =============================================================================
# MPU6500 Accelerometer/Gyroscope Constants and Class
# =============================================================================

# MPU6500 Register Addresses
_MPU6500_GYRO_CONFIG = const(0x1b)
_MPU6500_ACCEL_CONFIG = const(0x1c)
_MPU6500_ACCEL_CONFIG2 = const(0x1d)
_MPU6500_ACCEL_XOUT_H = const(0x3b)
_MPU6500_ACCEL_XOUT_L = const(0x3c)
_MPU6500_ACCEL_YOUT_H = const(0x3d)
_MPU6500_ACCEL_YOUT_L = const(0x3e)
_MPU6500_ACCEL_ZOUT_H = const(0x3f)
_MPU6500_ACCEL_ZOUT_L = const(0x40)
_MPU6500_TEMP_OUT_H = const(0x41)
_MPU6500_TEMP_OUT_L = const(0x42)
_MPU6500_GYRO_XOUT_H = const(0x43)
_MPU6500_GYRO_XOUT_L = const(0x44)
_MPU6500_GYRO_YOUT_H = const(0x45)
_MPU6500_GYRO_YOUT_L = const(0x46)
_MPU6500_GYRO_ZOUT_H = const(0x47)
_MPU6500_GYRO_ZOUT_L = const(0x48)
_MPU6500_WHO_AM_I = const(0x75)

# Accelerometer full scale selections
MPU6500_ACCEL_FS_SEL_2G = const(0b00000000)
MPU6500_ACCEL_FS_SEL_4G = const(0b00001000)
MPU6500_ACCEL_FS_SEL_8G = const(0b00010000)
MPU6500_ACCEL_FS_SEL_16G = const(0b00011000)

_MPU6500_ACCEL_SO_2G = 16384  # 1 / 16384 ie. 0.061 mg / digit
_MPU6500_ACCEL_SO_4G = 8192   # 1 / 8192 ie. 0.122 mg / digit
_MPU6500_ACCEL_SO_8G = 4096   # 1 / 4096 ie. 0.244 mg / digit
_MPU6500_ACCEL_SO_16G = 2048  # 1 / 2048 ie. 0.488 mg / digit

# Gyroscope full scale selections
MPU6500_GYRO_FS_SEL_250DPS = const(0b00000000)
MPU6500_GYRO_FS_SEL_500DPS = const(0b00001000)
MPU6500_GYRO_FS_SEL_1000DPS = const(0b00010000)
MPU6500_GYRO_FS_SEL_2000DPS = const(0b00011000)

_MPU6500_GYRO_SO_250DPS = 131
_MPU6500_GYRO_SO_500DPS = 62.5
_MPU6500_GYRO_SO_1000DPS = 32.8
_MPU6500_GYRO_SO_2000DPS = 16.4

_MPU6500_TEMP_SO = 333.87
_MPU6500_TEMP_OFFSET = 21

# Scale factors
SF_G = 1
SF_M_S2 = 9.80665  # 1 g = 9.80665 m/s2 ie. standard gravity
SF_DEG_S = 1
SF_RAD_S = 0.017453292519943  # 1 deg/s is 0.017453292519943 rad/s

class MPU6500:
    """Class which provides interface to MPU6500 6-axis motion tracking device."""
    
    def __init__(
        self, i2c, address=0x68,
        accel_fs=MPU6500_ACCEL_FS_SEL_2G, gyro_fs=MPU6500_GYRO_FS_SEL_250DPS,
        accel_sf=SF_M_S2, gyro_sf=SF_RAD_S,
        gyro_offset=(0, 0, 0)
    ):
        self.i2c = i2c
        self.address = address

        # 0x70 = standalone MPU6500, 0x71 = MPU6250 SIP
        if self.whoami not in [0x71, 0x70]:
            raise RuntimeError("MPU6500 not found in I2C bus.")

        self._accel_so = self._accel_fs(accel_fs)
        self._gyro_so = self._gyro_fs(gyro_fs)
        self._accel_sf = accel_sf
        self._gyro_sf = gyro_sf
        self._gyro_offset = gyro_offset

    @property
    def acceleration(self):
        """
        Acceleration measured by the sensor. By default will return a
        3-tuple of X, Y, Z axis acceleration values in m/s^2 as floats.
        """
        so = self._accel_so
        sf = self._accel_sf

        xyz = self._register_three_shorts(_MPU6500_ACCEL_XOUT_H)
        return tuple([value / so * sf for value in xyz])

    @property
    def gyro(self):
        """
        X, Y, Z radians per second as floats.
        """
        so = self._gyro_so
        sf = self._gyro_sf
        ox, oy, oz = self._gyro_offset

        xyz = self._register_three_shorts(_MPU6500_GYRO_XOUT_H)
        xyz = [value / so * sf for value in xyz]

        xyz[0] -= ox
        xyz[1] -= oy
        xyz[2] -= oz

        return tuple(xyz)

    @property
    def temperature(self):
        """
        Die temperature in celcius as a float.
        """
        temp = self._register_short(_MPU6500_TEMP_OUT_H)
        return ((temp - _MPU6500_TEMP_OFFSET) / _MPU6500_TEMP_SO) + _MPU6500_TEMP_OFFSET

    @property
    def whoami(self):
        """Value of the whoami register."""
        return self._register_char(_MPU6500_WHO_AM_I)

    def calibrate(self, count=256, delay=0):
        """Calibrate gyroscope offsets."""
        ox, oy, oz = (0.0, 0.0, 0.0)
        self._gyro_offset = (0.0, 0.0, 0.0)
        n = float(count)

        while count:
            utime.sleep_ms(delay)
            gx, gy, gz = self.gyro
            ox += gx
            oy += gy
            oz += gz
            count -= 1

        self._gyro_offset = (ox / n, oy / n, oz / n)
        return self._gyro_offset

    def _register_short(self, register, value=None, buf=bytearray(2)):
        if value is None:
            self.i2c.readfrom_mem_into(self.address, register, buf)
            return ustruct.unpack(">h", buf)[0]

        ustruct.pack_into(">h", buf, 0, value)
        return self.i2c.writeto_mem(self.address, register, buf)

    def _register_three_shorts(self, register, buf=bytearray(6)):
        self.i2c.readfrom_mem_into(self.address, register, buf)
        return ustruct.unpack(">hhh", buf)

    def _register_char(self, register, value=None, buf=bytearray(1)):
        if value is None:
            self.i2c.readfrom_mem_into(self.address, register, buf)
            return buf[0]

        ustruct.pack_into("<b", buf, 0, value)
        return self.i2c.writeto_mem(self.address, register, buf)

    def _accel_fs(self, value):
        self._register_char(_MPU6500_ACCEL_CONFIG, value)

        # Return the sensitivity divider
        if MPU6500_ACCEL_FS_SEL_2G == value:
            return _MPU6500_ACCEL_SO_2G
        elif MPU6500_ACCEL_FS_SEL_4G == value:
            return _MPU6500_ACCEL_SO_4G
        elif MPU6500_ACCEL_FS_SEL_8G == value:
            return _MPU6500_ACCEL_SO_8G
        elif MPU6500_ACCEL_FS_SEL_16G == value:
            return _MPU6500_ACCEL_SO_16G

    def _gyro_fs(self, value):
        self._register_char(_MPU6500_GYRO_CONFIG, value)

        # Return the sensitivity divider
        if MPU6500_GYRO_FS_SEL_250DPS == value:
            return _MPU6500_GYRO_SO_250DPS
        elif MPU6500_GYRO_FS_SEL_500DPS == value:
            return _MPU6500_GYRO_SO_500DPS
        elif MPU6500_GYRO_FS_SEL_1000DPS == value:
            return _MPU6500_GYRO_SO_1000DPS
        elif MPU6500_GYRO_FS_SEL_2000DPS == value:
            return _MPU6500_GYRO_SO_2000DPS

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        pass

# =============================================================================
# MPU9250 Main Class
# =============================================================================

# Used for enabling and disabling the I2C bypass access
_MPU9250_INT_PIN_CFG = const(0x37)
_MPU9250_I2C_BYPASS_MASK = const(0b00000010)
_MPU9250_I2C_BYPASS_EN = const(0b00000010)
_MPU9250_I2C_BYPASS_DIS = const(0b00000000)

class MPU9250:
    """Class which provides interface to MPU9250 9-axis motion tracking device."""
    
    def __init__(self, i2c, mpu6500=None, ak8963=None):
        if mpu6500 is None:
            self.mpu6500 = MPU6500(i2c)
        else:
            self.mpu6500 = mpu6500

        # Enable I2C bypass to access AK8963 directly.
        char = self.mpu6500._register_char(_MPU9250_INT_PIN_CFG)
        char &= ~_MPU9250_I2C_BYPASS_MASK  # clear I2C bits
        char |= _MPU9250_I2C_BYPASS_EN
        self.mpu6500._register_char(_MPU9250_INT_PIN_CFG, char)

        if ak8963 is None:
            self.ak8963 = AK8963(i2c)
        else:
            self.ak8963 = ak8963

    @property
    def acceleration(self):
        """
        Acceleration measured by the sensor. By default will return a
        3-tuple of X, Y, Z axis values in m/s^2 as floats.
        """
        return self.mpu6500.acceleration

    @property
    def gyro(self):
        """
        Gyro measured by the sensor. By default will return a 3-tuple of
        X, Y, Z axis values in rad/s as floats.
        """
        return self.mpu6500.gyro

    @property
    def temperature(self):
        """
        Die temperature in celcius as a float.
        """
        return self.mpu6500.temperature

    @property
    def magnetic(self):
        """
        X, Y, Z axis micro-Tesla (uT) as floats.
        """
        return self.ak8963.magnetic

    @property
    def whoami(self):
        """Return MPU6500 whoami value."""
        return self.mpu6500.whoami

    def get_all_data(self):
        """Get all sensor data in one call."""
        return {
            'accel': self.acceleration,
            'gyro': self.gyro,
            'mag': self.magnetic,
            'temp': self.temperature
        }

    def calibrate_gyro(self, count=256, delay=0):
        """Calibrate gyroscope offsets."""
        return self.mpu6500.calibrate(count, delay)

    def calibrate_mag(self, count=256, delay=200):
        """Calibrate magnetometer for hard and soft iron distortions."""
        return self.ak8963.calibrate(count, delay)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        pass

# =============================================================================
# Usage Example and Test Code
# =============================================================================

def main():
    """Main demonstration function."""
    # Initialize I2C
    i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
    
    # Scan for devices
    devices = i2c.scan()
    print("I2C devices found:", [hex(device) for device in devices])
    
    try:
        # Initialize MPU9250
        mpu = MPU9250(i2c)
        print("MPU9250 initialized successfully!")
        print(f"WHO_AM_I: 0x{mpu.whoami:02x}")
        
        # Main reading loop
        print("\nReading MPU9250 sensor data...")
        print("Press Ctrl+C to stop\n")
        
        while True:
            data = mpu.get_all_data()
            
            print("=" * 50)
            print(f"Temperature: {data['temp']:6.2f} °C")
            
            print("Accelerometer (m/s²):")
            accel = data['accel']
            print(f"  X: {accel[0]:8.3f}, Y: {accel[1]:8.3f}, Z: {accel[2]:8.3f}")
            
            print("Gyroscope (rad/s):")
            gyro = data['gyro']
            print(f"  X: {gyro[0]:8.3f}, Y: {gyro[1]:8.3f}, Z: {gyro[2]:8.3f}")
            
            print("Magnetometer (µT):")
            mag = data['mag']
            print(f"  X: {mag[0]:8.2f}, Y: {mag[1]:8.2f}, Z: {mag[2]:8.2f}")
            
            utime.sleep(1)
            
    except Exception as e:
        print(f"Error: {e}")

def quick_test():
    """Quick test to verify basic functionality."""
    i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
    
    try:
        mpu = MPU9250(i2c)
        print("Quick test - MPU9250")
        
        for i in range(5):
            data = mpu.get_all_data()
            print(f"Temp: {data['temp']:5.1f}°C | "
                  f"Accel Z: {data['accel'][2]:6.2f}m/s² | "
                  f"Gyro Z: {data['gyro'][2]:6.2f}rad/s")
            utime.sleep(0.5)
            
    except Exception as e:
        print(f"Quick test failed: {e}")

# Run the example if this file is executed directly
if __name__ == "__main__":
    main()
    # quick_test()  # Uncomment for quick test instead