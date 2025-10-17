"""
MPU9250 IMU Sensor Test - Standard Python Version
Reads accelerometer, gyroscope, and temperature data from MPU9250
"""

import time
import struct
from smbus2 import SMBus

class SimpleMPU9250:
    def __init__(self, bus_number=7, address=0x68):
        """Initialize MPU9250 sensor."""
        
        # Initialize I2C bus
        self.bus = SMBus(bus_number)
        self.addr = address
        
        # Wake up the device by writing 0 to PWR_MGMT_1 register (0x6B)
        self.bus.write_byte_data(self.addr, 0x6B, 0x00)
        time.sleep(0.1)  # Wait for sensor to stabilize
        
        print("✓ Simple MPU9250 initialized successfully")
        print(f"  I2C Bus: {bus_number}")
        print(f"  I2C Address: 0x{self.addr:02x}")
    
    def get_all_data(self):
        """
        Read all available sensor data from MPU9250
        Returns dictionary with sensor data.
        """
        # Read 14 bytes starting from ACCEL_XOUT_H register (0x3B)
        data = self.bus.read_i2c_block_data(self.addr, 0x3B, 14)
        
        # Convert raw bytes to acceleration values in m/s²
        ax = self._convert(data[0], data[1]) / 4096.0 * 9.80665
        ay = self._convert(data[2], data[3]) / 4096.0 * 9.80665  
        az = self._convert(data[4], data[5]) / 4096.0 * 9.80665
        accel_magnitude = (ax**2 + ay**2 + az**2)**0.5
        
        # Temperature conversion
        temp = self._convert(data[6], data[7]) / 333.87 + 21.0
        
        # Convert raw bytes to angular velocity in rad/s
        gx = self._convert(data[8], data[9]) / 32.8 * 0.0174533
        gy = self._convert(data[10], data[11]) / 32.8 * 0.0174533
        gz = self._convert(data[12], data[13]) / 32.8 * 0.0174533
        gyro_magnitude = (gx**2 + gy**2 + gz**2)**0.5
        
        return {
            'accel': (ax, ay, az),
            'accel_magnitude': accel_magnitude,
            'gyro': (gx, gy, gz),
            'gyro_magnitude': gyro_magnitude,
            'temp': temp,
            'mag': (0, 0, 0)  # Magnetometer not implemented in simple version
        }
    
    def _convert(self, high, low):
        """Convert two bytes to signed 16-bit integer."""
        value = (high << 8) | low
        if value >= 32768:
            value -= 65536
        return value
    
    def close(self):
        """Close I2C bus."""
        self.bus.close()

# ===========================================================================
# MAIN TEST PROGRAM
# ===========================================================================
print("\n" + "="*70)
print("MPU9250 IMU SENSOR TEST - STANDARD PYTHON")
print("="*70)
print("Sensor: MPU9250 9-Axis IMU")
print("Interface: I2C")
print("Bus: 7")
print("Address: 0x68")
print("="*70)

try:
    # Initialize the MPU9250 sensor
    print("\nInitializing sensor...")
    mpu = SimpleMPU9250(bus_number=7)
    
    print("\n" + "="*70)
    print("STARTING CONTINUOUS SENSOR READINGS")
    print("="*70)
    print("Press Ctrl+C to stop reading")
    print("="*70)
    
    # Display column headers
    print("\n{:<6} {:<8} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12}".format(
        "Count", "Temp(C)", "Accel-X", "Accel-Y", "Accel-Z", "Accel-Mag", "Gyro-X", "Gyro-Y", "Gyro-Z", "Gyro-Mag"
    ))
    print("{:<6} {:<8} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12}".format(
        "", "", "(m/s²)", "(m/s²)", "(m/s²)", "(m/s²)", "(rad/s)", "(rad/s)", "(rad/s)", "(rad/s)"
    ))
    print("-" * 120)
    
    count = 0
    
    while True:
        # Read all sensor data
        data = mpu.get_all_data()
        count += 1
        
        # Extract individual values
        temp = data['temp']
        accel_x, accel_y, accel_z = data['accel']
        accel_mag = data['accel_magnitude']
        gyro_x, gyro_y, gyro_z = data['gyro']
        gyro_mag = data['gyro_magnitude']
        
        # Display formatted output
        print("{:<6} {:<8.1f} {:<12.3f} {:<12.3f} {:<12.3f} {:<12.3f} {:<12.3f} {:<12.3f} {:<12.3f} {:<12.3f}".format(
            count, temp, accel_x, accel_y, accel_z, accel_mag, gyro_x, gyro_y, gyro_z, gyro_mag
        ))
        
        time.sleep(1)
        
except KeyboardInterrupt:
    print("\n" + "="*70)
    print("TEST STOPPED BY USER")
    print("="*70)
    print(f"Total readings: {count}")
    mpu.close()
    
except Exception as e:
    print("\n" + "="*70)
    print("ERROR OCCURRED")
    print("="*70)
    print(f"Error type: {type(e).__name__}")
    print(f"Error message: {e}")
    print("\nTroubleshooting tips:")
    print("1. Check I2C wiring and connections")
    print("2. Verify sensor is powered (3.3V)")
    print("3. Check if I2C device is detected: sudo i2cdetect -y 7")
    print("4. Ensure smbus2 is installed: pip3 install smbus2")

# Final message
print("\n" + "="*70)
print("MPU9250 TEST COMPLETE")
print("="*70)