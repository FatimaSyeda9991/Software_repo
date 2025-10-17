"""
MPU9250 IMU Sensor Test
Reads accelerometer, gyroscope, and temperature data from MPU9250
"""
# libraries
from machine import I2C, Pin
import utime
import ustruct

class SimpleMPU9250:
    #initializing MPU9250
    
    def __init__(self, i2c=None, sda_pin=0, scl_pin=1, freq=100000):
        
        # Initialize I2C communication
        if i2c is None:
            # Create new I2C instance if not provided
            self.i2c = I2C(0, sda=Pin(sda_pin), scl=Pin(scl_pin), freq=freq)
        else:
            # Use provided I2C instance
            self.i2c = i2c
        
        # MPU9250 I2C address (0x68 or 0x69 depending on AD0 pin)
        self.addr = 0x68
        
        # Wake up the device by writing 0 to PWR_MGMT_1 register (0x6B)
        # This clears the sleep bit and starts the sensor
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
        utime.sleep_ms(100)  # Wait for sensor to stabilize
        
        print("✓ Simple MPU9250 initialized successfully")
        print(f"  I2C Address: 0x{self.addr:02x}")
        print(f"  I2C Frequency: {freq} Hz")
    
    def get_all_data(self):
        """
        Read all available sensor data from MPU9250
        Returns:
            Dictionary containing:
            - accel: Tuple of (X, Y, Z) acceleration in m/s²
            - gyro: Tuple of (X, Y, Z) angular velocity in rad/s  
            - temp: Temperature in Celsius
            - mag: Tuple of (X, Y, Z) magnetic field in µT (not implemented)
        """
        
        # Read 14 bytes starting from ACCEL_XOUT_H register (0x3B)
        # This reads: Accelerometer (6 bytes) + Temperature (2 bytes) + Gyroscope (6 bytes)
        data = self.i2c.readfrom_mem(self.addr, 0x3B, 14)
        
        # ===========================================================================
        # ACCELEROMETER DATA PROCESSING
        # ===========================================================================
        # Raw accelerometer data is 16-bit signed integers
        # Scale factor: ±8g (can measure from -8g to +8g) 16g range, range = 4096 LSB/g
        # Convert to m/s² by multiplying by 9.80665 (standard gravity)
        
        # Convert raw bytes to acceleration values in m/s²
        ax = self._convert(data[0], data[1]) / 4096.0 * 9.80665  # X-axis acceleration
        ay = self._convert(data[2], data[3]) / 4096.0 * 9.80665  # Y-axis acceleration  
        az = self._convert(data[4], data[5]) / 4096.0 * 9.80665  # Z-axis acceleration
        
        # Also calculate magnitude of acceleration vector
        accel_magnitude = (ax**2 + ay**2 + az**2)**0.5
        
        # ===========================================================================
        # TEMPERATURE DATA PROCESSING  
        # ===========================================================================
        # Temperature sensor data conversion:
        # Formula: Temperature = (TEMP_OUT / 333.87) + 21.0
        # This converts raw sensor reading to Celsius
        temp = self._convert(data[6], data[7]) / 333.87 + 21.0
        
        # ===========================================================================
        # GYROSCOPE DATA PROCESSING
        # ===========================================================================
        # Raw gyroscope data is 16-bit signed integers
        # Scale factor: ±1000dps range = 32.8 LSB/°/s
        # Convert to rad/s by multiplying by 0.0174533 (degrees to radians)
        
        # Convert raw bytes to angular velocity in rad/s
        gx = self._convert(data[8], data[9]) / 32.8 * 0.0174533   # X-axis rotation
        gy = self._convert(data[10], data[11]) / 32.8 * 0.0174533 # Y-axis rotation
        gz = self._convert(data[12], data[13]) / 32.8 * 0.0174533 # Z-axis rotation
        
        # Also calculate magnitude of gyroscope vector
        gyro_magnitude = (gx**2 + gy**2 + gz**2)**0.5
        
        return {
            'accel': (ax, ay, az),                    # Linear acceleration (m/s²)
            'accel_magnitude': accel_magnitude,       # Total acceleration magnitude
            'gyro': (gx, gy, gz),                     # Angular velocity (rad/s)
            'gyro_magnitude': gyro_magnitude,         # Total rotation magnitude
            'temp': temp,                             # Temperature (°C)
            'mag': (0, 0, 0)                          # Magnetometer data (not implemented)
        }
    
    def _convert(self, high, low):
        """
        Convert two bytes to signed 16-bit integer
        
        Args:
            high: High byte of the 16-bit value
            low: Low byte of the 16-bit value
            
        Returns:
            Signed 16-bit integer value
        """
        # Combine two bytes into 16-bit value
        value = (high << 8) | low
        
        # Convert from unsigned to signed 16-bit integer
        if value >= 32768:  # If value is negative (two's complement)
            value -= 65536   # Convert to signed integer
            
        return value

# ===========================================================================
# MAIN TEST PROGRAM
# ===========================================================================
print("\n" + "="*70)
print("MPU9250 IMU SENSOR COMPREHENSIVE TEST")
print("="*70)
print("Sensor: MPU9250 9-Axis IMU (Accelerometer + Gyroscope + Magnetometer)")
print("Interface: I2C")
print("Address: 0x68")
print("Wiring: SDA→GP0, SCL→GP1, VCC→3.3V, GND→GND")
print("="*70)

try:
    # Initialize the MPU9250 sensor
    print("\nInitializing sensor...")
    mpu = SimpleMPU9250(sda_pin=0, scl_pin=1, freq=100000)
    
    print("\n" + "="*70)
    print("STARTING CONTINUOUS SENSOR READINGS")
    print("="*70)
    print("Expected values when stationary:")
    print("  - Acceleration: ~9.8 m/s² on axis pointing downward (gravity)")
    print("  - Gyroscope: ~0 rad/s (no rotation)")
    print("  - Temperature: ~20-40°C (ambient to operating temp)")
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
        
        # Display formatted output with all sensor values
        print("{:<6} {:<8.1f} {:<12.3f} {:<12.3f} {:<12.3f} {:<12.3f} {:<12.3f} {:<12.3f} {:<12.3f} {:<12.3f}".format(
            count, temp, accel_x, accel_y, accel_z, accel_mag, gyro_x, gyro_y, gyro_z, gyro_mag
        ))
        
        # Brief pause between readings (1 second interval)
        utime.sleep(1)
        
except KeyboardInterrupt:
    # Handle graceful shutdown when user presses Ctrl+C
    print("\n" + "="*70)
    print("TEST STOPPED BY USER")
    print("="*70)
    print(f"Total readings: {count}")
    print("Sensor shutdown complete")
    
except Exception as e:
    # Handle any other errors
    print("\n" + "="*70)
    print("ERROR OCCURRED")
    print("="*70)
    print(f"Error type: {type(e).__name__}")
    print(f"Error message: {e}")
    print("\nTroubleshooting tips:")
    print("1. Check wiring: SDA→GP0, SCL→GP1, VCC→3.3V, GND→GND")
    print("2. Verify I2C pull-up resistors (4.7kΩ recommended)")
    print("3. Ensure MPU9250 is properly powered")
    print("4. Check for I2C address conflicts")

# Final message
print("\n" + "="*70)
print("MPU9250 TEST COMPLETE")
print("="*70)