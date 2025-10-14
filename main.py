# main.py - Integrated Multi-Sensor Data Acquisition for Jetson Orin Nano
"""
MAIN MULTI-SENSOR DATA ACQUISITION SYSTEM
=========================================
This system integrates 4 different sensors with Jetson Orin Nano:
1. XM125 Ground Penetrating Radar
2. GPS with Hot Start capability  
3. AS7265x Spectral Triad Sensor
4. BME688 Environmental Sensor

All data is collected simultaneously and saved to a single Excel file
with timestamps for correlation analysis.
"""

import time
import threading
import pandas as pd
from datetime import datetime
import cv2
import sys
import json
import serial

# Import sensor classes (make sure these files are in the same directory)
try:
    from xm125_source import XM125GPR  # Radar sensor controller
    from GPS_HotStart import GroveGPS  # GPS with hot start functionality
    import qwiic_as7265x  # Spectral sensor library
except ImportError as e:
    print(f"Import error: {e}")
    print("Please ensure all sensor source files are in the same directory")
    sys.exit(1)

class MultiSensorController:
    """
    MAIN CONTROLLER CLASS
    =====================
    This class manages all 4 sensors, coordinates data collection,
    handles threading, and saves data to Excel files.
    """
    
    def __init__(self):
        """Initialize the multi-sensor controller"""
        # Control flags
        self.running = False  # Main operation flag
        self.data_buffer = []  # Store collected data before saving
        self.data_lock = threading.Lock()  # Thread safety for data access
        
        # Sensor objects dictionary - stores instances of all sensor classes
        self.sensors = {
            'radar': None,      # XM125 Ground Penetrating Radar
            'gps': None,        # GPS with hot start capability
            'spectral': None,   # AS7265x Spectral Triad (18-channel)
            'bme688': None      # BME688 Environmental sensor
        }
        
        # Data structure template - defines what data we collect from each sensor
        self.data_template = {
            # Timestamp for data correlation
            'timestamp': '',
            
            # Radar data (from XM125)
            'radar_scans': 0,           # Number of radar scans collected
            'radar_avg_intensity': 0.0, # Average signal intensity
            
            # GPS data (from GroveGPS)
            'gps_latitude': 0.0,        # Decimal degrees
            'gps_longitude': 0.0,       # Decimal degrees  
            'gps_altitude': 0.0,        # Meters above sea level
            'gps_speed': 0.0,           # Kilometers per hour
            'gps_satellites': 0,        # Number of visible satellites
            
            # Spectral data (from AS7265x) - selected wavelengths
            'spectral_410nm': 0.0,      # Violet light intensity
            'spectral_860nm': 0.0,      # Near-infrared intensity
            'spectral_940nm': 0.0,      # Infrared intensity
            
            # Environmental data (from BME688)
            'temperature': 0.0,         # Degrees Celsius
            'humidity': 0.0,            # Relative humidity percentage
            'pressure': 0.0,            # Atmospheric pressure in kPa
            'gas': 0.0                  # Gas resistance in ppm
        }
    
    def initialize_sensors(self):
        """
        INITIALIZE ALL FOUR SENSORS
        ===========================
        This method attempts to connect to each sensor and initialize them.
        If a sensor fails to initialize, it will continue with the others.
        """
        print("🔄 Initializing sensors...")
        print("-" * 40)
        
        # 1. INITIALIZE XM125 RADAR (Ground Penetrating Radar)
        # ====================================================
        try:
            # Create radar object - assumes XM125 has IP address 192.168.1.125
            self.sensors['radar'] = XM125GPR(ip="192.168.1.125")
            # Start radar scanning - this begins data acquisition
            if self.sensors['radar'].start_scan():
                print("✅ XM125 Radar: Connected and scanning")
            else:
                print("❌ XM125 Radar: Failed to connect or start scanning")
        except Exception as e:
            print(f"❌ XM125 Radar Error: {e}")
        
        # 2. INITIALIZE GPS MODULE
        # ========================
        try:
            # Create GPS object - uses UART0 on pins 0(TX) and 1(RX)
            self.sensors['gps'] = GroveGPS(uart_id=0, baudrate=9600, tx_pin=0, rx_pin=1)
            # Load previous GPS state for faster satellite acquisition (hot start)
            self.sensors['gps'].load_gps_state()
            print("✅ GPS: Initialized with hot start capability")
        except Exception as e:
            print(f"❌ GPS Error: {e}")
        
        # 3. INITIALIZE SPECTRAL SENSOR (AS7265x)
        # =======================================
        try:
            # Create spectral sensor object
            self.sensors['spectral'] = qwiic_as7265x.QwiicAS7265x()
            # Check if sensor is physically connected via I2C
            if self.sensors['spectral'].is_connected():
                # Initialize the sensor for operation
                if self.sensors['spectral'].begin():
                    print("✅ Spectral Sensor: Connected and calibrated")
                else:
                    print("❌ Spectral Sensor: Failed to initialize")
            else:
                print("❌ Spectral Sensor: Not connected - check I2C wiring")
        except Exception as e:
            print(f"❌ Spectral Sensor Error: {e}")
        
        # 4. INITIALIZE BME688 ENVIRONMENTAL SENSOR
        # =========================================
        try:
            # BME688 is connected via RoboBoard using serial communication
            # Open serial port at /dev/ttyACM0 with 115200 baud rate
            self.sensors['bme688'] = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)  # Wait for serial connection to stabilize
            print("✅ BME688: Serial port opened successfully")
        except Exception as e:
            print(f"❌ BME688 Error: {e}")
        
        print("-" * 40)
        print("Sensor initialization complete\n")
    
    def read_radar_data(self):
        """
        READ DATA FROM XM125 RADAR
        ==========================
        Extracts useful information from the radar's scan data:
        - Number of scans collected
        - Average signal intensity of the latest scan
        """
        if not self.sensors['radar']:
            return {}  # Return empty if radar not initialized
        
        try:
            # Count how many radar scans we've collected
            scan_count = len(self.sensors['radar'].scan_data)
            avg_intensity = 0.0
            
            # Calculate intensity if we have scans
            if scan_count > 0:
                # Get the most recent radar scan
                latest_scan = self.sensors['radar'].scan_data[-1]
                # Calculate average intensity (absolute value to ignore negative values)
                if len(latest_scan) > 0:
                    avg_intensity = float(np.mean(np.abs(latest_scan)))
            
            return {
                'radar_scans': scan_count,
                'radar_avg_intensity': avg_intensity
            }
        except Exception as e:
            print(f"Radar read error: {e}")
            return {}  # Return empty dict on error
    
    def read_gps_data(self):
        """
        READ DATA FROM GPS MODULE
        =========================
        Gets position, altitude, speed, and satellite information.
        Uses hot start data if available for faster acquisition.
        """
        if not self.sensors['gps']:
            return {}  # Return empty if GPS not initialized
        
        try:
            # Read and parse the latest GPS data from serial
            self.sensors['gps'].read_data()
            
            # Extract relevant GPS information
            return {
                'gps_latitude': self.sensors['gps'].data['latitude'],
                'gps_longitude': self.sensors['gps'].data['longitude'], 
                'gps_altitude': self.sensors['gps'].data['altitude'],
                'gps_speed': self.sensors['gps'].data['speed'],
                'gps_satellites': self.sensors['gps'].data['satellites']
            }
        except Exception as e:
            print(f"GPS read error: {e}")
            return {}
    
    def read_spectral_data(self):
        """
        READ DATA FROM SPECTRAL SENSOR
        ==============================
        Measures light intensity at 18 different wavelengths.
        We return 3 representative wavelengths for data efficiency.
        """
        if not self.sensors['spectral'] or not self.sensors['spectral'].is_connected():
            return {}  # Return empty if spectral sensor not available
        
        try:
            # Take a measurement - this reads all 18 channels
            self.sensors['spectral'].take_measurements()
            
            # Return selected wavelengths (410nm, 860nm, 940nm)
            return {
                'spectral_410nm': self.sensors['spectral'].get_calibrated_a(),  # 410nm - Violet
                'spectral_860nm': self.sensors['spectral'].get_calibrated_w(),  # 860nm - Near-IR
                'spectral_940nm': self.sensors['spectral'].get_calibrated_l()   # 940nm - Infrared
            }
        except Exception as e:
            print(f"Spectral read error: {e}")
            return {}
    
    def read_bme688_data(self):
        """
        READ DATA FROM BME688 ENVIRONMENTAL SENSOR
        ==========================================
        Reads temperature, humidity, pressure, and gas resistance
        via serial communication with RoboBoard.
        """
        if not self.sensors['bme688']:
            return {}  # Return empty if BME688 not initialized
        
        try:
            # Send read command to RoboBoard via serial
            self.sensors['bme688'].write(b'read\n')
            time.sleep(0.1)  # Wait for response
            
            # Check if data is available to read
            if self.sensors['bme688'].in_waiting > 0:
                # Read and decode the response line
                line = self.sensors['bme688'].readline().decode('utf-8').strip()
                
                # Parse the data based on expected format: "Temp: X.XC, RH: X.X%, Pressure: X.XkPa, Gas: X.Xppm"
                if 'Temp:' in line and 'RH:' in line:
                    # Split the line into components
                    parts = line.split(',')
                    
                    # Extract and convert each value
                    temp = float(parts[0].split(':')[1].replace('C', '').strip())
                    humidity = float(parts[1].split(':')[1].replace('%', '').strip())
                    pressure = float(parts[2].split(':')[1].replace('kPa', '').strip())
                    gas = float(parts[3].split(':')[1].replace('ppm', '').strip())
                    
                    return {
                        'temperature': temp,
                        'humidity': humidity,
                        'pressure': pressure,
                        'gas': gas
                    }
        except Exception as e:
            print(f"BME688 read error: {e}")
        
        return {}  # Return empty if reading fails
    
    def collect_data_sample(self):
        """
        COLLECT ONE DATA SAMPLE FROM ALL SENSORS
        ========================================
        This is the core method that reads all 4 sensors simultaneously
        and packages the data into a unified format.
        """
        # Create precise timestamp for data correlation
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # Millisecond precision
        
        # Start with template data structure
        data_record = self.data_template.copy()
        data_record['timestamp'] = timestamp
        
        # READ FROM ALL 4 SENSORS (in sequence)
        # =====================================
        radar_data = self.read_radar_data()      # Get radar scan info
        gps_data = self.read_gps_data()          # Get position data  
        spectral_data = self.read_spectral_data() # Get light spectrum data
        bme_data = self.read_bme688_data()       # Get environmental data
        
        # COMBINE ALL SENSOR DATA
        # =======================
        # Update the data record with values from all sensors
        # If a sensor fails, its values remain at default (0)
        data_record.update(radar_data)
        data_record.update(gps_data) 
        data_record.update(spectral_data)
        data_record.update(bme_data)
        
        # THREAD-SAFE DATA STORAGE
        # ========================
        # Use lock to prevent data corruption during simultaneous access
        with self.data_lock:
            self.data_buffer.append(data_record)
        
        return data_record
    
    def data_collection_loop(self, interval=5):
        """
        MAIN DATA COLLECTION LOOP
        =========================
        Runs in a separate thread to collect data at regular intervals.
        This prevents the main program from blocking while waiting for sensors.
        """
        print(f"🔄 Starting data collection every {interval} seconds...")
        
        # Continue collecting until stopped
        while self.running:
            start_time = time.time()  # Track timing for precise intervals
            
            # Collect one sample from all sensors
            sample = self.collect_data_sample()
            
            # REAL-TIME STATUS DISPLAY
            # ========================
            print(f"📊 Sample: "
                  f"GPS({sample['gps_satellites']} sats) | "
                  f"Radar({sample['radar_scans']} scans) | "
                  f"Temp({sample['temperature']:.1f}C) | "
                  f"Spectral({sample['spectral_410nm']:.1f})")
            
            # PRECISE TIMING CONTROL
            # ======================
            # Calculate how long to sleep to maintain exact interval
            elapsed = time.time() - start_time
            sleep_time = max(0, interval - elapsed)  # Never negative
            time.sleep(sleep_time)
    
    def save_to_excel(self, filename=None):
        """
        SAVE COLLECTED DATA TO EXCEL FILE
        =================================
        Converts all collected data to pandas DataFrame and saves as Excel.
        Uses openpyxl engine for better compatibility.
        """
        # Generate filename with timestamp if not provided
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"multi_sensor_data_{timestamp}.xlsx"
        
        try:
            # THREAD-SAFE DATA ACCESS
            # =======================
            with self.data_lock:
                if not self.data_buffer:
                    print("❌ No data to save - buffer is empty")
                    return False
                
                # CONVERT TO PANDAS DATAFRAME
                # ===========================
                # DataFrame provides easy Excel export and data analysis
                df = pd.DataFrame(self.data_buffer)
                
                # SAVE TO EXCEL FILE
                # ==================
                df.to_excel(filename, index=False, engine='openpyxl')
                
                print(f"✅ Data successfully saved to: {filename}")
                print(f"📈 Total samples collected: {len(self.data_buffer)}")
                print(f"📊 Data columns: {list(df.columns)}")
                return True
                
        except Exception as e:
            print(f"❌ Error saving to Excel: {e}")
            return False
    
    def live_radar_display(self):
        """
        LIVE RADAR DISPLAY (OPTIONAL)
        =============================
        Shows real-time radar B-scan images in a separate thread.
        This is useful for monitoring radar performance during data collection.
        """
        def display_loop():
            """Internal function that runs in display thread"""
            while self.running:
                try:
                    # Check if we have enough radar data to display
                    if (self.sensors['radar'] and 
                        len(self.sensors['radar'].scan_data) > 10):
                        
                        # Generate B-scan image from radar data
                        bscan = self.sensors['radar'].get_bscan_image()
                        if bscan is not None:
                            # Resize for better display and show
                            display_img = cv2.resize(bscan, (600, 400))
                            cv2.imshow("Live Radar - Press 'q' to close", display_img)
                            
                            # Check for quit key
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break
                    time.sleep(0.1)  # Small delay to prevent CPU overload
                except Exception as e:
                    print(f"Radar display error: {e}")
                    break
            
            # Clean up OpenCV windows
            cv2.destroyAllWindows()
        
        # Start the display in a separate daemon thread
        display_thread = threading.Thread(target=display_loop)
        display_thread.daemon = True  # Thread will exit when main program exits
        display_thread.start()
    
    def start(self, collection_interval=5, duration=300):
        """
        START THE MULTI-SENSOR DATA ACQUISITION SYSTEM
        ==============================================
        This is the main entry point that coordinates everything.
        """
        print("🚀 Starting Multi-Sensor Data Acquisition System")
        print("=" * 60)
        
        # STEP 1: INITIALIZE ALL SENSORS
        # ==============================
        self.initialize_sensors()
        self.running = True  # Set flag to start operations
        
        # STEP 2: START DATA COLLECTION THREAD
        # ====================================
        # Run data collection in background thread to avoid blocking
        collection_thread = threading.Thread(target=self.data_collection_loop, 
                                           args=(collection_interval,))
        collection_thread.daemon = True  # Thread dies when main program exits
        collection_thread.start()
        
        # STEP 3: START OPTIONAL RADAR DISPLAY
        # ====================================
        self.live_radar_display()
        
        # STEP 4: RUN FOR SPECIFIED DURATION
        # ==================================
        print(f"⏰ Collecting data for {duration} seconds...")
        print("Press Ctrl+C to stop early and save data")
        print("-" * 60)
        
        try:
            # Main program sleeps while threads do the work
            time.sleep(duration)
            
        except KeyboardInterrupt:
            # User pressed Ctrl+C - stop gracefully
            print("\n🛑 Stopping early by user request...")
        
        finally:
            # STEP 5: CLEAN SHUTDOWN
            # ======================
            self.stop()
    
    def stop(self):
        """
        GRACEFUL SHUTDOWN PROCEDURE
        ===========================
        Stops all sensors, saves data, and cleans up resources.
        This ensures no data is lost when the program exits.
        """
        print("\n🛑 Stopping sensors and saving data...")
        self.running = False  # Signal threads to stop
        
        # STOP INDIVIDUAL SENSORS
        # =======================
        
        # Stop radar scanning
        if self.sensors['radar']:
            self.sensors['radar'].stop_scan()
            print("✅ Radar stopped")
        
        # Save GPS state for faster startup next time
        if self.sensors['gps'] and self.sensors['gps'].data['fix']:
            self.sensors['gps'].save_gps_state()
            print("✅ GPS state saved")
        
        # Close BME688 serial connection
        if self.sensors['bme688']:
            self.sensors['bme688'].close()
            print("✅ BME688 disconnected")
        
        # STEP 6: SAVE ALL DATA TO EXCEL
        # ==============================
        self.save_to_excel()
        
        # FINAL SUMMARY
        # =============
        print("✅ All sensors stopped gracefully")
        print(f"💾 Total data samples collected: {len(self.data_buffer)}")
        print("🎯 Program completed successfully!")


# SIMPLE USAGE FUNCTIONS
# ======================

def quick_start():
    """
    QUICK START FUNCTION
    ====================
    Simple one-line function for basic data collection.
    Good for testing and quick measurements.
    """
    controller = MultiSensorController()
    controller.start(duration=60)  # Collect for 1 minute

def advanced_start():
    """
    ADVANCED START FUNCTION  
    ======================
    Customizable data collection with user-defined parameters.
    Good for long-term or specialized data collection.
    """
    controller = MultiSensorController()
    
    # Custom settings
    collection_interval = 2  # seconds between samples (more frequent)
    duration = 600  # 10 minutes total collection
    
    controller.start(collection_interval=collection_interval, duration=duration)


# MAIN PROGRAM ENTRY POINT
# ========================

if __name__ == "__main__":
    """
    MAIN PROGRAM EXECUTION
    ======================
    This code runs when the file is executed directly.
    It provides a user interface for selecting operation mode.
    """
    
    # REQUIRED PACKAGES - install these first:
    # pip install pandas openpyxl opencv-python numpy pyserial
    
    print("🤖 Multi-Sensor Data Acquisition System")
    print("======================================")
    print("Supported sensors:")
    print("  • XM125 Ground Penetrating Radar")
    print("  • GPS with Hot Start capability") 
    print("  • AS7265x Spectral Triad Sensor (18 channels)")
    print("  • BME688 Environmental Sensor")
    print()
    
    # USER INTERFACE FOR MODE SELECTION
    # ==================================
    print("Select operation mode:")
    print("1. Quick start (1 minute - good for testing)")
    print("2. Custom duration (you specify how long)")
    print("3. Advanced settings (custom interval and duration)")
    
    try:
        # Get user input
        choice = input("Enter your choice (1-3): ").strip()
        
        # Create the main controller
        controller = MultiSensorController()
        
        # Execute based on user choice
        if choice == "1":
            # Mode 1: Quick test (1 minute)
            controller.start(duration=60)
        elif choice == "2":
            # Mode 2: Custom duration
            duration = int(input("Enter duration in seconds: "))
            controller.start(duration=duration)
        elif choice == "3":
            # Mode 3: Fully customizable
            interval = float(input("Enter collection interval (seconds): "))
            duration = int(input("Enter total duration (seconds): "))
            controller.start(collection_interval=interval, duration=duration)
        else:
            # Default: 5 minutes with standard settings
            print("Starting with default settings (5 minutes)")
            controller.start(duration=300)
            
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        print("\nProgram terminated by user")
    except ValueError:
        # Handle invalid number input
        print("Error: Please enter valid numbers")
    except Exception as e:
        # Handle any other errors
        print(f"Unexpected error: {e}")