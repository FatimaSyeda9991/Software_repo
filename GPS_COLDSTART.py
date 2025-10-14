import machine
import time
from machine import UART, Pin, I2C
import math

class GroveGPS:
    def __init__(self, uart_id=0, baudrate=9600, tx_pin=0, rx_pin=1):
        """
        Initialize GPS for Seeed Studio Grove GPS (AIR530)
        Default: UART0, GPIO0 (RX), GPIO1 (TX)
        """
        self.uart = UART(uart_id, baudrate=baudrate, bits=8, parity=None, stop=1, tx=Pin(tx_pin), rx=Pin(rx_pin))
        self.data = {
            'latitude': 0.0,
            'longitude': 0.0,
            'altitude': 0.0,
            'speed': 0.0,
            'course': 0.0,
            'satellites': 0,
            'fix_quality': 0,
            'fix': False,
            'timestamp': '',
            'date': '',
            'hdop': 0.0  # Horizontal dilution of precision
        }
        self.raw_data = ""
        
    def parse_nmea_sentence(self, sentence):
        """Parse NMEA sentences from AIR530 GPS"""
        try:
            if sentence.startswith('$GNGGA') or sentence.startswith('$GPGGA'):
                # Global Positioning System Fix Data
                parts = sentence.split(',')
                if len(parts) >= 15:
                    # Fix quality (0=no fix, 1=GPS fix, 2=DGPS fix, etc.)
                    self.data['fix_quality'] = int(parts[6]) if parts[6] else 0
                    self.data['fix'] = self.data['fix_quality'] > 0
                    
                    # Number of satellites
                    self.data['satellites'] = int(parts[7]) if parts[7] else 0
                    
                    # HDOP
                    self.data['hdop'] = float(parts[8]) if parts[8] else 0.0
                    
                    # Altitude
                    if parts[9]:
                        self.data['altitude'] = float(parts[9])
                    
                    # Time
                    if parts[1]:
                        time_str = parts[1]
                        if len(time_str) >= 6:
                            self.data['timestamp'] = f"{time_str[0:2]}:{time_str[2:4]}:{time_str[4:6]}"
                    
                    # Latitude and Longitude
                    if parts[2] and parts[3] and parts[4] and parts[5]:
                        # Parse latitude (format: DDMM.MMMMM)
                        lat = float(parts[2])
                        lat_deg = int(lat / 100)
                        lat_min = lat - (lat_deg * 100)
                        self.data['latitude'] = lat_deg + (lat_min / 60.0)
                        if parts[3] == 'S':
                            self.data['latitude'] = -self.data['latitude']
                        
                        # Parse longitude (format: DDDMM.MMMMM)
                        lon = float(parts[4])
                        lon_deg = int(lon / 100)
                        lon_min = lon - (lon_deg * 100)
                        self.data['longitude'] = lon_deg + (lon_min / 60.0)
                        if parts[5] == 'W':
                            self.data['longitude'] = -self.data['longitude']
            
            elif sentence.startswith('$GNRMC') or sentence.startswith('$GPRMC'):
                # Recommended Minimum Specific GPS/Transit Data
                parts = sentence.split(',')
                if len(parts) >= 12:
                    # Speed over ground (knots)
                    if parts[7]:
                        self.data['speed'] = float(parts[7]) * 1.852  # Convert knots to km/h
                    
                    # Course over ground (degrees)
                    if parts[8]:
                        self.data['course'] = float(parts[8])
                    
                    # Date
                    if parts[9]:
                        date_str = parts[9]
                        if len(date_str) >= 6:
                            self.data['date'] = f"{date_str[4:6]}/{date_str[2:4]}/{date_str[0:2]}"
        
        except (ValueError, IndexError) as e:
            print(f"Parse error: {e}")
            return False
        
        return True
    
    def read_data(self):
        """Read and process GPS data"""
        if self.uart.any():
            try:
                # Read all available data
                raw_data = self.uart.read().decode('utf-8').strip()
                sentences = raw_data.split('\r\n')
                
                for sentence in sentences:
                    sentence = sentence.strip()
                    if sentence and sentence.startswith('$'):
                        self.raw_data = sentence
                        if self.parse_nmea_sentence(sentence):
                            return True
                
            except UnicodeError:
                print("Unicode decode error")
            except Exception as e:
                print(f"Read error: {e}")
        
        return False
    
    def get_position(self):
        """Return latitude and longitude as tuple"""
        return (self.data['latitude'], self.data['longitude'])
    
    def get_formatted_position(self):
        """Return formatted position string"""
        lat = self.data['latitude']
        lon = self.data['longitude']
        lat_dir = 'N' if lat >= 0 else 'S'
        lon_dir = 'E' if lon >= 0 else 'W'
        return f"{abs(lat):.6f}°{lat_dir}, {abs(lon):.6f}°{lon_dir}"
    
    def get_status(self):
        """Return GPS status as string"""
        fix_status = "Fix" if self.data['fix'] else "No Fix"
        return f"{fix_status}, Sats: {self.data['satellites']}, HDOP: {self.data['hdop']:.1f}"
    
    def print_data(self):
        """Print all GPS data"""
        print("\n" + "="*50)
        print("GPS DATA:")
        print(f"Position: {self.get_formatted_position()}")
        print(f"Altitude: {self.data['altitude']:.1f}m")
        print(f"Speed: {self.data['speed']:.1f} km/h")
        print(f"Course: {self.data['course']:.1f}°")
        print(f"Time: {self.data['timestamp']} Date: {self.data['date']}")
        print(f"Status: {self.get_status()}")
        print(f"Raw: {self.raw_data}")
        print("="*50)

# Main program
def main():
    # Initialize GPS - AIR530 uses 9600 baud by default
    gps = GroveGPS(uart_id=0, baudrate=9600, tx_pin=0, rx_pin=1)
    
    print("GPS Module Initialized")
    print("Waiting for satellite fix...")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            if gps.read_data():
                gps.print_data()
            
            # Check if we have a fix
            if gps.data['fix']:
                # Do something with the GPS data
                pass
            
            time.sleep(1)  # Read every second
            
    except KeyboardInterrupt:
        print("\nProgram stopped")

# Alternative: Simple continuous reading
def simple_gps_reader():
    gps = GroveGPS()
    
    while True:
        if gps.read_data():
            if gps.data['fix']:
                print(f"Position: {gps.get_formatted_position()}")
                print(f"Speed: {gps.data['speed']:.1f} km/h, Sats: {gps.data['satellites']}")
            else:
                print("Searching for satellites...")
        
        time.sleep(2)

if __name__ == "__main__":
    # Run the main program
    main()
    
    # Or run the simple reader
    # simple_gps_reader()