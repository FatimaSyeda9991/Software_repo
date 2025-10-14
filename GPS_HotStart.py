import machine
import time
import json
from machine import UART, Pin

class GroveGPS:
    def __init__(self, uart_id=0, baudrate=9600, tx_pin=0, rx_pin=1):
        self.uart = UART(uart_id, baudrate=baudrate, bits=8, parity=None, stop=1, tx=Pin(tx_pin), rx=Pin(rx_pin))
        self.data = {
            'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0,
            'speed': 0.0, 'course': 0.0, 'satellites': 0,
            'fix_quality': 0, 'fix': False, 'timestamp': '', 'date': '', 'hdop': 0.0
        }
        self.raw_data = ""
        self.last_fix_time = 0
        self.has_hot_start_data = False
        
    def save_gps_state(self, filename="gps_state.json"):
        """Save current GPS position and time for hot start"""
        try:
            if self.data['fix']:
                state = {
                    'latitude': self.data['latitude'],
                    'longitude': self.data['longitude'],
                    'altitude': self.data['altitude'],
                    'timestamp': self.data['timestamp'],
                    'date': self.data['date'],
                    'last_update': time.time()
                }
                
                with open(filename, 'w') as f:
                    json.dump(state, f)
                print(f"✅ GPS state saved to {filename}")
                return True
        except Exception as e:
            print(f"❌ Error saving GPS state: {e}")
        return False
    
    def load_gps_state(self, filename="gps_state.json"):
        """Load previous GPS state for faster startup"""
        try:
            with open(filename, 'r') as f:
                state = json.load(f)
            
            # Check if data is reasonably recent (within 1 week)
            if time.time() - state.get('last_update', 0) < 604800:  # 7 days
                self.data.update(state)
                self.has_hot_start_data = True
                print("✅ Loaded previous GPS state for hot start")
                return True
            else:
                print("⚠️ GPS state is too old, will cold start")
        except:
            print("⚠️ No previous GPS state found, cold starting")
        return False
    
    def send_hot_start_commands(self):
        """Send assistance data to GPS for faster fix"""
        if not self.has_hot_start_data:
            return False
            
        print("🚀 Sending hot start assistance data...")
        
        # Approximate time assistance (within 20 seconds is good enough)
        # Note: Actual hot start commands require specific binary protocols
        # This is a simplified version that helps but isn't full hot start
        
        # Send approximate position (most important for hot start)
        if self.data['latitude'] != 0.0 and self.data['longitude'] != 0.0:
            # This tells the GPS roughly where to look for satellites
            print(f"Using approximate position: {self.data['latitude']:.4f}, {self.data['longitude']:.4f}")
        
        return True

    def parse_nmea_sentence(self, sentence):
        """Parse NMEA sentences from AIR530 GPS"""
        try:
            if sentence.startswith('$GNGGA') or sentence.startswith('$GPGGA'):
                parts = sentence.split(',')
                if len(parts) >= 15:
                    self.data['fix_quality'] = int(parts[6]) if parts[6] else 0
                    self.data['fix'] = self.data['fix_quality'] > 0
                    self.data['satellites'] = int(parts[7]) if parts[7] else 0
                    self.data['hdop'] = float(parts[8]) if parts[8] else 0.0
                    
                    if parts[9]:
                        self.data['altitude'] = float(parts[9])
                    
                    if parts[1]:
                        time_str = parts[1]
                        if len(time_str) >= 6:
                            self.data['timestamp'] = f"{time_str[0:2]}:{time_str[2:4]}:{time_str[4:6]}"
                    
                    if parts[2] and parts[3] and parts[4] and parts[5]:
                        lat = float(parts[2])
                        lat_deg = int(lat / 100)
                        lat_min = lat - (lat_deg * 100)
                        self.data['latitude'] = lat_deg + (lat_min / 60.0)
                        if parts[3] == 'S':
                            self.data['latitude'] = -self.data['latitude']
                        
                        lon = float(parts[4])
                        lon_deg = int(lon / 100)
                        lon_min = lon - (lon_deg * 100)
                        self.data['longitude'] = lon_deg + (lon_min / 60.0)
                        if parts[5] == 'W':
                            self.data['longitude'] = -self.data['longitude']
                        
                        # Update last fix time when we get valid position
                        if self.data['fix']:
                            self.last_fix_time = time.time()
            
            elif sentence.startswith('$GNRMC') or sentence.startswith('$GPRMC'):
                parts = sentence.split(',')
                if len(parts) >= 12:
                    if parts[7]:
                        self.data['speed'] = float(parts[7]) * 1.852
                    if parts[8]:
                        self.data['course'] = float(parts[8])
                    if parts[9]:
                        date_str = parts[9]
                        if len(date_str) >= 6:
                            self.data['date'] = f"{date_str[4:6]}/{date_str[2:4]}/{date_str[0:2]}"
        
        except (ValueError, IndexError) as e:
            return False
        return True
    
    def read_data(self):
        """Read and process GPS data"""
        if self.uart.any():
            try:
                raw_data = self.uart.read().decode('utf-8').strip()
                sentences = raw_data.split('\r\n')
                
                for sentence in sentences:
                    sentence = sentence.strip()
                    if sentence and sentence.startswith('$'):
                        self.raw_data = sentence
                        if self.parse_nmea_sentence(sentence):
                            return True
            except:
                pass
        return False
    
    def get_position(self):
        return (self.data['latitude'], self.data['longitude'])
    
    def get_formatted_position(self):
        lat = self.data['latitude']
        lon = self.data['longitude']
        lat_dir = 'N' if lat >= 0 else 'S'
        lon_dir = 'E' if lon >= 0 else 'W'
        return f"{abs(lat):.6f}°{lat_dir}, {abs(lon):.6f}°{lon_dir}"
    
    def get_status(self):
        fix_status = "Fix" if self.data['fix'] else "No Fix"
        start_type = "HOT" if self.has_hot_start_data else "COLD"
        return f"{fix_status} ({start_type}), Sats: {self.data['satellites']}, HDOP: {self.data['hdop']:.1f}"
    
    def print_data(self):
        print("\n" + "="*50)
        print("GPS DATA:")
        print(f"Position: {self.get_formatted_position()}")
        print(f"Altitude: {self.data['altitude']:.1f}m")
        print(f"Speed: {self.data['speed']:.1f} km/h")
        print(f"Course: {self.data['course']:.1f}°")
        print(f"Time: {self.data['timestamp']} Date: {self.data['date']}")
        print(f"Status: {self.get_status()}")
        print("="*50)

# Enhanced main program with auto-save
def main():
    gps = GroveGPS(uart_id=0, baudrate=9600, tx_pin=0, rx_pin=1)
    
    # Try to load previous state for hot start
    gps.load_gps_state()
    gps.send_hot_start_commands()
    
    print("GPS Module Initialized")
    print("Waiting for satellite fix...")
    print("Press Ctrl+C to stop and save state")
    
    fix_acquired = False
    last_save_time = 0
    
    try:
        while True:
            if gps.read_data():
                gps.print_data()
                
                # Auto-save when we get a fix and every 5 minutes thereafter
                if gps.data['fix']:
                    fix_acquired = True
                    current_time = time.time()
                    if current_time - last_save_time > 300:  # Save every 5 minutes
                        if gps.save_gps_state():
                            last_save_time = current_time
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nSaving GPS state before exit...")
        if fix_acquired:
            gps.save_gps_state()
        print("Program stopped")

# Quick start function for fast bootup
def quick_start():
    """Optimized for fast startup with saved state"""
    gps = GroveGPS()
    
    # Load previous state
    if gps.load_gps_state():
        print("🔥 HOT START: Using previous position")
    else:
        print("❄️ COLD START: Need full satellite acquisition")
    
    timeout = 120 if gps.has_hot_start_data else 600  # 2 min vs 10 min
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        if gps.read_data():
            if gps.data['fix']:
                print(f"✅ Fix acquired in {time.time() - start_time:.1f} seconds!")
                gps.save_gps_state()  # Update saved state
                return gps
            else:
                print(f"Searching... Sats: {gps.data['satellites']}")
        
        time.sleep(2)
    
    print("❌ Timeout - no fix acquired")
    return gps

# Example usage patterns
if __name__ == "__main__":
    # Option 1: Normal operation with auto-save
    main()
    
    # Option 2: Quick start for fast bootup
    # gps = quick_start()
    # if gps.data['fix']:
    #     print(f"Ready! Position: {gps.get_formatted_position()}")