import pandas as pd
from datetime import datetime
import cv2
import sys
import board
import time
from GPS import GroveGPS
import adafruit_bme680

from Radar import RadarXM125, generate_data_stream, data_analysis_mode

def custom_processing(radar):
    """Your custom radar data processing logic"""
    sample_count = 0
    
    while sample_count < 100:  # Or your condition
        data = radar.get_sensor_data()
        
        if data and data['targets']:
            # Your custom analysis here
            closest_target = min(data['targets'], key=lambda x: x['distance_cm'])
            print(f"Closest target: {closest_target['distance_cm']:.1f}cm")
            
            # Your application-specific logic
            if closest_target['distance_cm'] < 50:
                print("⚠️ Close proximity warning!")
def main():
    # CHANGE PINS HERE
    radar = RadarXM125(i2c_addr=0x52, i2c_bus=7, wake_up_pin=7, int_pin=11)
    gps = GroveGPS('/dev/ttyTHS1', 9600)
    
    if not gps.connect():
        return
    try:
        # Initialize
        if not radar.initialize():
            print("Failed to initialize radar")
            return
        
        # Configure
        if not radar.configure_continuous_mode():
            print("Configuration failed")
            return
            
        if not radar.apply_configuration():
            print("Calibration had issues, continuing...")
            
    # Load previous state for hot start
    gps.load_gps_state()
    gps.send_hot_start_commands()
    
    try:
        while True:
            if gps.read_data():
                if gps.data['fix']:
                    lat, lon = gps.get_position()
                    print(f"Current position: {gps.get_formatted_position()}")
    
                    
            time.sleep(1)
    except KeyboardInterrupt:
        gps.close()

if __name__ == "__main__":
    main()

