#!/usr/bin/env python3
import smbus2
import time
import struct
import RPi.GPIO as GPIO
import json
from datetime import datetime

# ============================
# XM125(A121) PCR Radar Module
# Continuous Data Generation Mode
# ============================

# Constants
I2C_ADDR = 0x52
I2C_BUS = 7

# GPIO Pins
WAKE_UP_PIN = 7
INT_PIN = 11

# Register Addresses
REGISTERS = {
    'DETECTOR_START': 0x0040,
    'DETECTOR_END': 0x0041,
    'MAX_STEP_LENGTH': 0x0042,
    'CLOSE_RANGE_LEAKAGE_CANCELLATION': 0x0043,
    'SIGNAL_QUALITY': 0x0044,
    'MAX_PROFILE': 0x0045,
    'THRESHOLD_METHOD': 0x0046,
    'PEAK_SORTING': 0x0047,
    'NUM_FRAMES_RECORDED_THRESHOLD': 0x0048,
    'FIXED_THRESHOLD_VALUE': 0x0049,
    'THRESHOLD_SENSITIVITY': 0x004A,
    'REFLECTOR_SHAPE': 0x004B,
    'FIXED_STRENGTH_THRESHOLD': 0x004C,
    'MEASURE_ON_WAKEUP': 0x0080,
    'COMMAND': 0x0100,
}

# Command Values
COMMAND_APPLY_CONFIG_AND_CALIBRATE = 1
COMMAND_MEASURE_DISTANCE = 2

# Initialize I2C and GPIO
bus = smbus2.SMBus(I2C_BUS)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(WAKE_UP_PIN, GPIO.OUT)
GPIO.setup(INT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def read_unsigned_register(reg_addr):
    """Reads a 32-bit unsigned register."""
    try:
        addr_high = (reg_addr >> 8) & 0xFF
        addr_low = reg_addr & 0xFF
        bus.write_byte_data(I2C_ADDR, addr_high, addr_low)
        time.sleep(0.01)
        data = bus.read_i2c_block_data(I2C_ADDR, 0x00, 4)
        return struct.unpack('>I', bytes(data))[0]
    except Exception as e:
        print(f"Read error 0x{reg_addr:04X}: {e}")
        return None

def read_signed_register(reg_addr):
    """Reads a 32-bit signed register."""
    try:
        addr_high = (reg_addr >> 8) & 0xFF
        addr_low = reg_addr & 0xFF
        bus.write_byte_data(I2C_ADDR, addr_high, addr_low)
        time.sleep(0.01)
        data = bus.read_i2c_block_data(I2C_ADDR, 0x00, 4)
        return struct.unpack('>i', bytes(data))[0]
    except Exception as e:
        print(f"Read signed error 0x{reg_addr:04X}: {e}")
        return None

def write_register(reg_addr, value):
    """Writes a 32-bit value to a register."""
    try:
        addr_high = (reg_addr >> 8) & 0xFF
        addr_low = reg_addr & 0xFF
        value_bytes = struct.pack('>I', value)
        data = [addr_low] + list(value_bytes)
        bus.write_i2c_block_data(I2C_ADDR, addr_high, data)
        return True
    except Exception as e:
        print(f"Write error 0x{reg_addr:04X}: {e}")
        return False

def initialize_module():
    """Initializes the XM125 module."""
    print("Initializing module...")
    GPIO.output(WAKE_UP_PIN, GPIO.LOW)
    time.sleep(0.2)
    GPIO.output(WAKE_UP_PIN, GPIO.HIGH)
    time.sleep(1.0)
    return True

def configure_continuous_mode():
    """
    Configures the radar for continuous data generation.
    Wide range with maximum sensitivity.
    """
    print("Configuring for continuous data generation...")
    
    # Maximum range configuration (up to ~10 meters)
    config = {
        # Maximum possible range (in mm)
        REGISTERS['DETECTOR_START']: 10,        # 1cm min
        REGISTERS['DETECTOR_END']: 10000,       # 10m max
        
        # Signal processing - maximum sensitivity
        REGISTERS['MAX_STEP_LENGTH']: 0,        # Auto step length
        REGISTERS['CLOSE_RANGE_LEAKAGE_CANCELLATION']: 1,
        REGISTERS['SIGNAL_QUALITY']: 30000,     # High sensitivity
        
        # Profile - balanced for wide range
        REGISTERS['MAX_PROFILE']: 3,            # Profile 3 for long range
        
        # Threshold - sensitive to detect everything
        REGISTERS['THRESHOLD_METHOD']: 3,       # CFAR
        REGISTERS['PEAK_SORTING']: 2,           # Strongest first
        REGISTERS['THRESHOLD_SENSITIVITY']: 300, # Sensitive CFAR
        
        # Reflector - generic for all types
        REGISTERS['REFLECTOR_SHAPE']: 1,        # Generic
        
        # Other settings
        REGISTERS['NUM_FRAMES_RECORDED_THRESHOLD']: 10,
        REGISTERS['MEASURE_ON_WAKEUP']: 0,
    }
    
    # Write configuration
    for reg, value in config.items():
        if write_register(reg, value):
            print(f"✓ Config 0x{reg:04X} = {value}")
        else:
            print(f"✗ Failed: 0x{reg:04X} = {value}")
        time.sleep(0.02)
    
    return True

def apply_configuration():
    """Applies configuration and calibrates."""
    print("Applying configuration...")
    
    if not write_register(REGISTERS['COMMAND'], COMMAND_APPLY_CONFIG_AND_CALIBRATE):
        return False
    
    # Wait for calibration
    timeout = 15
    start_time = time.time()
    while time.time() - start_time < timeout:
        status = read_unsigned_register(0x0003)
        if status is not None:
            busy = (status >> 31) & 0x1
            if not busy:
                error_flags = (status >> 16) & 0xFF
                if error_flags == 0:
                    print("✓ Configuration applied successfully")
                    return True
                else:
                    print(f"Calibration warnings: 0x{error_flags:02X}")
                    return True  # Continue anyway
        time.sleep(0.2)
    
    print("Calibration timeout")
    return False

def get_comprehensive_sensor_data():
    """
    Collects comprehensive sensor data including:
    - Multiple target distances and strengths
    - Environmental data
    - System status
    - Raw measurement data
    """
    timestamp = datetime.now().isoformat()
    
    # Send measurement command
    if not write_register(REGISTERS['COMMAND'], COMMAND_MEASURE_DISTANCE):
        return None
    
    # Wait for measurement
    timeout = 3
    start_time = time.time()
    while time.time() - start_time < timeout:
        status = read_unsigned_register(0x0003)
        if status is not None:
            busy = (status >> 31) & 0x1
            if not busy:
                break
        time.sleep(0.1)
    else:
        return None
    
    # Build comprehensive data packet
    data_packet = {
        'timestamp': timestamp,
        'system': {},
        'targets': [],
        'environment': {},
        'raw': {}
    }
    
    # System information
    data_packet['system']['measurement_counter'] = read_unsigned_register(0x0002)
    data_packet['system']['protocol_status'] = read_unsigned_register(0x0001)
    data_packet['system']['detector_status'] = read_unsigned_register(0x0003)
    
    # Distance results
    distance_result = read_unsigned_register(0x0010)
    if distance_result is not None:
        data_packet['raw']['distance_result'] = distance_result
        num_targets = distance_result & 0x0000000F
        data_packet['system']['num_targets'] = num_targets
        data_packet['system']['measurement_error'] = (distance_result >> 10) & 0x1
        
        # Read all detected targets
        for i in range(min(num_targets, 10)):  # Up to 10 targets
            dist_raw = read_unsigned_register(0x0011 + i)
            strength_raw = read_signed_register(0x001B + i)
            
            if dist_raw is not None and strength_raw is not None:
                target_data = {
                    'id': i,
                    'distance_m': dist_raw / 1000.0,
                    'distance_cm': dist_raw / 10.0,
                    'strength_db': strength_raw / 1000.0,
                    'raw_distance': dist_raw,
                    'raw_strength': strength_raw
                }
                data_packet['targets'].append(target_data)
    
    # Environmental data
    ambient_level = read_signed_register(0x0025)
    if ambient_level is not None:
        data_packet['environment']['ambient_level_db'] = ambient_level / 1000.0
        data_packet['environment']['raw_ambient'] = ambient_level
    
    # Additional sensor data
    temperature = read_signed_register(0x0026)  # If available
    if temperature is not None:
        data_packet['environment']['temperature_c'] = temperature / 1000.0
    
    return data_packet

def generate_data_stream():
    """
    Continuous data generation with multiple output formats.
    """
    print("\n🚀 STARTING CONTINUOUS DATA STREAM")
    print("=" * 60)
    print("Data Formats:")
    print("- JSON: Structured data for applications")
    print("- CSV: Comma-separated for spreadsheets")
    print("- RAW: Raw sensor readings")
    print("- SIMPLE: Human-readable summary")
    print("=" * 60)
    print("Press Ctrl+C to stop\n")
    
    stream_id = 0
    try:
        while True:
            stream_id += 1
            data = get_comprehensive_sensor_data()
            
            if data:
                # Output in multiple formats
                print(f"\n📊 STREAM #{stream_id} - {data['timestamp']}")
                print("-" * 50)
                
                # JSON Output (full data)
                json_output = json.dumps(data, indent=2)
                print("🎯 JSON:")
                print(json_output)
                
                # CSV-like output
                print("\n📈 CSV Format:")
                targets = data.get('targets', [])
                if targets:
                    print("TargetID,DistanceCM,StrengthDB,RawDist,RawStrength")
                    for target in targets:
                        print(f"{target['id']},{target['distance_cm']:.2f},"
                              f"{target['strength_db']:.2f},{target['raw_distance']},"
                              f"{target['raw_strength']}")
                else:
                    print("No targets detected")
                
                # Simple summary
                print(f"\n📋 Summary: {len(targets)} targets | "
                      f"Ambient: {data.get('environment', {}).get('ambient_level_db', 0):.1f}dB | "
                      f"Counter: {data['system'].get('measurement_counter', 0)}")
                
                # Raw hex data for debugging
                print(f"\n🔧 Raw: Status=0x{data['system'].get('detector_status', 0):08X} | "
                      f"DistResult=0x{data['raw'].get('distance_result', 0):08X}")
                
            else:
                print(f"\n❌ STREAM #{stream_id} - FAILED")
            
            print("=" * 60)
            time.sleep(0.2)  # Very fast updates for continuous stream
            
    except KeyboardInterrupt:
        print(f"\n🛑 DATA STREAM STOPPED")
        print(f"Total streams generated: {stream_id}")

def data_analysis_mode():
    """
    Alternative mode with real-time data analysis.
    """
    print("\n🔍 STARTING ANALYTICS MODE")
    print("=" * 50)
    
    stream_count = 0
    target_history = []
    
    try:
        while True:
            stream_count += 1
            data = get_comprehensive_sensor_data()
            
            if data and 'targets' in data:
                targets = data['targets']
                target_history.extend([t['distance_cm'] for t in targets])
                
                # Keep only recent history
                if len(target_history) > 100:
                    target_history = target_history[-100:]
                
                print(f"\n📈 Analytics Frame #{stream_count}")
                print(f"📊 Current: {len(targets)} targets")
                
                if targets:
                    # Statistical analysis
                    distances = [t['distance_cm'] for t in targets]
                    strengths = [t['strength_db'] for t in targets]
                    
                    avg_dist = sum(distances) / len(distances)
                    max_strength = max(strengths)
                    
                    print(f"📍 Avg Distance: {avg_dist:.1f}cm")
                    print(f"💪 Max Strength: {max_strength:.1f}dB")
                    print(f"📏 Distance Range: {min(distances):.1f} - {max(distances):.1f}cm")
                    
                    # Target classification
                    close_targets = len([d for d in distances if d < 50])
                    far_targets = len([d for d in distances if d >= 50])
                    
                    print(f"🎯 Classification: {close_targets} close, {far_targets} far targets")
                
                # Movement detection (simple)
                if len(target_history) > 10:
                    recent_avg = sum(target_history[-10:]) / 10
                    overall_avg = sum(target_history) / len(target_history)
                    movement = abs(recent_avg - overall_avg)
                    
                    if movement > 5:
                        print(f"🚀 Movement detected: Δ{movement:.1f}cm")
                
            time.sleep(0.3)
            
    except KeyboardInterrupt:
        print(f"\n📊 Analytics Summary: Processed {stream_count} frames")

def main():
    """Main function for continuous data generation."""
    try:
        print("XM125 Radar - Continuous Data Generation")
        print("=" * 50)
        
        # Initialize
        if not initialize_module():
            return
        
        # Configure
        if not configure_continuous_mode():
            return
        
        if not apply_configuration():
            print("Warning: Continuing with potential configuration issues")
        
        # Let user choose mode
        print("\nSelect Data Mode:")
        print("1. Raw Data Stream (All formats)")
        print("2. Analytics Mode (With analysis)")
        print("3. High-Speed Stream (Minimal output)")
        
        try:
            choice = input("Enter choice (1-3, default=1): ").strip()
        except:
            choice = "1"
        
        if choice == "2":
            data_analysis_mode()
        elif choice == "3":
            print("\nStarting high-speed mode...")
            high_speed_count = 0
            try:
                while True:
                    data = get_comprehensive_sensor_data()
                    high_speed_count += 1
                    if data and high_speed_count % 10 == 0:
                        targets = len(data.get('targets', []))
                        print(f"Frame {high_speed_count}: {targets} targets")
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print(f"\nHigh-speed frames: {high_speed_count}")
        else:
            generate_data_stream()
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Cleaning up...")
        GPIO.output(WAKE_UP_PIN, GPIO.LOW)
        GPIO.cleanup()
        bus.close()

if __name__ == "__main__":
    main()
