import time
import cv2
from xm125_source import XM125GPR  # ← THIS IS HOW YOU USE THE SOURCE CODE

# Create radar object using the source code
radar = XM125GPR(ip="192.168.1.125")

# Start scanning
radar.start_scan()

print("Collecting radar data for 10 seconds...")

# Collect data for 10 seconds
for i in range(10):
    time.sleep(1)
    print(f"Second {i+1}: Collected {len(radar.scan_data)} scans")
    
    # Show live image every 2 seconds
    if i % 2 == 0:
        image = radar.get_bscan_image()
        if image is not None:
            display_img = cv2.resize(image, (600, 400))
            cv2.imshow("Live Radar", display_img)
            cv2.waitKey(1)

# Stop scanning
radar.stop_scan()

# Show final result
final_image = radar.get_bscan_image()
if final_image is not None:
    display_img = cv2.resize(final_image, (800, 600))
    cv2.imshow("Final Radar Scan", display_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

print(f"Done! Collected {len(radar.scan_data)} total scans")