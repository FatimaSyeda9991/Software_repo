import socket
import threading
import time
import numpy as np
import cv2

class XM125GPR:
    def __init__(self, ip="192.168.1.125", port=1234):
        self.ip = ip
        self.port = port
        self.socket = None
        self.running = False
        self.scan_data = []
    
    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.settimeout(1.0)
            print(f"Connected to XM125 at {self.ip}:{self.port}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def start_scan(self):
        if not self.connect():
            return False
            
        self.running = True
        self.data_thread = threading.Thread(target=self._receive_data)
        self.data_thread.daemon = True
        self.data_thread.start()
        
        try:
            self.socket.sendto(b'\x01', (self.ip, self.port))
        except:
            pass
            
        print("Radar scanning started")
        return True
    
    def stop_scan(self):
        self.running = False
        try:
            self.socket.sendto(b'\x02', (self.ip, self.port))
            self.socket.close()
        except:
            pass
        print("Radar scanning stopped")
    
    def _receive_data(self):
        while self.running:
            try:
                data, addr = self.socket.recvfrom(2048)
                if data:
                    samples = np.frombuffer(data, dtype=np.int16)
                    self.scan_data.append(samples)
            except socket.timeout:
                continue
            except:
                pass
    
    def get_bscan_image(self):
        if len(self.scan_data) < 2:
            return None
            
        bscan = np.array(self.scan_data).T
        bscan_normalized = cv2.normalize(bscan, None, 0, 255, cv2.NORM_MINMAX)
        bscan_uint8 = bscan_normalized.astype(np.uint8)
        bscan_color = cv2.applyColorMap(bscan_uint8, cv2.COLORMAP_JET)
        return bscan_color