
import threading
import time
from slambot_lidar_pkg.lidar.rpilidar import RPLidar

PORT_NAME = 'COM5'

class LidarStreamer:
    def __init__(self, port=PORT_NAME, wedges=500):
        self.lidar = RPLidar(port)
        self.measurements = {}
        self.running = False
        self.thread = None
        self.wedges = wedges
        self.wedge_size = 360 / wedges

    def start(self):
        """Start the LiDAR scanning in a separate thread."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._scan, daemon=True)
            self.thread.start()
            print("LiDAR started...")

    def _scan(self):
        """Continuously read LiDAR data in a loop."""
        try:
            for new_scan, quality, angle, distance in self.lidar.iter_measures(max_buf_meas=4096):
                if not self.running:
                    break  # Stop the loop if the flag is set

                angle = angle % 360
                angle_key = (((angle + (self.wedge_size / 2)) // self.wedge_size) * self.wedge_size) % 360
                self.measurements[str(angle_key)] = distance

        except Exception as e:
            print(f"Error in LiDAR scanning: {e}")
        finally:
            self.stop()

    def get_latest_measurements(self):
        """Return the most recent LiDAR scan data."""
        return self.measurements.copy()  # Return a copy to avoid threading issues

    def stop(self):
        """Stop the LiDAR safely."""
        if self.running:
            print("Stopping LiDAR...")
            self.running = False
            self.lidar.stop()
            self.lidar.disconnect()
            if self.thread and self.thread.is_alive():
                self.thread.join()
