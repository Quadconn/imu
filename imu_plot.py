import serial
import re
import time
import threading
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = '/dev/ttyUSB0'  # Windows users: change to 'COM3', 'COM4', etc.
BAUD_RATE = 115200            # Adjust to match your IMU settings
SAMPLE_RATE = 1000            # Hz

# We want 10 seconds of TOTAL visible X-axis width.
# Because the newest point is centered, you will see 5 seconds of history 
# on the left, and 5 seconds of empty "future" space on the right.
TOTAL_X_AXIS_SECONDS = 10     
HALF_WINDOW = TOTAL_X_AXIS_SECONDS / 2

# We only need to buffer enough data to fill the left half of the screen
MAX_POINTS = int(SAMPLE_RATE * HALF_WINDOW)

# ==========================================
# DATA STRUCTURES
# ==========================================
t_data  = deque(maxlen=MAX_POINTS)
ax_data = deque(maxlen=MAX_POINTS)
ay_data = deque(maxlen=MAX_POINTS)
az_data = deque(maxlen=MAX_POINTS)
gx_data = deque(maxlen=MAX_POINTS)
gy_data = deque(maxlen=MAX_POINTS)
gz_data = deque(maxlen=MAX_POINTS)

is_reading = True

REGEX_PATTERN = re.compile(r"accel\(([^)]+)\),\s*gyro\(([^)]+)\)")

# ==========================================
# PARSING & READING FUNCTIONS
# ==========================================
def parse_line(line):
    match = REGEX_PATTERN.search(line)
    if match:
        try:
            accel_str, gyro_str = match.groups()
            ax, ay, az = map(float, accel_str.split(','))
            gx, gy, gz = map(float, gyro_str.split(','))
            return ax, ay, az, gx, gy, gz
        except ValueError:
            pass
    return None

def serial_reader():
    global is_reading
    start_time = time.time()
    
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
            while is_reading:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    
                    parsed = parse_line(line)
                    if parsed:
                        ax, ay, az, gx, gy, gz = parsed
                        current_time = time.time() - start_time
                        
                        t_data.append(current_time)
                        ax_data.append(ax)
                        ay_data.append(ay)
                        az_data.append(az)
                        gx_data.append(gx)
                        gy_data.append(gy)
                        gz_data.append(gz)
                
                except Exception as e:
                    print(f"Read error: {e}")
                    
    except serial.SerialException as e:
        print(f"Failed to open {SERIAL_PORT}: {e}")
        is_reading = False

# ==========================================
# PLOTTING FUNCTIONS
# ==========================================
def update_plot(frame):
    if not t_data:
        return line_ax, line_ay, line_az, line_gx, line_gy, line_gz

    t_list = list(t_data)
    
    # Update lines
    line_ax.set_data(t_list, list(ax_data))
    line_ay.set_data(t_list, list(ay_data))
    line_az.set_data(t_list, list(az_data))
    
    line_gx.set_data(t_list, list(gx_data))
    line_gy.set_data(t_list, list(gy_data))
    line_gz.set_data(t_list, list(gz_data))

    current_t = t_list[-1]
    
    # Slide the X-axis to keep current_t perfectly centered
    ax1.set_xlim(current_t - HALF_WINDOW, current_t + HALF_WINDOW)
    ax2.set_xlim(current_t - HALF_WINDOW, current_t + HALF_WINDOW)

    # Dynamically rescale Y-axis to fit newly drawn data
    ax1.relim()
    ax1.autoscale_view(scalex=False, scaley=True)
    ax2.relim()
    ax2.autoscale_view(scalex=False, scaley=True)

    return line_ax, line_ay, line_az, line_gx, line_gy, line_gz

# ==========================================
# MAIN EXECUTION
# ==========================================
if __name__ == "__main__":
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.tight_layout(pad=4.0)

    # Accelerometer Plot Setup
    ax1.set_title("Real-Time Accelerometer Data")
    ax1.set_ylabel("Acceleration (m/s²)")
    ax1.grid(True, linestyle='--', alpha=0.6)
    line_ax, = ax1.plot([], [], label='X', color='#d62728', lw=1)
    line_ay, = ax1.plot([], [], label='Y', color='#2ca02c', lw=1)
    line_az, = ax1.plot([], [], label='Z', color='#1f77b4', lw=1)
    ax1.legend(loc='upper right')

    # Gyroscope Plot Setup
    ax2.set_title("Real-Time Gyroscope Data")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Angular Velocity (°/s)")
    ax2.grid(True, linestyle='--', alpha=0.6)
    line_gx, = ax2.plot([], [], label='X', color='#d62728', lw=1)
    line_gy, = ax2.plot([], [], label='Y', color='#2ca02c', lw=1)
    line_gz, = ax2.plot([], [], label='Z', color='#1f77b4', lw=1)
    ax2.legend(loc='upper right')

    # Start Background Serial Thread
    thread = threading.Thread(target=serial_reader, daemon=True)
    thread.start()

    # Start Animation
    ani = animation.FuncAnimation(
        fig, 
        update_plot, 
        interval=50, 
        blit=False, 
        cache_frame_data=False
    )

    plt.show()

    print("Stopping serial reader...")
    is_reading = False
    thread.join(timeout=1.0)
    print("Program terminated safely.")
