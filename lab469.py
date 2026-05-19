import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys, serial
import csv
import time

xsize = 500
port = "COM6"
baud = 9600
record_seconds = 60
csv_filename = "sensor_readings.csv"

try:
    ser = serial.Serial(port, baud, timeout=1)
    ser.reset_input_buffer()
except Exception as e:
    print(f"Error opening serial port: {e}")
    sys.exit(1)

#csv handling
csv_file = open(csv_filename, "w", newline="")
writer = csv.writer(csv_file)
writer.writerow(["Time (s)", "Gyro deg", "Acc deg"])

start_time = time.time()
recording_done = False


def data_gen():
    t = data_gen.t

    while True:
        elapsed = time.time() - start_time

        #auto sotp after amount 5 / 30 
        if elapsed >= record_seconds:
            break

        line_raw = ser.readline().decode(errors="ignore").strip()

        if not line_raw:
            continue

        try:
            parts = line_raw.split()

            # format num num num\n\r
            if len(parts) >= 3:
                acc = float(parts[0])
                gyro = float(parts[1])
                comp = float(parts[2])
                
            else:
                continue

        except ValueError:
            continue

        t += 1
        data_gen.t = t

        #save to csv
        writer.writerow([round(elapsed, 4), gyro, acc, comp]) #3 original, 4 now? 
        csv_file.flush()

        yield elapsed, gyro, acc, comp

    finish_recording()


def run(data):
    t, gyro, acc, comp = data
    
    #k = 0.9
    #comp = 5
    

    if t > -1:

        xdata.append(t)
        gyrodata.append(gyro)
        accdata.append(acc)
        compdata.append(comp)

        if len(xdata) > xsize * 2:
            xdata.pop(0)
            gyrodata.pop(0)
            accdata.pop(0)

       
        #compdata = 
        gyro_line.set_data(xdata, gyrodata)
        acc_line.set_data(xdata, accdata)
        comp_line.set_data(xdata, compdata)

        status_text.set_text(
            f"TIME: {time.time() - start_time:.2f}s   |   Gyr: {gyro:.2f} deg   |   Acc: {acc:.2f} deg |   Comp: {comp:.2f} deg"
        )

        if t > xsize:
            ax.set_xlim(t - xsize, t)

    return gyro_line, acc_line, comp_line, status_text


def finish_recording():
    global recording_done

    if recording_done:
        return

    recording_done = True

    try:
        csv_file.close()
    except Exception:
        pass

    try:
        ser.close()
    except Exception:
        pass

    print(f"Finished. Data saved to {csv_filename}")

    try:
        plt.close(fig)
    except Exception:
        pass


def on_close_figure(event):
    finish_recording()
    sys.exit(0)


data_gen.t = -1
xdata = []
gyrodata = []
accdata = []
compdata = []

fig, ax = plt.subplots(figsize=(12, 6))
plt.subplots_adjust(top=0.85)

fig.canvas.mpl_connect("close_event", on_close_figure)

status_text = fig.text(
    0.5,
    0.92,
    "",
    transform=fig.transFigure,
    ha="center",
    fontsize=14,
    fontweight="bold",
    bbox=dict(facecolor="white", alpha=0.8, edgecolor="black"),
)

# Gyro line
gyro_line, = ax.plot([], [], lw=2, color="blue", label="Gyroscope")

# Acc line
acc_line, = ax.plot([], [], lw=2, color="red", label="Accelerometer")

#comb line
comp_line, = ax.plot([], [], lw=2, color="green", label="Complimentary")

ax.set_ylim(-100, 100)
ax.set_xlim(0, 60)

ax.set_title("Gyroscope and Accelerometer")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Angle (deg)")

ax.grid(True)
ax.legend()

ani = animation.FuncAnimation(
    fig,
    run,
    data_gen,
    blit=False,
    interval=100,
    cache_frame_data=False,
)

plt.show()
