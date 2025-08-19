import serial
import matplotlib.pyplot as plt
import numpy as np

SERIAL_PORT = 'COM3'  # Change this to your Arduino's port
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

plt.ion()
fig, axs = plt.subplots(3, 1, figsize=(10, 8))
labels = ['Accel (g)', 'Gyro (deg/s)', 'Mag (uT)']
lines = [axs[i].plot([], [])[0] for i in range(3)]
data = np.zeros((3, 3, 200))  # 3 sensors, 3 axes, 200 samples

for i, ax in enumerate(axs):
    ax.set_title(labels[i])
    ax.set_xlim(0, 200)
    ax.set_ylim(-20, 20)
    ax.legend(['X', 'Y', 'Z'])

while True:
    try:
        line = ser.readline().decode().strip()
        if not line:
            continue
        parts = line.split(',')
        if len(parts) != 9:
            continue
        vals = [float(x) for x in parts]
        for i in range(3):  # 0:accel, 1:gyro, 2:mag
            data[i] = np.roll(data[i], -1, axis=1)
            data[i][:, -1] = vals[i*3:(i+1)*3]
            for j in range(3):
                lines[i].set_data(np.arange(200), data[i][j])
            axs[i].relim()
            axs[i].autoscale_view()
        plt.pause(0.01)
    except KeyboardInterrupt:
        break
    except Exception as e:
        print('Error:', e)

ser.close()