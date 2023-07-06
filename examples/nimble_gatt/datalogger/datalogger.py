#choose the right port
#works on windows

import serial
import time
import matplotlib.pyplot as plt

x, y, z, t = [], [], [], []
length = 0
window = 500
count = 0

pot = [0 for i in range(3)]

fig, axs = plt.subplots(4, 1, figsize=(12, 6))
legends = ["x", "y", "z", "t"]
ylabel = [a + '"(g)' for a in legends] + ["temp(C)"]

port = serial.Serial("COM3", 115200)

while 1:
    line = port.readline()
    try:
        if line.split()[0] == b"time:":
            # print(line.split()[1])
            continue
        else:
            values = [float(n) for n in line.split()]
            # print(values)
            x.append(values[0])
            pot[0] += values[0] ** 2
            y.append(values[1])
            pot[1] += values[1] ** 2
            z.append(values[2])
            pot[2] += values[2] ** 2
            t.append(values[3])
            count += 1
            length += 1
            print([p / length for p in pot])
    except ValueError as err:
        print(err)
    finally:
        if count > 100:
            count = 0
            for ax in axs:
                ax.clear()
            axs[0].set_title("Pulga")
            axs[-1].set_xlabel("Timestamp")
            axs[0].plot(x, label="x", color="red")
            axs[1].plot(y, label="y", color="blue")
            axs[2].plot(z, label="z", color="green")
            axs[3].plot(t, label="t", color="pink")
            for ax in axs[:-1]:
                right = max(length, window)
                ax.set_xlim(right - window, right)
                ax.legend(loc="upper right")
            plt.pause(0.1)
            plt.show(block=False)
            plt.savefig(f"data{length/100}.png")
            # time.sleep(0.1)
