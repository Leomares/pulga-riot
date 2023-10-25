# choose the right port
# works on windows

import serial
import time
import matplotlib.pyplot as plt
import sqlite3
import atexit


def exitHandler():
    connection = sqlite3.connect('heater.db')
    cursor = connection.cursor()
    cursor.execute('')

    create_table_query = '''
        CREATE TABLE sensordata (
            id INTEGER PRIMARY KEY,
            x REAL,
            y REAL,
            z REAL
        )
    '''
    cursor.execute(create_table_query)
    connection.commit()
    insert_query = 'INSERT INTO sensordata VALUES(?,?,?,?)'
    for i in range(len(x)):
        cursor.execute(insert_query, (i, x[i], y[i], z[i]))
    connection.commit()
    connection.close()


atexit.register(exitHandler)

x, y, z = [], [], []
length = 0
window = 3200
count = 0

#pot = [0 for i in range(3)]

fig, axs = plt.subplots(3, 1, figsize=(12, 6))
legends = ["x", "y", "z"]
ylabel = [a + '"(g)' for a in legends[:-1]]

port = serial.Serial("COM3", 115200) # windows
#port = serial.Serial("/dev/ttyUSB0", 115200) # linux

while 1:
    line = port.readline()
    try:
        if line.split()[0] == b"time:":
            print(line.split()[1])
            continue
        else:
            values = [float(n) for n in line.split()]
            # print(values)
            x.append(values[0])
            #pot[0] += values[0] ** 2
            y.append(values[1])
            #pot[1] += values[1] ** 2
            z.append(values[2])
            #pot[2] += values[2] ** 2
            count += 1
            length += 1
            #print([p / length for p in pot])
    except ValueError as err:
        print(err)
    finally:
        if count > 1599:
            count = 0
            for ax in axs:
                ax.clear()
            axs[0].set_title("BMI 1600 Hz - Pulga")
            axs[-1].set_xlabel("Timestamp")
            axs[0].plot(x[-window:], label="x(g)", color="red")
            axs[1].plot(y[-window:], label="y(g)", color="blue")
            axs[2].plot(z[-window:], label="z(g)", color="green")
            axs[0].set_ylim([-1.5, 1.5])
            axs[1].set_ylim([-1.5, 1.5])
            axs[2].set_ylim([-1.5, 1.5])
            axs[3].set_ylim([20, 30])
            for ax in axs:
                axs[i].set_ylabel(ylabel[i])
                ax.legend(loc="upper right")
            plt.pause(0.1)
            plt.show(block=False)
