import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig, ax = plt.subplots()
data = np.zeros((32, 100))
X = np.arange(data.shape[-1])

# Generate line plots
lines = []
for i in range(len(data)):
    # Each plot each shifter upward
    line, = ax.plot(X, i+data[i], color=".75")
    lines.append(line)

# Set limits
ax.set_ylim(0, len(data))
ax.set_xlim(0, data.shape[-1]-1)

# Update function


def update(*args):
    # Shift data left
    data[:, :-1] = data[:, 1:]

    # Append new values
    data[:, -1] = np.arange(len(data))+np.random.uniform(0, 1, len(data))

    # Update data
    for i in range(len(data)):
        lines[i].set_ydata(data[i])


ani = animation.FuncAnimation(fig, update, interval=10)
plt.show()
