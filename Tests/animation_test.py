import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Set up the figure, axis, and plot element
fig, ax = plt.subplots()
x = np.linspace(0, 2 * np.pi, 1000)
# line, = ax.plot(x, np.sin(x))

# Initialize the plot
def init():
    # line.set_ydata(np.sin(x))
    ax.plot(x, np.sin(x))
    # return line,

# Update the plot for each frame
def update(frame):
    # line.set_ydata(np.sin(x + frame/5.0))  # Shift sine wave over time
    ax.clear()
    ax.plot(np.sin(x + frame/5.0))
    # return line,

# Create the animation
# ani = animation.FuncAnimation(fig, update, frames=100, init_func=init, blit=True)
ani = animation.FuncAnimation(fig, update, frames=100)
plt.show()
