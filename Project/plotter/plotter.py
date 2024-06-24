from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import string


def plot(cells2D, traces):
    # Close any existing figures
    plt.close("all")

    # Create a new figure
    fig = plt.figure()

    ax = fig.add_subplot(111, projection="3d")

    # Dictionary to track plotted vertices and their labels
    plotted_vertices = {}

    # Define a list of colors to cycle through
    colors = ["r", "g", "k"]

    # Iterate through each cell in cell2d, using enumerate to get the index

    for [i, cell] in cells2D.items():

        # Separate the vertices into x, y, and z components for each cell
        x = [vertex[0] for vertex in cell]
        y = [vertex[1] for vertex in cell]
        z = [vertex[2] for vertex in cell]

        # Create a list of tuples for the Poly3DCollection for each cell
        vertices = [list(zip(x, y, z))]
        # Use the color corresponding to the current index, cycling through the colors list
        color = colors[i % len(colors)]
        poly = Poly3DCollection(vertices, alpha=0.5, facecolors=[color])
        ax.add_collection3d(poly)

        # Optionally, print the cell's vertices
        # print(f"Cell vertices: {vertices}")
    for trace in traces:
        trace_x = [point[0] for point in trace.extremes]
        trace_y = [point[1] for point in trace.extremes]
        trace_z = [point[2] for point in trace.extremes]
        ax.plot(trace_x, trace_y, trace_z, color="g")

        # Label each trace with "T" and the trace ID
        trace_mid_x = (trace_x[0] + trace_x[1]) / 2
        trace_mid_y = (trace_y[0] + trace_y[1]) / 2
        trace_mid_z = (trace_z[0] + trace_z[1]) / 2
        trace_label = f"T{trace.idTrace}"
        ax.text(
            trace_mid_x,
            trace_mid_y,
            trace_mid_z,
            trace_label,
            color="green",
            fontsize=8,
            ha="center",
        )

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # Show the plot
    plt.show()

    # Close the figure
    plt.close(fig)
