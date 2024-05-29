from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import string


def plot(fractures, points, selected_fracture_ids=None):
    # Close any existing figures
    plt.close("all")

    # Create a new figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Dictionary to track plotted vertices and their labels
    plotted_vertices = {}

    # Plot fractures
    for fracture in fractures:
        # Check if selected_fracture_ids is provided and if the fracture ID is in the list
        if (
            selected_fracture_ids is None
            or fracture.fracture_id in selected_fracture_ids
        ):
            # Separate the vertices into x, y, and z components
            x = [vertex[0] for vertex in fracture.vertices]
            y = [vertex[1] for vertex in fracture.vertices]
            z = [vertex[2] for vertex in fracture.vertices]

            # Create a list of tuples for the Poly3DCollection
            vertices = [list(zip(x, y, z))]

            # Generate a random color for the polygon
            color = np.random.rand(
                3,
            )

            # Plot the polygon with the random color
            poly = Poly3DCollection(vertices, alpha=0.5, facecolors=[color])
            ax.add_collection3d(poly)

            # Plot the vertices
            ax.scatter(x, y, z, c="r", marker="o")

            # Label each vertex with a letter and the fracture ID
            for i, (vx, vy, vz) in enumerate(zip(x, y, z)):
                vertex = (vx, vy, vz)
                letter = string.ascii_uppercase[i % 26]  # Cycle through A-Z
                label = f"{letter}{fracture.fracture_id}"

                # If the vertex is already plotted, update the label
                if vertex in plotted_vertices:
                    plotted_vertices[vertex].set_text(label)
                else:
                    # Add the label if it is not plotted yet
                    text_obj = ax.text(
                        vx, vy, vz, label, color="blue", fontsize=8, ha="right"
                    )
                    plotted_vertices[vertex] = text_obj

            # Label the fracture with its ID
            centroid_x = sum(x) / len(x)
            centroid_y = sum(y) / len(y)
            centroid_z = sum(z) / len(z)
            ax.text(
                centroid_x,
                centroid_y,
                centroid_z,
                f"ID {fracture.fracture_id}",
                color="black",
                fontsize=10,
                ha="center",
            )

            # Plot and label traces
            for trace in fracture.traces:
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
    for p in points:
        ax.scatter(p[0], p[1], p[2], c="b", marker="o")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # Show the plot
    plt.show()

    # Close the figure
    plt.close(fig)
