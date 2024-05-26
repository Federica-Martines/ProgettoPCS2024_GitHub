from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def plot(fractures, selected_fracture_ids=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

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

            # Plot the polygon
            poly = Poly3DCollection(vertices, alpha=0.5)
            ax.add_collection3d(poly)

            # Optionally, plot the vertices
            ax.scatter(x, y, z, c="r", marker="o")

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

            # Plot traces
            for trace in fracture.traces:
                # Check if selected_trace_ids is provided and if the trace ID is in the list
                trace_x = [point[0] for point in trace.extremes]
                trace_y = [point[1] for point in trace.extremes]
                trace_z = [point[2] for point in trace.extremes]
                ax.plot(trace_x, trace_y, trace_z, color="g")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    plt.show()
