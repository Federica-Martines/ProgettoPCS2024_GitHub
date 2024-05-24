import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Function to parse the data
def parse_data(filename):
    with open(filename, "r") as file:
        data = file.read()

    # Use regular expressions to extract relevant information
    pattern = r"(\w+)\t@(.*?)\tGeometry::(\w+)\n\s+(.*?)(?=\n\s+\w+|\Z)"
    matches = re.findall(pattern, data, re.DOTALL)

    parsed_data = []
    for match in matches:
        name = match[0]
        address = match[1]
        typename = match[2]
        properties = match[3]

        # Extract number of vertices and vertex coordinates
        num_vertices_match = re.search(r"numVertices\t(\d+)", properties)
        num_vertices = int(num_vertices_match.group(1)) if num_vertices_match else None

        vertices = []
        if num_vertices:
            vertex_pattern = r"\[(\d+)\]\t(.*?)\n"
            vertex_matches = re.findall(vertex_pattern, properties)
            for vertex_match in vertex_matches:
                vertex_coords = tuple(map(float, vertex_match[1].split(",")))
                vertices.append(vertex_coords)

        parsed_data.append(
            {
                "name": name,
                "address": address,
                "type": typename,
                "num_vertices": num_vertices,
                "vertices": vertices,
            }
        )

    return parsed_data


# Function to plot the vertices
def plot_vertices(data):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    for entry in data:
        if entry["num_vertices"] and entry["vertices"]:
            vertices = entry["vertices"]
            x_values = [vertex[0] for vertex in vertices]
            y_values = [vertex[1] for vertex in vertices]
            z_values = [vertex[2] for vertex in vertices]
            ax.scatter(x_values, y_values, z_values, label=entry["name"])

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Vertices of Fractures")
    ax.legend()
    plt.show()


# Main function
def main():
    filename = "test_data.txt"
    parsed_data = parse_data(filename)
    plot_vertices(parsed_data)


if __name__ == "__main__":
    main()
