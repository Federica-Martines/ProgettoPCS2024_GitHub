from typing import List
import numpy as np
from geometry import Fracture, Trace


def read_fractures(file_path):
    fractures = []

    with open(file_path, "r") as file:
        lines = file.readlines()

        # Skip the first line as it contains the number of fractures
        line_idx = 0

        while line_idx < len(lines):
            if lines[line_idx].startswith("# FractureId; NumVertices"):
                # Read FractureId and NumVertices
                fracture_info = lines[line_idx + 1].strip().split(";")
                fracture_id = int(fracture_info[0].strip())
                num_vertices = int(fracture_info[1].strip())

                # Read vertices
                vertices = []
                line_idx += 3  # Skip to the line with x-coordinates

                for i in range(num_vertices):
                    vertex = [
                        float(coords) for coords in lines[line_idx].strip().split(";")
                    ]
                    line_idx += 1
                    vertices.append(vertex)

                # Create a Fracture object and add to the list
                fracture = Fracture(fracture_id, num_vertices, vertices)
                fractures.append(fracture)

            else:
                line_idx += 1

    return fractures


def read_traces(filename: str, fractures: List[Fracture]) -> List[Trace]:
    traces = []
    with open(filename, "r") as file:
        lines = file.readlines()

        for line in lines:
            trace_data = line.strip().split("; ")
            trace_id = int(trace_data[0])
            fracture_id1 = int(trace_data[1])
            fracture_id2 = int(trace_data[2])
            x1, y1, z1 = map(float, trace_data[3:6])
            x2, y2, z2 = map(float, trace_data[6:])
            extremes = [[x1, y1, z1], [x2, y2, z2]]
            length = np.linalg.norm(np.array(extremes[0]) - np.array(extremes[1]))
            tips = False  # Tips information is not provided in the file, set to False by default

            trace = Trace(trace_id, fracture_id1, fracture_id2, extremes, length, tips)

            for fracture in fractures:
                if (
                    fracture.fracture_id == fracture_id1
                    or fracture.fracture_id == fracture_id2
                ):
                    fracture.traces.append(trace)

            traces.append(trace)

    traces = sorted(traces, key=lambda t: t.length, reverse=True)
    # for t in traces:
    #     print(t.idTrace, t.length)
    return traces


def read_points(file_path):
    points = []

    with open(file_path, "r") as file:
        lines = file.readlines()

        # Skip the first line as it contains the number of fractures
        line_idx = 0

        while line_idx < len(lines):
            if lines[line_idx].startswith("# Point"):
                line_idx += 1

                point = [float(coords) for coords in lines[line_idx].strip().split(";")]

                points.append(point)

            else:
                line_idx += 1
    return points
