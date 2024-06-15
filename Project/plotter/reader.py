from typing import List
import numpy as np
from geometry import Fracture, Trace


def readMesh(cellIds):
    cells2D = {}

    for cellId in cellIds:
        mesh_path = f"../Debug/polygonalMeshes/mesh{cellId}/Cell0D.txt"

        meshCell0D = []

        with open(mesh_path, "r") as file:
            lines = file.readlines()

            # Skip the first line as it contains the number of fractures
            line_idx = 0

            while line_idx < len(lines):
                if not (
                    lines[line_idx].startswith("Number")
                    or lines[line_idx].startswith("Id")
                ):
                    # Read vertices
                    coordinates = lines[line_idx].split(";")
                    meshCell0D.append(
                        [
                            float(coordinates[1]),
                            float(coordinates[2]),
                            float(coordinates[3]),
                        ]
                    )
                    line_idx += 1
                else:
                    line_idx += 1

        cells2D[cellId] = meshCell0D

    return cells2D


def read_traces(filename: str, cell2DId) -> List[Trace]:
    traces = []
    with open(filename, "r") as file:
        lines = file.readlines()

        for line in lines[3:]:
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

            if fracture_id1 in cell2DId or fracture_id2 in cell2DId:
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
