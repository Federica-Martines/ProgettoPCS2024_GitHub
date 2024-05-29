from typing import List
import numpy as np


class Fracture:
    def __init__(self, fracture_id, num_vertices, vertices):
        self.fracture_id = fracture_id
        self.num_vertices = num_vertices
        self.vertices = vertices
        self.traces = []

    def add_trace(self, segment):
        self.traces.append(segment)

    def __repr__(self):
        return (
            f"Fracture(\n id={self.fracture_id}\n num_vertices={self.num_vertices}\n "
            f"vertices={self.vertices}\n tracesNumber={len(self.traces)})"
        )


class Trace:
    def __init__(
        self,
        idTrace: int,
        idGenerator1: int,
        idGenerator2: int,
        extremes: List[List[float]],
        length: float,
        tips: bool,
    ):
        self.idTrace = idTrace
        self.idGenerator1 = idGenerator1
        self.idGenerator2 = idGenerator2
        self.extremes = [np.array(extreme) for extreme in extremes]
        self.length = length
        self.tips = tips
