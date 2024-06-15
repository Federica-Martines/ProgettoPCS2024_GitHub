from plotter import plot
from reader import readMesh, read_points, read_traces


def __main__():
    cell2DId = [2]
    debug_fractures_path = "../Debug/debug.txt"
    debug_traces_path = "../Debug/debug_traces.txt"
    debug_points_path = "../Debug/debug_points.txt"
    # fractures = read_fractures(fractures_path)
    cells2D = readMesh(cell2DId)
    traces = read_traces(debug_traces_path, cell2DId)
    points = read_points(debug_points_path)
    # points = [[0.21174, 0.86746, 0.381013]]
    # print(cells2D, traces, points, [2])

    plot(cells2D, traces)


__main__()
