from plotter import plot
from reader import read_fractures, read_points, read_traces


def __main__():
    fractures_path = "../Debug/debug_fractures.txt"
    debug_fractures_path = "../Debug/debug.txt"
    debug_traces_path = "../Debug/debug_traces.txt"
    debug_points_path = "../Debug/debug_points.txt"
    # fractures = read_fractures(fractures_path)
    fractures = read_fractures(debug_fractures_path)
    traces = read_traces(debug_traces_path, fractures)
    points = read_points(debug_points_path)
    # points = [[0.21174, 0.86746, 0.381013]]
    print(fractures, traces, points, [0])
    plot(fractures, points)


__main__()
