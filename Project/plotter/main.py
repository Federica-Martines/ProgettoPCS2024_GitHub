from plotter import plot
from reader import read_fractures, read_traces


def __main__():
    file_path = "../Debug/debug.txt"
    file2_path = "../Debug/traces.txt"
    fractures = read_fractures(file_path)
    traces = read_traces(file2_path, fractures)
    print(fractures)
    plot(fractures)


__main__()
