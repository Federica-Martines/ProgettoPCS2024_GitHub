def read_points(file_path):
    points = {}
    with open(file_path, "r") as file:
        next(file)
        next(file)
        next(file)  # Skip header
        for line in file:
            parts = line.strip().split("; ")
            id_cell, x, y, z = map(float, parts)
            points[(x, y, z)] = int(id_cell)
    return points


def find_unique_points(file1, file2):
    points1 = read_points(file1)
    points2 = read_points(file2)

    unique_in_file1 = set(points1.keys()) - set(points2.keys())
    unique_in_file2 = set(points2.keys()) - set(points1.keys())

    unique_ids_file1 = [points1[point] for point in unique_in_file1]
    unique_ids_file2 = [points2[point] for point in unique_in_file2]

    return unique_ids_file1, unique_ids_file2


filename1 = "../Debug/polygonalMeshes/mesh0/Cell0D.txt"
filename2 = "../Debug/polygonalMeshes/mesh1/Cell0D.txt"
unique_ids_file1, unique_ids_file2 = find_unique_points(filename1, filename2)

print("Unique IDs in file 1:", unique_ids_file1)
print("Unique IDs in file 2:", unique_ids_file2)
