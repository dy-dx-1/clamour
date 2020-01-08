"""
Finds and plots the longest walk among the provided files.
Also finds the max number of recorded neighbors.
Must be executed in a directory that contains csv files for Chambord walks.
"""


import csv
import matplotlib.pyplot as plt

from glob import glob
from mpl_toolkits.mplot3d import Axes3D


def find_longest_trajectory() -> str:
    max_nb_lines, longest_trajectory = 0, ""
    max_neighbors, max_neighbor_file = 0, ""

    for file_name in glob("**", recursive=True):
        if ".csv" in file_name:
            file_name, max_nb_lines, max_neighbors, max_neighbor_file, longest_trajectory = parse_file(file_name, max_nb_lines, max_neighbors, max_neighbor_file, longest_trajectory)
            # print(file_name, max_nb_lines, max_neighbors, max_neighbor_file, longest_trajectory)
    
    print(f"Longest_trajectory: {longest_trajectory} with {max_nb_lines}")
    print(f"Max number of times neighbors were registered in a file ({max_neighbor_file}): {max_neighbors}")

    return longest_trajectory


def parse_file(file_name: str, max_nb_lines: int, max_neighbors: int, max_neighbor_file: str, longest_trajectory: str) -> tuple:
    with open(file_name) as data:
        nb_lines = len(data.readlines())
        if nb_lines > max_nb_lines:
            max_nb_lines, longest_trajectory = nb_lines, file_name

    # Two readings are necessary because of iterator
    with open(file_name) as data:
        reader = csv.reader(data, delimiter=",", quotechar="'")
        nb_neighbors = count_neighbors(list(reader)[1:])

        if nb_neighbors > max_neighbors:
            max_neighbors, max_neighbor_file = nb_neighbors, file_name

    return file_name, max_nb_lines, max_neighbors, max_neighbor_file, longest_trajectory
    

def count_neighbors(rows: list) -> int:
    return sum(1 for row in rows if row[-1] != "{}")


def extract_points(rows: list) -> tuple:
    x, y, t = [], [], []
    for row in rows:
        x.append(int(float(row[4])))
        y.append(int(float(row[6])))
        t.append(float(row[1]))

    return x, y, t


def display(x: list, y: list, t: list):
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    ax.scatter(x, y, t, s=10, c='r', marker="o")

    ax.set_xlabel('X coordinate')
    ax.set_ylabel('Y coordinate')
    ax.set_zlabel('Time')
    
    ax.set_xlim((-20000, 20000))
    ax.set_ylim((-20000, 20000))


    plt.grid()
    plt.show()



def plot_longest_trajectory(file_name: str) -> None:
    with open(file_name) as trajectory:
        reader = csv.reader(trajectory, delimiter=",", quotechar="'")
        reader.__next__()  # Skip header row

        rows = list(reader)
        display(*extract_points(rows))

        print(f"Number of neighbors of longest trajectory: {count_neighbors(list(reader))}")


def main() -> None:
    longest_trajectory = find_longest_trajectory()
    plot_longest_trajectory(longest_trajectory)


if __name__ == "__main__":
    main()
