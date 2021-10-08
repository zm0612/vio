import argparse
import unittest
import numpy as np
import matplotlib.pyplot as plt


def read_mu_file(file_path):
    data = []
    with open(file_path) as file:
        for line in file.readlines():
            data.append(line)

    return data


def get_arguments():
    parser = argparse.ArgumentParser(description='Draw LM\'s mu', )
    parser.add_argument('-f', '--file_path', required=True, help='file path')

    return parser.parse_args()


def draw_mu(data):
    assert data is not None
    x = np.arange(0, len(data)).astype(dtype=np.str)
    plt.plot(x, data, 'r-')
    plt.show()


if __name__ == '__main__':
    arguments = get_arguments()
    data = read_mu_file(arguments.file_path)
    data = np.asarray(data, dtype=float)
    draw_mu(data)
