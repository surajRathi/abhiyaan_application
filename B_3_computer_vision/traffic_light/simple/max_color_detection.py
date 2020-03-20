#! /usr/bin/env python
# Can also use a blob_detection algo


import cv2 as cv
import numpy as np

color_names = ['Yellow', 'Green', 'Red']


def do(img):
    img[:, :, 0] = np.mean(img[:, :, 1:], axis=2)  # Convert to YGR
    sums = np.sum(np.sum(img, axis=0), axis=0)
    # noinspection PyTypeChecker
    return color_names[np.argmax(sums)]


def main():
    pics = [cv.imread(f'{i}.png') for i in range(1, 4)]
    for pic in pics:
        print(do(pic))


if __name__ == '__main__':
    main()
