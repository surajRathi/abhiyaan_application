#! /usr/bin/env python

# Could maybe use a blob_detection algorithm instead.
# But no need to overcomplicate a problem.


import cv2 as cv
import numpy as np

color_names = ['Yellow', 'Green', 'Red']  # YGR
yellow_range = np.array((28, 0, 0), dtype=np.uint8), np.array((32, 255, 255), dtype=np.uint8)
green_range = np.array((40, 0, 0), dtype=np.uint8), np.array((75, 255, 255), dtype=np.uint8)
red_ranges = (np.array((0, 20, 20), dtype=np.uint8), np.array((10, 255, 255), dtype=np.uint8)), \
             (np.array((165, 20, 20), dtype=np.uint8), np.array((179, 255, 255), dtype=np.uint8))


def get_dominant_color(img):
    """Gets dominaint color from an HSV colorspace."""
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask_sums = np.sum(cv.inRange(hsv, *yellow_range)), np.sum(cv.inRange(hsv, *green_range)), \
                np.sum(cv.addWeighted(cv.inRange(hsv, *red_ranges[0]), 1, cv.inRange(hsv, *red_ranges[1]), 1, 0))

    return color_names[np.argmax(mask_sums)]


def simple_get_dominant_color(img):
    img[:, :, 0] = np.mean(img[:, :, 1:], axis=2)  # Convert to YGR
    sums = np.sum(np.sum(img, axis=0), axis=0)  # Find the amount of Yellow, Blue, and Red in the image.
    print(sums / 1000)
    print(np.argmax(sums))
    return color_names[np.argmax(sums)]


def main():
    for pic in (cv.imread(f'{i}.png') for i in range(1, 4)):
        print(get_dominant_color(pic))


if __name__ == '__main__':
    main()
