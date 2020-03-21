#! /usr/bin/env python
# Traffic lights seem to have a pretty distinctive color.
# TODO: Remove many close by contours.
# TODO: Better color ranges.
# TODO: Hough Circle detection.

import os

import cv2 as cv
import numpy as np

FOLDER = 'data_set'
WINDOW = 'window'
img = None

keys = {
    'LEFT': 81,
    'RIGHT': 83,
    'Q': 113
}


def mouse_callback(event, x, y, flags, params):
    global img
    if event == 1:
        print(cv.cvtColor(np.resize(img[y, x, :], (1, 1, 3)), cv.COLOR_BGR2HSV)[0, 0, :])


orange_ranges = (
    ((0, 140, 100), (40, 255, 256)),
    ((160, 150, 150), (180, 210, 256))
)
green_range = (59, 100, 90), (95, 190, 256)


def process(image):
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    hsv[:int(hsv.shape[0] * 0.2), :, :] = 0
    hsv[int(hsv.shape[0] * 0.45):, :, :] = 0
    hsv[:, :int(hsv.shape[1] * 0.3), :] = 0
    hsv[:, int(hsv.shape[1] * 0.7):, :] = 0

    o1 = cv.inRange(hsv, *orange_ranges[0])
    o2 = cv.inRange(hsv, *orange_ranges[1])
    g = cv.inRange(hsv, *green_range)

    mask = o1 | o2 | g
    print(np.max(mask))

    mask = cv.medianBlur(mask, 3).astype(np.uint8)
    mask = cv.blur(mask, (3, 3)).astype(np.uint8)

    contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    print(len(contours))
    a = []
    for contour in contours:
        if 10 <= cv.contourArea(contour) <= 40:
            M = cv.moments(contour)
            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])
            r = int((cv.contourArea(contour) / np.pi) ** (1 / 2))
            a.append((x, y, r))

    if len(a):
        for x, y, r in a:
            cv.rectangle(image, (x - 4 * r, y - 6 * r), (x + 4 * r, y + 6 * r), (0, 0, 255), 1)

    # cv.drawContours(image, contours, -1, (0, 0, 255), 2)
    return image


def main():
    global img
    images = (iter(os.listdir(FOLDER)))
    cur = next(images)
    cv.namedWindow(WINDOW)

    cv.setMouseCallback(WINDOW, mouse_callback)

    key = 0
    while key != keys['Q']:
        img = cv.imread(f'./{FOLDER}/{cur}')
        processed = process(img)
        cv.imshow(WINDOW, processed)
        key = cv.waitKey(-1)

        if key == keys['RIGHT']:
            cur = next(images)

    cv.destroyWindow(WINDOW)


if __name__ == '__main__':
    main()
