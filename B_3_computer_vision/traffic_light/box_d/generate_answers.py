#! /usr/bin/env python
import os

import cv2 as cv

FILE = './boxes.csv'
SKIPPED = './skipped.list'
FOLDER = './data_set'

WINDOW = 'Box Maker'

out_file = None
cur_img = None
img_iter = None

# FORMAT: IMG_NUM, x1, x2, y1, y2
x_0, y_0 = 0, 0


def mouse_callback(event, x, y, flags, params):
    global out_file, cur_img, img_iter, WINDOW, x_0, y_0
    if event == cv.EVENT_MBUTTONDOWN:
        x_0, y_0 = x, y
    if event == 4:
        out_file.write(f'{cur_img},{x_0},{x},{y_0},{y}\n')
        out_file.flush()

        cur_img = next(img_iter)
        print(cur_img)
        cv.imshow(WINDOW, cv.imread(f'{FOLDER}/{cur_img}'))


def main():
    global out_file, cur_img, img_iter, WINDOW

    out_file = open(FILE, 'a')
    skip_file = open(SKIPPED, 'a')
    out_file.write('\n')
    skip_file.write('\n')
    img_iter = iter(os.listdir(FOLDER))
    """ out_file.write('Hello World')
    out_file.write('LOLZ')"""
    out_file.flush()

    cv.namedWindow(WINDOW)
    cv.setMouseCallback(WINDOW, mouse_callback)
    cur_img = next(img_iter)
    print(cur_img)
    cv.imshow(WINDOW, cv.imread(f'{FOLDER}/{cur_img}'))

    while (x := cv.waitKey(-1)) != 113:
        if x == 33:
            skip_file.write(cur_img)
            skip_file.write('\n')
            skip_file.flush()
            print(f'{cur_img} is skipped')
            cur_img = next(img_iter)
            print(cur_img)
            cv.imshow(WINDOW, cv.imread(f'{FOLDER}/{cur_img}'))

    cv.destroyAllWindows()


if __name__ == '__main__':
    main()
