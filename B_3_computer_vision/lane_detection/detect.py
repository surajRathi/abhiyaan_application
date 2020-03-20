#! /usr/bin/env python
import cv2
import matplotlib.pyplot as plt
import numpy as np

BASIC_COLOR_LIST = (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255), (255, 0, 255)

NAMES = ['./sample_images/' + x for x in ['long_road.jpg', 'roads.jpeg']]
NAME = NAMES[1]


def color_iter(colors=BASIC_COLOR_LIST):
    i = 0
    while True:
        yield colors[i]
        i += 1
        i %= len(colors)


def lines(name):
    img = cv2.imread(name, 1)

    mask = np.zeros(img.shape[:2])
    h, w = mask.shape
    cv2.fillConvexPoly(mask, np.array([(0, h), (0, int(0.8 * h)), (w // 2, int(0.6 * h)), (w, int(0.8 * h)), (w, h)],
                                      dtype=np.int32), 1)

    edge = cv2.Canny(img, 2150, 2400, True, 5)
    masked = (edge * mask).astype(np.uint8)

    detected_lines = cv2.HoughLinesP(masked, 5, np.pi / 90, 2, minLineLength=150, maxLineGap=60)

    """ # The below addition does reduce the number of lines but it doesnt give endpoints.
    points = lines.reshape(-1, 1, 2)
    l1 = cv2.HoughLinesPointSet(points.astype(np.int32), lines_max=2, threshold=1, min_rho=100,
                                max_rho=np.linalg.norm(img.shape),
                                rho_step=1, min_theta=(np.pi / 12), max_theta=(np.pi * 11 / 12),
                                theta_step=(np.pi / 180))
    """

    c = color_iter()
    if detected_lines is not None:
        for (x1, y1, x2, y2), in detected_lines:
            cv2.line(img, (x1, y1), (x2, y2), next(c), 10)
            print(x1, x2, y1, y2)

    fig, ax = plt.subplots(2)
    ax[0].imshow((edge * (0.2 + 0.8 * mask)).astype(np.int32))
    ax[1].imshow(img[:, :, ::-1])  # Coverts BRG to RGB.
    plt.show()


def main():
    lines(NAME)


if __name__ == '__main__':
    main()
