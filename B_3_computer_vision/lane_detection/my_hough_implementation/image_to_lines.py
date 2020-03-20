#! /usr/bin/env python
# Prompts for an image and uses hough transform to superimpose lines on it.
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from lines import Group

mpl.rcParams['toolbar'] = 'None'


# noinspection PyUnreachableCode
def get_filename():
    """Prompts for an image and returns the filename."""
    # return "../sample_images/roads.jpeg"
    # TODO Invoke the GTK picker.

    from tkinter import Tk
    from tkinter.filedialog import askopenfilename

    Tk().withdraw()  # No Root window.
    filename = askopenfilename(initialdir='~/Pictures', title='Pick an Image',
                               filetypes=[("Images", "*.jpeg *.jpg *.png")])
    del Tk
    del askopenfilename

    return filename


def open_image(filename):
    #  TODO: Check for existence of image
    import matplotlib.image as mpl_img
    return mpl_img.imread(filename, 'jpg')


def edge_points(img, threshold=500):
    return np.where(np.sum(np.square(np.gradient(np.sum(img, axis=2) / 3)), axis=0) >= threshold)


def peaks(locs, max_dist):
    groups = []
    epsilon = max_dist

    for x, y in zip(*locs):
        for group in groups:
            if group.do((x, y)):
                break
        else:
            groups.append(Group((x, y), epsilon))

    return tuple(zip(*[g.centroid(as_int=True) for g in groups]))  # zip thing transposes


def hough_transform(points, no_theta=1000, no_rho=1000, frac=0.6, peak_distance=25, data_shape=None):
    if not data_shape:
        data_shape = np.max(points, axis=1)

    thetas = np.linspace(-np.pi / 2, np.pi / 2, no_theta)

    max_rho = np.sqrt(np.sum(np.square(data_shape)))
    rhos = np.linspace(-max_rho, max_rho, no_rho)

    hist_space = np.zeros((no_theta, no_rho))
    multiplier = no_rho * np.array([np.cos(thetas), np.sin(thetas)])

    if not isinstance(points, np.ndarray):
        points = np.array(points)

    rho_calc = (no_rho / 2 + (points.T @ multiplier) / (2 * max_rho)).astype('int')

    for i in range(no_theta):
        bins = np.bincount(rho_calc[:, i])
        hist_space[i, :len(bins)] = bins

    bin_threshold = np.max(hist_space) * frac
    locs = np.where(hist_space >= bin_threshold)
    print(f'Consolidating {len(locs[0])} lines.')

    theta_indices, rho_indices = peaks(locs, peak_distance)
    return thetas[list(theta_indices)], rhos[list(rho_indices)]


def show(img, thetas, rhos):
    def line(theta, r):
        if np.abs(theta) < np.pi / 2:
            y = np.linspace(0, img.T.shape[1])
            return r / np.cos(theta) - y * np.tan(theta), y
        x = np.linspace(0, img.T.shape[0])
        return x, r / np.sin(theta) - x / np.tan(theta)

    # Because of imshow, whole plot is bloody transposed, thus we must swap x and y coords of line
    for lin in zip(thetas, rhos):
        plt.plot(*(line(*lin)[::-1]), color="white", alpha=0.5, linewidth=5)

    plt.imshow(img)  # q , origin='lower', alpha=1)

    plt.gca().xaxis.tick_top()
    plt.xlim(0, img.shape[1])
    plt.ylim(img.shape[0], 0)
    plt.show()


def main():
    path = get_filename()
    if not path:
        from sys import exit
        exit(1)
    print(f'Opening image from {path}.')

    img = open_image(path)
    print(f'Opened a {img.shape[0]}x{img.shape[1]} image.')

    points = edge_points(img)
    print(f'Detected {len(points[0])} edge points. '
          f'(Of {np.product(img.shape[:2])}, i.e. {100 * len(points[0]) / np.product(img.shape[:2]):.2f}%)')

    thetas, rs = hough_transform(points, frac=0.6, peak_distance=10, no_theta=600, no_rho=200, data_shape=img.shape)
    print(f'{len(thetas)} lines detected.')

    show(img, thetas, rs)


if __name__ == '__main__':
    main()
