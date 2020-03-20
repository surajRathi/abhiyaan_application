#! /usr/bin/env python
import numpy as np


# TODO: astype taking long time.

# TODO: Implement a standard peaks algorithm instead of this slow stuff.
class Group:
    def __init__(self, point, epsilon, distance_func=None):
        self.epsilon = epsilon
        if distance_func:
            self.distance = distance_func

        self.points = [point]
        self.xrange = np.ones(2) * point[0]
        self.yrange = np.ones(2) * point[1]

    @staticmethod
    def distance(P1, P2, /):
        """Manhattan distance."""
        return abs(P1[0] - P2[0]) + abs(P1[1] - P2[1])

    def in_rectangle(self, point):
        return (self.xrange[0] - self.epsilon) <= point[0] <= (self.xrange[1] + self.epsilon) and (
                self.yrange[0] - self.epsilon) <= point[1] <= (self.yrange[1] + self.epsilon)

    def in_actual_range(self, point):  # Expensive, only run if
        """Calculates actual distance. (2-norm)"""
        for p in self.points:
            if self.distance(point, p) <= self.epsilon:
                return True
        else:
            return False

    def insert(self, point):
        """Updates bounds and adds point."""
        if point[0] > self.xrange[1]:
            self.xrange[1] = point[0]
        elif point[0] < self.xrange[0]:
            self.xrange[0] = point[0]

        if point[1] > self.yrange[1]:
            self.yrange[1] = point[1]
        elif point[1] < self.yrange[0]:
            self.yrange[0] = point[1]

        self.points.append(point)

    def centroid(self, as_int=True):
        x_sum = 0
        y_sum = 0
        for x, y in self.points:  # TODO Weight with hist_space?
            x_sum += x
            y_sum += y

        return (x_sum // len(self.points), y_sum // len(self.points)) if as_int else (
            x_sum / len(self.points), y_sum / len(self.points))

    def do(self, point):  # TODO Rename
        """If point belongs in group, inserts it and returns True. Otherwise returns False."""
        return self.in_rectangle(point) and self.in_actual_range(point) and (self.insert(point) or True)

    def __iter__(self):
        return self.points

    """
    def __str__(self):
        return str(self.points)
    """


class Lines:
    # increase frac to make more sensitive to smaller lines
    # decrease max_dist to make it more sensitive to closer lines
    def __init__(self, frame_shape, edge_threshold=500, no_theta=1000, no_rho=1000, frac=0.6, max_dist=30):
        self.frame_shape = frame_shape
        self.line_x = np.arange(0, self.frame_shape[0]).astype(int)
        self.line_y = np.arange(0, self.frame_shape[1]).astype(int)

        self.edge_threshold = edge_threshold
        self.peaks_max_dist = max_dist

        self.no_theta = no_theta
        self.no_rho = no_rho
        # self.rho_indice_bins = np.arange(self.no_rho)

        self.thetas = np.linspace(-np.pi / 2, np.pi / 2, no_theta)
        self.max_rho = np.sqrt(np.sum(np.square(self.frame_shape)))
        self.rhos = np.linspace(-self.max_rho, self.max_rho, no_rho)
        self.frac = frac

        self.hist_space = np.zeros((self.no_theta, self.no_rho))

        self.multiplier = no_rho * np.array([np.cos(self.thetas), np.sin(self.thetas)])

    def _edge_points(self, frame):
        return np.where(np.sum(np.square(np.gradient(np.sum(frame, axis=2) / 3)), axis=0) >= self.edge_threshold)

    def _peaks(self, locations):
        groups = []
        for x, y in zip(*locations):
            for group in groups:
                if group.do((x, y)):
                    break
            else:
                groups.append(Group((x, y), self.peaks_max_dist))

        return tuple(zip(*[g.centroid(as_int=True) for g in groups]))

    def _hough_transform(self, points):
        # TODO: Fix remove astype(int)
        points = np.array(points)

        # In [14]: %timeit np.add.at(a,np.digitize(x,bins), 1)
        # 10.9 ms ± 48.9 µs per loop (mean ± std. dev. of 7 runs, 100 loops each)
        #
        # In [15]: %timeit np.bincount(x.astype(int))
        # 149 µs ± 810 ns per loop (mean ± std. dev. of 7 runs, 10000 loops each)
        #
        # ??????? Doenst seem to happen, bad array size/ distribution

        rho_calc = (self.no_rho / 2 + (points.T @ self.multiplier) / (2 * self.max_rho)).astype('int')
        # rho_calc = (self.no_rho / 2 + (points.T @ self.multiplier) / (2 * self.max_rho))

        self.hist_space[...] = 0

        for i in range(self.no_theta):
            # np.add.at(self.hist_space[i, :], np.digitize(rho_calc[:, i], self.rho_indice_bins), 1) <- Slow.
            bins = np.bincount(rho_calc[:, i])
            self.hist_space[i, :len(bins)] = bins

        locations = np.where(self.hist_space >= (np.max(self.hist_space) * self.frac))

        theta_indices, rho_indices = self._peaks(locations)
        return self.thetas[list(theta_indices)], self.rhos[list(rho_indices)]

    def _line(self, theta, r):
        """Get line in cartesian form"""
        if np.abs(theta) < np.pi / 2:
            return np.clip((r / np.cos(theta) - self.line_y * np.tan(theta)).astype(int), 0, self.frame_shape[0] - 1) \
                , self.line_y, 'y_wise'
        return self.line_x, \
               np.clip((r / np.sin(theta) - self.line_x / np.tan(theta)).astype(int), 0,
                       self.frame_shape[1] - 1), 'x_wise'

    def transform(self, frame):
        """Detect and write lines to a frame."""
        lines = self._hough_transform(self._edge_points(frame))

        for line in zip(*lines):
            x, y, a = self._line(*line)
            frame[x, y, :] = 255

        return frame
