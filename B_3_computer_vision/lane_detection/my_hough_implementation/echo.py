#! /usr/bin/env python

# Applies hough transform on a webcam or remote video feed.


import cv2
import numpy as np
import requests
from lines import Lines

url = r'http://192.168.43.1:8080/shot.jpg'


def main():
    cap = None

    def get():
        return cv2.imdecode(np.asarray(bytearray(requests.get(url, stream=True).raw.read()), dtype="uint8"),
                            cv2.IMREAD_COLOR)

    try:  # Try to access remote camera.
        frame = get()
    except requests.exceptions.ConnectionError:  # Fallback to local camera instead.
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

        def get():
            return cap.read()[1]

        frame = get()

    ht = Lines(frame.shape[:2])

    while True:
        frame = get()

        trans = ht.transform(frame)

        cv2.imshow('Hmmm', trans)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    if cap:
        cap.release()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
