#!/usr/bin/env python

import numpy as np
import cv2

# Use a 5x5 elliptic kernel.
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

# TODO: Set the threshold for min area?
bbox_area_threshold = 200


def morph(frame, kernel):
    cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel, dst=frame)
    cv2.morphologyEx(frame, cv2.MORPH_CLOSE, kernel, dst=frame)


def max_bounding_box(binary_img):
    contours = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    if contours:
        max_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(max_contour) > bbox_area_threshold:
            return cv2.boundingRect(max_contour)


class ColorDetector:

    def __init__(self, hue=(0, 179), saturation=(0, 255), value=(0, 255)):
        '''Initialize a color detector for hsv images with certain hue, saturation, value ranges.'''
        if not ((0 <= hue[0] <= 179) and (0 <= hue[1] <= 179)):
            raise ValueError('Valid hsv range for hue is [0, 179]')
        if not ((0 <= saturation[0] <= 255) and (0 <= saturation[1] <= 255) and
                (0 <= value[0] <= 255) and (0 <= value[1] <= 255)):
            raise ValueError('Valid hsv range for saturation and value is [0, 255]')

        self.hsv_values = np.array([hue, saturation, value], dtype=np.uint8)

    def threshold(self, frame):
        '''Returns a binary image with pixels falling into the appropriate hsv range.'''
        return cv2.inRange(frame, self.hsv_values[:, 0], self.hsv_values[:, 1], dst=frame)

    def bounding_box(self, frame):
        '''Returns (x, y, width, height) for the largest bounding box of a region
        of the given color.'''
        mask = self.threshold(frame)
        morph(mask, kernel)
        self.mask = mask  # Cache the mask for debuggin.
        return max_bounding_box(mask)


def main(video_fname=None):
    '''Main function for testing purposes.'''
    if video_fname:
        cap = cv2.VideoCapture(video_fname)
        print 'Opening source: ' + video_fname
    else:
        cap = cv2.VideoCapture(0)

    cv2.namedWindow('Original', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('BBox', cv2.WINDOW_AUTOSIZE)
    cv2.startWindowThread()

    green = ((59, 85), (0, 255), (0, 255))
    detect = ColorDetector(*green)
    key = None
    print('Press q to exit.')
    try:
        while key != ord('q'):
            success, frame = cap.read()
            if not success:
                print 'Failed to read from capture'
                break

            # frame = cv2.resize(frame, (640, 480))
            cv2.imshow('Original', frame)

            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            bbox = detect.bounding_box(hsv_frame)
            if bbox:
                (x, y, w, h) = bbox
                cv2.rectangle(frame, (x, y), (x+w, y+h), color=(0, 255, 0))
            cv2.imshow('BBox', frame)

            key = cv2.waitKey(30) & 0xff
    finally:
        print 'Exiting'
        cap.release()
        cv2.waitKey(1)
        cv2.destroyAllWindows()
        cv2.waitKey(1)

if __name__ == '__main__':
    import sys
    source = sys.argv[1] if len(sys.argv) > 1 else 0
    main(source)
