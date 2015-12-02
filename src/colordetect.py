#!/usr/bin/env python

import numpy as np
import cv2

# Use a 5x5 elliptic kernel.
kernel = np.ones((7, 7), np.uint8)

# Set the threshold for min area?
bbox_area_threshold = 500

# When detected area at least 1/5 of total then say it's flag.
total_area = 640 * 480


def large_area(area):
    return area > 0.1 * total_area


def morph(frame, kernel):
    morphed = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)
    cv2.morphologyEx(frame, cv2.MORPH_CLOSE, kernel, dst=morphed)
    return morphed


def max_bounding_box(binary_img):
    contours = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    if contours:
        bboxes = [(cv2.contourArea(c), cv2.boundingRect(c)) for c in contours]
        bboxes = [(a, r) for (a, r) in bboxes if (r[2] < r[3]) or large_area(a)]
        if bboxes:
            area, bbox = max(bboxes)
            if area > bbox_area_threshold:
                return bbox


class ColorDetector:

    def __init__(self, hue=(0, 179), saturation=(0, 255), value=(0, 255)):
        '''Initialize a color detector for hsv images with certain hue, saturation, value ranges.'''
        if not ((0 <= hue[0] <= 179) and (0 <= hue[1])):
            raise ValueError('Valid hsv range for hue is [0, 179]')
        if not ((0 <= saturation[0] <= 255) and (0 <= saturation[1] <= 255) and
                (0 <= value[0] <= 255) and (0 <= value[1] <= 255)):
            raise ValueError('Valid hsv range for saturation and value is [0, 255]')

        self.hsv_values = np.array([hue, saturation, value], dtype=np.uint8)
        self.hsv_values2 = None
        if hue[1] > 179:
            self.hsv_values2 = np.array([(0, hue[1] % 179), saturation, value], dtype=np.uint8)

    def threshold(self, frame):
        '''Returns a binary image with pixels falling into the appropriate hsv range.'''
        mask = cv2.inRange(frame, self.hsv_values[:, 0], self.hsv_values[:, 1])
        if self.hsv_values2 is not None:
            mask2 = cv2.inRange(frame, self.hsv_values2[:, 0], self.hsv_values2[:, 1])
            cv2.bitwise_or(mask, mask2, dst=mask)
        return mask

    def bounding_box(self, frame):
        '''Returns (x, y, width, height) for the largest bounding box of a region
        of the given color.'''
        self.mask = self.threshold(frame)
        morphed = morph(self.mask, kernel)
        return max_bounding_box(morphed)


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
