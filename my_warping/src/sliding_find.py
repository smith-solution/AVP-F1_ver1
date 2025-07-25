#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, random, math, copy

WIDTH = 640
HEIGHT = 480
WARP_W = 640
WARP_H = 480
NUM_WIN = 10
MARGIN = 50
MINPIX = 5

warp_src  = np.array([
    [180, 300],  
    [ 20, 400],
    [430, 300],
    [640, 400]
], dtype=np.float32)

warp_dist = np.array([
    [0,0],
    [0,WARP_H],
    [WARP_W,0],
    [WARP_W, WARP_H]
], dtype=np.float32)

calibrated = True
if calibrated:
    mtx = np.array([
        [422.037858, 0.0, 245.895397], 
        [0.0, 435.589734, 163.625535], 
        [0.0, 0.0, 1.0]
    ])
    dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
    cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (WIDTH, HEIGHT), 1, (WIDTH, HEIGHT))

def calibrate_image(frame):
    global WIDTH, HEIGHT
    global mtx, dist
    global cal_mtx, cal_roi
    
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]

    return cv2.resize(tf_image, (WIDTH, HEIGHT))

def warp_image(img, src, dst, size):
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)

    return warp_img, M, Minv

def warp_process_image(img):
    global NUM_WIN
    global MARGIN
    global MINPIX

    blur = cv2.GaussianBlur(img,(5,5), 0)
    H, L, S = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
    img = cv2.inRange(L, 150, 255)

    # cv2.imshow('HSL', img)
    
    # Get hitogram with bottom 20% of img
    histogram = np.sum(img[img.shape[0]//5:,:], axis=0)   
   
    # print(histogram)

    midpoint = int(histogram.shape[0]/2)
    leftx_current = np.argmax(histogram[:midpoint])
    rightx_current = np.argmax(histogram[midpoint:]) + midpoint

    # print(leftx_current, rightx_current)

    window_height = int(img.shape[0]/NUM_WIN)
    nz = img.nonzero()

    left_lane_inds = []
    right_lane_inds = []
    lx, ly, rx, ry = [], [], [], []
    
    out_img = np.zeros([HEIGHT,WIDTH, 3],dtype=np.uint8)
    out_img.fill(0)
    
    for window in range(NUM_WIN):

        win_yl = img.shape[0] - (window+1)*window_height
        win_yh = img.shape[0] - window*window_height

        win_xll = leftx_current - MARGIN
        win_xlh = leftx_current + MARGIN
        win_xrl = rightx_current - MARGIN
        win_xrh = rightx_current + MARGIN

        cv2.rectangle(out_img,(win_xll,win_yl),(win_xlh,win_yh),(0,255,0), 2) 
        cv2.rectangle(out_img,(win_xrl,win_yl),(win_xrh,win_yh),(0,255,0), 2) 

        good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0]
        good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > MINPIX:
            leftx_current = int(np.mean(nz[1][good_left_inds]))
        if len(good_right_inds) > MINPIX:        
            rightx_current = int(np.mean(nz[1][good_right_inds]))

        lx.append(leftx_current)
        ly.append((win_yl + win_yh)/2)

        rx.append(rightx_current)
        ry.append((win_yl + win_yh)/2)

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
 
    lfit = np.polyfit(np.array(ly),np.array(lx),2)
    rfit = np.polyfit(np.array(ry),np.array(rx),2)

    out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
    out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]] = [0, 0, 255]
    cv2.imshow("viewer", out_img)
    
    return lfit, rfit

def draw_lane(image, warp_img, Minv, left_fit, right_fit):
    global WIDTH, HEIGHT

    yMax = warp_img.shape[0]
    ploty = np.linspace(0, yMax-1, yMax)
    color_warp = np.zeros_like(warp_img).astype(np.uint8)
    
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))]) 
    pts = np.hstack((pts_left, pts_right))
    
    color_warp = cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    newwarp = cv2.warpPerspective(color_warp, Minv, (WIDTH, HEIGHT))

    return cv2.addWeighted(image, 1, newwarp, 0.3, 0)

def start():
    
    #cap = cv2.VideoCapture("xycar_track1.mp4")
    #cap = cv2.VideoCapture("road_video1.mp4")
    cap = cv2.VideoCapture("road_video2.mp4")

    while True:

        ret, image = cap.read()

        if ret:
            cal_image = calibrate_image(image)

            # cv2.imshow('calibration', cal_image)
            # cv2.waitKey(1)

            warp_img, M, Minv = warp_image(cal_image, warp_src, warp_dist, (WARP_W, WARP_H))

            cv2.imshow('warp image', warp_img)
            cv2.waitKey(1)

            left_fit, right_fit = warp_process_image(warp_img)
            lane_img = draw_lane(cal_image, warp_img, Minv, left_fit, right_fit)

            cv2.imshow('Sliding Window', lane_img)
            cv2.waitKey(1)


if __name__ == '__main__':
    start()

