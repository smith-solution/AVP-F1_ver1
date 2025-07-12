#!/usr/bin/env python
# -*- coding: utf-8 -*- 1
#=============================================
import cv2
import numpy as np

Blue =  (255,0,0) # 파란색
Green = (0,255,0) # 녹색
Red =   (0,0,255) # 빨간색
Yellow = (0,255,255) # 노란색

#=============================================
# 신호등의 파란불을 체크해서 True/False 값을 반환하는 함수
#=============================================
def check_traffic_light(image):
    MIN_RADIUS, MAX_RADIUS = 5, 50
    
    # 원본이미지를 복제한 후에 특정영역(ROI Area)을 잘라내기
    cimg = image.copy()
    Center_X, Center_Y = 320, 100  # ROI 영역의 중심위치 좌표 
    XX, YY = 220, 80  # 위 중심 좌표에서 좌우로 XX, 상하로 YY만큼씩 벌려서 ROI 영역을 잘라냄   

    # ROI 영역에 녹색 사각형으로 테두리를 쳐서 표시함 
    cv2.rectangle(cimg, (Center_X-XX, Center_Y-YY), (Center_X+XX, Center_Y+YY), Green, 2)
	
    # 원본 이미지에서 ROI 영역만큼 잘라서 roi_img에 담음 
    roi_img = cimg[Center_Y-YY:Center_Y+YY, Center_X-XX:Center_X+XX]

    # roi_img 칼라 이미지를 회색 이미지로 바꾸고 노이즈 제거를 위해 블러링 처리를 함  
    img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(img, (5, 5), 0)

    # Hough Circle 함수를 이용해서 이미지에서 원을 (여러개) 찾음 
    circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1, 20,
                  param1=40, param2=20, 
                  minRadius=MIN_RADIUS, maxRadius=MAX_RADIUS)

    # 디버깅을 위해서 Canny 처리를 했을때의 모습을 화면에 표시함
    # 위 HoughCircles에서 param1, param2에 사용했던 값을 아래 canny에서 똑같이 적용해야 함. 순서 조심.
    canny = cv2.Canny(blur, 20, 40)  # 주의: param1, param2 순서가 바뀜
    cv2.imshow('Canny image used by HoughCircles', canny)
    cv2.waitKey(1)

    if circles is not None:
    
        # 정수값으로 바꾸고 발견된 원의 개수를 출력
        circles = np.round(circles[0, :]).astype("int")
        #print("Found",len(circles),"circles")
        
        # 중심의 Y좌표값 순서대로 소팅해서 따로 저장
        y_circles = sorted(circles, key=lambda circle: circle[1])
 
        # 중심의 X좌표값 순서대로 소팅해서 circles에 다시 저장
        circles = sorted(circles, key=lambda circle: circle[0])
         
        # 발견된 원들에 대해서 루프를 돌면서 하나씩 녹색으로 그리기 
        for i, (x, y, r) in enumerate(circles):
            cv2.circle(cimg, (x+Center_X-XX, y+Center_Y-YY), r, Green, 2)
 
    # 이미지에서 정확하게 3개의 원이 발견됐다면 신호등 찾는 작업을 진행  
    if (circles is not None) and (len(circles)==3):
         
        # 가장 밝은 원을 찾을 때 사용할 리스트 선언
        brightness_values = []       

        # 발견된 원들에 대해서 루프를 돌면서 하나씩 처리 
 	    # 원의 중심좌표, 반지름. 내부밝기 정보를 구해서 화면에 출력
        for i, (x,y,r) in enumerate(circles):
            roi = img[y-(r//2):y+(r//2),x-(r//2):x+(r//2)]
            # 밝기 값은 반올림해서 10의 자리수로 만들어 사용
            mean_value = round(np.mean(roi),-1)
            brightness_values.append(mean_value)
            #print(f"Circle {i} at ({x},{y}), radius={r}: brightness={mean_value}")
			                
            # 원의 밝기를 계산했던 사각형 영역을 빨간색으로 그리기 
            cv2.rectangle(cimg, ((x-(r//2))+Center_X-XX, (y-(r//2))+Center_Y-YY),
                ((x+(r//2))+Center_X-XX, (y+(r//2))+Center_Y-YY), Red, 2)

        # 가장 밝은 원의 인덱스 찾기
        brightest_idx = np.argmax(brightness_values)
        brightest_circle = circles[brightest_idx]
        x, y, r = brightest_circle

        print(f" --- Circle {brightest_idx} is the brightest.")
        cv2.circle(cimg,(x+Center_X-XX,y+Center_Y-YY),r,Yellow,2)
            
        # 신호등 찾기 결과가 표시된 이미지를 화면에 출력
        cv2.imshow('Circles Detected', cimg)
        cv2.waitKey(1)

        # 아래와 같은 에러처리 코드가 보강되면 좋음
        #  (1)제일 위와 제일 아래에 있는 2개 원의 Y좌표값 차이가 크면 안됨 
        #  (2)제일 왼쪽과 제일 오른쪽에 있는 2개 원의 X좌표값 차이가 크면 안됨  
        #  (3)원들이 좌우로 너무 붙어 있으면 안됨 
       

        print(f"Found 3 circles. Brightness: {brightness_values}")

        colors = ['Red', 'Yellow', 'Blue']
        return True, colors[brightest_idx]
      
    # 신호등 찾기 결과가 표시된 이미지를 화면에 출력
    cv2.imshow('Circles Detected', cimg)
    
    # 원본 이미지에서 원이 발견되지 않았다면 False 리턴   
    #print("Can't find Traffic Light...!")
    return False, 'Black'
    