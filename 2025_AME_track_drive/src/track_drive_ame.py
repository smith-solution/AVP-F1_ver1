#!/usr/bin/env python
# -*- coding: utf-8 -*- 1
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
import numpy as np
import cv2, rospy, time, os, math
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
from cv_bridge import CvBridge

#=========================================
# 외부에 있는 아래 파일들을 사용하기 위한 import 코드 
# aprilartag.py, filter.py, hough.py, pid.py, 
# stopline.py, trafficlight.py, ultradrive.py 
#=========================================
import aprilartag
import filter
import hough
import pid
import stopline
import trafficlight
import ultradrive
#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
motor = None  # 모터 노드 변수
Fix_Speed = 12  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
prev_angle = 0
new_speed = Fix_Speed  # 모터 속도 초기값
bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지 
ultra_msg = None  # 초음파 데이터를 담을 변수
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
motor_msg = xycar_motor()  # 모터 토픽 메시지
WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
View_Center = WIDTH//2 + 10  # 화면의 중앙값 = 카메라 위치
ar_msg = {"ID":[],"DX":[],"DZ":[]}  # AR태그 토픽을 담을 변수

#=============================================
# 초음파 8개의 거리정보에 대해서 이동평균필터를 적용하기 위한 선언
#=============================================
avg_count = 25  # 이동평균값을 계산할 데이터 갯수 지정
ultra_data = [filter.MovingAverage(avg_count) for i in range(8)]

#=============================================
# 조향각에 대해서 이동평균필터를 적용하기 위한 선언
#=============================================
angle_avg_count = 10  # 이동평균값을 계산할 데이터 갯수 지정
angle_avg = filter.MovingAverage(angle_avg_count)

#=============================================
# 특정 게인으로 PID 제어기 인스턴스를 생성
#=============================================
pid = pid.PID(kp=0.8, ki=0.001, kd=0.4)

#=============================================
# 카메라 토픽을 수신하는 Subscriber 함수
#=============================================
def usbcam_callback(msg):
    global image
    image = bridge.imgmsg_to_cv2(msg, "bgr8")

#=============================================
# 초음파 토픽을 수신하는 Subscriber 함수
#=============================================
def ultra_callback(msg):
    global ultra_msg
    ultra_msg = msg.data

    # 수집한 데이터를 필터링 처리함.
    #ultra_msg = ultra_filtering(ultra_msg)

#=============================================
# 수집된 정보를 필터링 하는 함수
#=============================================
def ultra_filtering(ultra_msg):

    global ultra_data
    
    # 새로운 데이터를 리스트에 추가
    for i in range(8):
        ultra_data[i].add_sample(float(ultra_msg[i]))
        
    # 아래에서 하나의 필터기법을 선택해서 사용 (주석제거)
    
    # 중앙값(Median)을 이용  
    #ultra_list = [int(ultra_data[i].get_mmed()) for i in range(8)]
    
    # 평균값(Average)을 이용 
    #ultra_list = [int(ultra_data[i].get_mavg()) for i in range(8)]
    
    # 가중평균값(Weighted Average)을 이용 
    #ultra_list = [int(ultra_data[i].get_wmavg()) for i in range(8)]
        
    # 최소값(Min Value)을 이용 
    ultra_list = [int(ultra_data[i].get_min()) for i in range(8)]
    
    # 최대값(Max Value)을 이용 
    #ultra_list = [int(ultra_data[i].get_max()) for i in range(8)]
    
    return tuple(ultra_list)
    
#=============================================
# 모터제어 명령 토픽을 발행하는 Publisher 함수
#=============================================
def drive(angle, speed):
    motor_msg.angle = float(angle)
    motor_msg.speed = float(speed)
    motor.publish(motor_msg)
    
#=============================================
# 차량을 duration(1초 단위) 동안 정차시키는 함수
#=============================================
def stop_car(duration):
    for _ in range(int(duration*10)):
        drive(angle=0, speed=0)
        time.sleep(0.1)
        
#=============================================
# 차량을 정해진 시간동안 정해진 방향과 속도로 주행시키는 함수  
# duration(초) 시간동안 angle, speed 값을 모터로 보냄.
#=============================================
def move_car(duration, angle, speed):
    for i in range(int(duration*10)): 
        drive(angle, speed)
        time.sleep(0.1)
        
#=============================================
# 카메라의 Exposure 값을 변경하는 함수 
# 입력으로 0~255 값을 받을 수 있음.
# 32의 배수가 되는 순간 갑자기 어두워진다. 주의~!
#=============================================
def cam_exposure(value):
    command = 'v4l2-ctl -d /dev/videoCAM -c exposure_absolute=' + str(value)
    os.system(command)
    
#=============================================
# 실질적인 메인 함수 
#=============================================
def start():

    global motor, ultra_msg, image, angle_avg
    global new_angle, new_speed, prev_angle
    retry_count = 0
    View_Center = 320

    #=========================================
    # 노드를 생성하고, 구독/발행할 토픽들을 선언합니다.
    #=========================================
    rospy.init_node('Track_Driver')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/",Image,usbcam_callback, queue_size=1)
    
    #=========================================
    # 노드들로부터 첫번째 토픽들이 도착할 때까지 기다립니다.
    #=========================================
    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")
    rospy.wait_for_message("xycar_ultrasonic", Int32MultiArray)
    print("UltraSonic Ready ----------")
    
    print("======================================")
    print(" S T A R T    D R I V I N G ...")
    print("======================================")

    # 동작모드를 고유의 숫자값으로 구분하여 지정합니다. 
    # 각 동작모드마다 while 블럭이 하나씩 존재합니다. 
    STOP_LINE = 1
    TRAFFIC_LIGHT = 2
    SENSOR_DRIVE = 3
    AR_DRIVE = 4
    LANE_DRIVE = 5
    PARKING = 6
    FINISH = 7
    FAST = 8

    #===================================
    # 아래 while 블럭중에 
    # 처음에 어디로 들어갈지 결정합니다.
    # 각종 로컬변수의 값을 초기화 합니다.
    #===================================.
    stop_car(2.0)  # 차량을 멈춥니다. 
    #cv2.destroyAllWindows()  # OpenCV 창을 모두 닫습니다. 
    drive_mode = LANE_DRIVE  # 처음 진입할 모드를 선택합니다.
    cam_exposure(105) # 카메라의 노출값을 적절하게 변경합니다.
    print("----- Finding Stopline starts... -----")
    go_next = False  # 다음을 위해 원래대로 False로 바꿉니다.
    count = 0

    # lane change
    move_flag = 0

    #===================================
    # 메인 루프 - Main While Loop
    #===================================
    while not rospy.is_shutdown():
    
        # ======================================
        # 전방에서 정지선을 찾고 정차합니다.
        # 다음 순서인 TRAFFIC_LIGHT 모드로 넘어갑니다.  
        # ======================================
        while drive_mode == STOP_LINE:
                              
            # 잠깐씩 쉬면서 while 루프를 돕니다.
            # 개발단계에서의 편의를 위해 삽입된 코드입니다. 
            # 코드 개발이 끝난후 꼭 필요하지 않으면 삭제하세요.             
            # time.sleep(0.3)

            # 차량을 똑바로 앞으로 전진시킵니다. 
            new_angle = 0        
            new_speed = 7
            drive(new_angle, new_speed)
            
            # 정지선이 있는지 체크합니다.          
            flag = stopline.check_stopline(image)
            
            # 정지선을 찾았으면 다음 모드로 넘어갑니다.
            if (flag == True):
                print("#======================================#") 
                print("#  Found Stopline! -- Go Next!         #") 
                print("#======================================#") 
                time.sleep(0.5)
                go_next = True

            #else:
            #    print('.', end='', flush=True)

            # 아래 if 블럭에는 go_next가 True로 변경된 경우에만 들어갈 수 있습니다.
            if (go_next == True):                
                stop_car(2.0)  # 차량을 멈춥니다. 
                #cv2.destroyAllWindows()  # OpenCV 창을 모두 닫습니다. 
                # drive_mode = TRAFFIC_LIGHT  # 다음 모드로 넘어갑니다.
                drive_mode = TRAFFIC_LIGHT
                cam_exposure(100)  # 카메라의 노출값을 적절하게 변경합니다.
                print("----- Traffic Light Checking starts... -----")
                go_next = False  # 다음을 위해 원래대로 False로 바꿉니다.
                count = 0

        # ======================================
        # 전방에서 신호등을 찾고 파란색 불이 켜지면 출발합니다.
        # 곧바로 다음 순서로 넘어갑니다.  
        # ======================================
        while drive_mode == TRAFFIC_LIGHT:
                              
            # 잠깐씩 쉬면서 while 루프를 돕니다.
            # 개발단계에서의 편의를 위해 삽입된 코드입니다. 
            # 코드 개발이 끝난후 꼭 필요하지 않으면 삭제하세요.             
            # time.sleep(0.3)

            # 신호등이 있는지, 있다면 어떤 불이 켜졌는지 체크합니다.  
            flag, color = trafficlight.check_traffic_light(image)
            
            # 신호등을 찾았고, 파란불이 켜졌으면 다음 모드로 넘어갑니다.
            if (flag == True) and (color == 'Blue'):
                print("#======================================#") 
                print("#  Traffic light is Blue! -- Go Next!  #") 
                print("#======================================#") 
                go_next = True
            # time.sleep(3)
            # go_next = True
            
            # 아래 if 블럭에는 go_next가 True로 변경된 경우에만 들어갈 수 있습니다.
            if (go_next == True):                
                stop_car(2.0)  # 차량을 멈춥니다.
                #cv2.destroyAllWindows()  # OpenCV 창을 모두 닫습니다. 
                drive_mode = SENSOR_DRIVE  # 다음 모드로 넘어갑니다.
                cam_exposure(100)  # 카메라의 노출값을 적절하게 변경합니다.
                print ("----- Sensor Driving starts... -----")
                go_next = False  # 다음을 위해 원래대로 False로 바꿉니다.
                count = 0

        # ======================================
        # 거리센서를 이용하여 라바콘 사이를 주행합니다.
        # 주행 중에 전방에 AR태그가 있는지 계속 체크합니다.
        # AR태그가 가까이에 보이면 다음 순서로 넘어갑니다. 
        # ======================================
        while drive_mode == SENSOR_DRIVE:

            # 잠깐씩 쉬면서 while 루프를 돕니다.
            # 개발단계에서의 편의를 위해 삽입된 코드입니다. 
            # 코드 개발이 끝난후 꼭 필요하지 않으면 삭제하세요.             
            # time.sleep(0.1)
            
            # 라바콘 주행을 위한 핸들값을 찾아냅니다.
            new_angle = ultradrive.sonic_drive(ultra_msg, new_angle)  
            
            # 주행 속도값을 세팅합니다. 
            new_speed = 8

            # 위에서 결정된 핸들값과 속도값을 토픽에 담아 모터로 보냅니다.
            drive(new_angle, new_speed)

            # 전방에 AR태그가 있는지 체크합니다. 
            ar_ID, z_pos, x_pos = aprilartag.check_AR(image)            

            # AR태그가 안보이면, 발견하지 못했다는 메시지를 출력합니다.  
            if (ar_ID == 99):
                print(f"......Cannot find AR nearby... {count}") 
                count = count+1

            # AR태그가 발견되면 여기로 들어갑니다. 
            else:
                # Z값이 멀면 메시지 출력하고 라바콘 주행을 계속합니다. 
                if (z_pos > 80):
                    print(f"....AR found, but it's far away {count}")
                    count = count+1
                    
                # Z값이 가까우면 다음 모드로 넘어갑니다.
                else:
                    # 발견된 AR태그의 ID값과 Z거리값, X거리값을 출력합니다.  
                    print(f"ID={ar_ID} Z={z_pos} X={x_pos}")
                    print("#=================================#") 
                    print("#  Nearby AR found! -- Go Next!   #")
                    print("#=================================#") 
                    go_next = True
                  
            # 아래 if 블럭에는 go_next가 True로 변경된 경우에만 들어갈 수 있습니다.
            if (go_next == True):                
                stop_car(2.0)  # 차량을 멈춥니다.
                #cv2.destroyAllWindows()  # OpenCV 창을 모두 닫습니다. 
                drive_mode = AR_DRIVE  # 다음 모드로 넘어갑니다.
                cam_exposure(100)  # 카메라의 노출값을 적절하게 변경합니다.
                print ("----- AR Driving starts... -----")
                go_next = False  # 다음을 위해 원래대로 False로 바꿉니다.
                count = 0

        # ======================================
        # AR박스를 발견하면 따라가는 주행을 합니다.
        # AR박스가 retry_count 회수만큼 보이지 않으면
        # 다음 순서인 LANE_DRIVE 모드로 넘어갑니다.          
        # ======================================
        arcnt = 0
        artim = 0
        while drive_mode == AR_DRIVE:
        
            # 잠깐씩 쉬면서 while 루프를 돕니다.
            # 코드 개발이 끝난후 꼭 필요하지 않으면 삭제하세요.             
            # time.sleep(0.3)
                                                
            # 전방에 AR태그가 보이는지 체크합니다.
            ar_ID, z_pos, x_pos = aprilartag.check_AR(image)
            artim += 1
                           
            # AR태그 보이면 그쪽으로 주행합니다. 
            if (ar_ID != 99): # AR이 발견된 경우

                # 발견된 AR태그의 ID값과 Z거리값, X거리값을 출력합니다.  
                print(f"ID={ar_ID} Z={z_pos} X={x_pos}")

                # retry_count는 처음값인 '0'으로 세팅합니다.
                retry_count = 0
                
                # AR태그까지의 거리를 계산합니다. 
                distance = math.sqrt(z_pos**2 + x_pos**2)
                
                # 거리와 X값을 따져서 핸들값을 적절히 세팅합니다. 
                if (distance > 60): # far
                    x_pos = x_pos + 0
                    if x_pos < 0: # AR is left
                        new_angle = -30
                    else: # x_pos > 0: # AR is right
                        new_angle = 30
                elif (distance > 25):
                    if x_pos < 50: # AR is so so left
                        new_angle = -40
                    else: # x_pos > 50: # AR is too too right
                        new_angle = 50
                # else: # so close
                    if arcnt == 0:
                        print("STOPSTOPSTOPSTOPSTOPSTOPSTOP")
                        drive(0,0)
                        time.sleep(0.7)
                    # x_pos = x_pos + 30
                    # new_angle = -(x_pos * 1.7)
                    # new_angle = -80
                    arcnt += 1

                # 속도값을 적절하게 세팅합니다.
                new_speed = 8
                
                # 위에서 결정된 핸들값과 속도값을 토픽에 담아 모터로 보냅니다.
                drive(new_angle, new_speed)

            # AR태그 보이지 않으면 상황판단을 해서 대책을 강구합니다.            
            else:  # AR이 발견되지 않은 경우 
                retry_count = retry_count + 1
                if (retry_count == 15):
                    print("#============================#") 
                    print("#  No more AR! -- Go Next!   #")
                    print("#============================#") 
                    go_next = True                   
                else:
                    print(f"AR not found. Searching... {retry_count}")
                    # 조향각과 속도값을 토픽에 담아 모터로 보냅니다.
                    # new_angle = max(min(new_angle, 50), 50)
                    # new_angle = 0 if arcnt==0 else (-65 if retry_count < 8 else 80)
                    new_angle = 0 if arcnt==0 else -artim*1.5
                    new_speed = 8
                    drive(new_angle, new_speed)
                    time.sleep(0.3)
  
            # 아래 if 블럭에는 go_next가 True로 변경된 경우에만 들어갈 수 있습니다.
            if (go_next == True):                
                stop_car(2.0)  # 차량을 멈춥니다.
                #cv2.destroyAllWindows()  # OpenCV 창을 모두 닫습니다. 
                drive_mode = LANE_DRIVE  # 다음 모드로 넘어갑니다.
                cam_exposure(100)  # 카메라의 노출값을 적절하게 변경합니다.
                print("----- Lane driving starts... -----")
                go_next = False  # 다음을 위해 원래대로 False로 바꿉니다.
                count = 0

        # ======================================
        # 차선을 인식해서 차선을 따라 주행합니다. 
        # 중간에 장애물이 있으면 회피하여 주행합니다. 
        # 최대한 빠르게 주행합니다.
        # ======================================
        while drive_mode == LANE_DRIVE:

            # 잠깐씩 쉬면서 while 루프를 돕니다.
            # 코드 개발이 끝난후 꼭 필요하지 않으면 삭제하세요.             
            time.sleep(0.3)

            # lane change
            ar_ID, z_pos, x_pos = aprilartag.check_AR(image)
            if move_flag == 0 and ar_ID != 99:
                move_flag = 1
                for _ in range(7):
                    drive(-20,14)
                    time.sleep(0.1)
                for _ in range(6):
                    drive(20,10)
                    time.sleep(0.1)
        
            # 카메라 영상에서 차선의 위치를 알아냅니다.
            found, x_left, x_right = hough.lane_detect(image)

            # 차선인식이 됐으면 핸들값과 속도값을 결정합니다.
            if found:
                count = 0                
                #====================================================
                # 차선 중점과 차량의 위치차이로 핸들값을 계산합니다.
                #  * 위치차이를 핸들값으로 변환할때 가중치 조정이 필요합니다.
                #  * 코너주행에서는 계산법이 달라져야 합니다.
                #====================================================
                x_midpoint = (x_left + x_right) // 2  # 차선의 중점 구하기
                corner_shift = new_angle * (0.5)            
                new_angle = (x_midpoint - (View_Center+corner_shift)) * 1.0
                
                #=========================================
                # 핸들값에 PID제어를 적용합니다. 
                #=========================================
                new_angle = pid.pid_control(new_angle)        
                # 1. 차선 중심 오차 계산
                offset = x_midpoint - View_Center

                # 2. 곡선 여부 판단
                is_curve = abs(offset) > 30

                # 3. 직선에서는 오차가 작으면 무시
                if not is_curve and abs(offset) < 5:
                    offset = 0.0

                # 4. 변화량 필터 적용 (직선에서만)
                delta_offset = offset - prev_offset
                if not is_curve and abs(delta_offset) < 2:
                    offset = prev_offset
                prev_offset = offset

                # 5. PID 제어 및 이동 평균 적용
                new_angle = pid.pid_control(offset)
                angle_avg.add_sample(new_angle)
                new_angle = angle_avg.get_mavg()
                new_angle = max(-50, min(50, new_angle))  # 조향 제한

                # 6. PID 파라미터 및 속도 다단계 조정
                angle_abs = abs(new_angle)
                if angle_abs < 10:
                    pid.Kp = 0.3; pid.Ki = 0.001; pid.Kd = 0.4
                    target_speed = 20
                elif angle_abs < 20:
                    pid.Kp = 0.4; pid.Ki = 0.001; pid.Kd = 0.5
                    target_speed = 18
                elif angle_abs < 30:
                    pid.Kp = 0.6; pid.Ki = 0.001; pid.Kd = 0.5
                    target_speed = 15
                elif angle_abs < 45:
                    pid.Kp = 1.0; pid.Ki = 0.0; pid.Kd = 0.3
                    target_speed = 12
                else:
                    pid.Kp = 1.2; pid.Ki = 0.0; pid.Kd = 0.2
                    target_speed = 10

                # 7. 속도 보간 적용
                alpha = 0.3
                new_speed = alpha * target_speed + (1 - alpha) * new_speed

                # 8. 모터 제어
                drive(new_angle, new_speed)

                # ### round
                # if abs(new_angle) > 60 :
                #     pid.Kp = 1.2
                #     pid.Kd = 0.2
                # else : #straight
                #     pid.Kp = 0.35
                #     pid.Ki = 0.0001
                #     pid.Kd = 1.0
                    
                #====================================================
                # 속도값을 적절하게 세팅합니다. 
                #  * 직진구간에서는 코너보다 빠르게 주행하게 하면 좋습니다.
                #====================================================
                if (abs(new_angle) < 20):
                    new_speed = 50
                elif (abs(new_angle) < 40):
                    new_speed = 40
                else:  # 40 < new_angle < 100
                    new_speed = 20
                
                # 위에서 결정된 핸들값과 속도값을 토픽에 담아 모터로 보냅니다.
                #print(f"Angle={new_angle:.1f} Speed={new_speed:.1f}")
                # drive(new_angle if abs(new_angle) > 60 else new_angle*0.5, new_speed)
                
            # 차선인식이 안됐으면 가던대로 갑니다.
            else:         
                #print(f"Lane finding fails... Keep going! {count}")
                count = count+1
                            
                # count 값이 기준값을 넘지 않은 경우, 즉 잠깐동안 차선을 찾지 못한 경우에는 가던대로 갑니다.
                if (count < 10):
                    # 기존의 핸들값과 속도값을 그대로 다시 한번 토픽에 담아 모터로 보냅니다.
                    drive(new_angle, new_speed)
                    time.sleep(0.1)
                    
                # count 값이 기준값을 넘은 경우, 즉 계속해서 차선을 찾지 못한 경우에는 차량을 멈춥니다.
                else:
                    # 차량을 멈추기 위해 속도값을 0으로 바꾸어 토픽에 담아 모터로 보냅니다.
                    new_speed = 0
                    drive(new_angle, new_speed)
                    time.sleep(0.1)
        while drive_mode == FAST:
            print("DO IT FASTER")
            drive(0,100)
            time.sleep(1)
            drive(0,100)
            time.sleep(1)
            drive(0,100)
            time.sleep(0.5)
            drive(0,0)
            print("STOP")
            drive_mode = FINISH
                
#=============================================
# 메인함수를 호출합니다.
# start() 함수가 실질적인 메인함수입니다.
#=============================================
if __name__ == '__main__':
    start()
