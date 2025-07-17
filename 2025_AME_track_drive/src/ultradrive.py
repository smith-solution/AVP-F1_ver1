#!/usr/bin/env python
# -*- coding: utf-8 -*-
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================

#=============================================
# 거리센서를 이용해서 장애물까지의 거리를 알아내서
# 장애물과 충돌하지 않으며 주행하도록 조향값을 반환 
#=============================================

left_buf = 0
left_prev = 0
right_buf = 0
right_prev = 0

Kp = 0.6
Ki = 0.001
Kd = 0.3

perr = 0.0
ierr = 0.0
derr = 0.0

cte_prev = None
cte = 0

cnt = 0

def sonic_drive(ultra_msg, orig_angle):
    global left_buf
    global left_prev
    global right_buf
    global right_prev
    global Kp
    global Ki
    global Kd
    global perr
    global ierr
    global derr
    global cte_prev
    global cte
    global cnt
    imax = 10
    imin = -10
    dt = 1.0 / 30

    left = ultra_msg[0]
    lfront = ultra_msg[1]
    cfront = ultra_msg[2]
    rfront = ultra_msg[3]
    right = ultra_msg[4]

    angle_by_side = 0
    angle_by_front = 0
    
    # Update left & right distance
    if left_prev == 0 and left > 0:
        left_buf = left
    if right_prev == 0 and right > 0:
        right_buf = right
    left_prev = left
    right_prev = right

    # Go to centre using PID control
    cte = right_buf - left_buf # right>left: must go to right(positive angle)

    if cte_prev is None:
        cte_prev = cte

    derr = (cte - cte_prev) / dt
    perr = cte
    ierr += cte * dt
    ierr = max(min(ierr,imax), imin)

    cte_prev = cte

    angle_by_side = Kp * perr + Ki * ierr + Kd * derr

    # Curve Control
    if rfront > 0 and rfront < 30:
        cnt = -5
        angle_by_front = -80
    if lfront > 0 and lfront < 30:
        cnt = 5
        angle_by_front = 80

    while cnt is not 0:
        if cnt > 0:
            cnt -= 1
            angle_by_front = +60
        else:
            cnt += 1
            angle_by_front = -60

    # Merge two handlings
    angle = angle_by_front + angle_by_side 

    # angle 값을 반환
    return angle

