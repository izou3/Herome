from sort import *
import numpy as np
import argparse
import cv2
import os
import time
from time import sleep
import RPi.GPIO as GPIO
import math
import serial

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference

def main():
    default_model_dir = './all_models'
    default_model = 'ssdlite_mobiledet_coco_qat_postprocess_edgetpu.tflite'
    default_labels = 'coco_labels.txt'
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', help='.tflite model path',
                        default=os.path.join(default_model_dir,default_model))
    parser.add_argument('--labels', help='label file path',
                        default=os.path.join(default_model_dir, default_labels))
    parser.add_argument('--top_k', type=int, default=3,
                        help='number of categories with highest score to display')
    parser.add_argument('--camera_idx', type=int, help='Index of which video source to use. ', default = 0)
    parser.add_argument('--threshold', type=float, default=0.2,
                        help='classifier score threshold')
    args = parser.parse_args()

    print('Loading {} with {} labels.'.format(args.model, args.labels))
    interpreter = make_interpreter(args.model)
    interpreter.allocate_tensors()
    labels = read_label_file(args.labels)
    inference_size = input_size(interpreter)

    cap = cv2.VideoCapture(args.camera_idx)
    
    # max_age: the maximum number of frames to keep alive a track without associated detection
    # min_hits: minimum number of associated detections before track is initialised
    # iou_threshold: the minimum iou threshold for a match
    sort_tracker = Sort(max_age=5, min_hits=3, iou_threshold=0.3)

    count = 1
    avgWidth = 0
    avgHorDisp = 0
    
    ledpin = 12
    right_back = 32
    
    left = 35
    left_back = 33

    # right side direction bits
    # AI1: 18
    # AI2: 16
    # BI1: 22
    # BI2: 24
    
    GPIO.setmode(GPIO.BOARD)
    
    GPIO.setup(ledpin, GPIO.OUT)
    GPIO.setup(right_back, GPIO.OUT)
    pi_pwm = GPIO.PWM(ledpin, 1000)
    right_back_pwm = GPIO.PWM(right_back, 1000)
    pi_pwm.start(0)
    right_back_pwm.start(0)
    
    GPIO.setup(left, GPIO.OUT)
    GPIO.setup(left_back, GPIO.OUT)
    left_pwm = GPIO.PWM(left, 1000)
    left_back_pwm = GPIO.PWM(left_back, 1000)
    left_pwm.start(0)
    left_back_pwm.start(0)
    
    # right side
    GPIO.setup(29, GPIO.OUT) # BI2 
    GPIO.setup(31, GPIO.OUT) #BI1
    
    # left side
    GPIO.setup(38, GPIO.OUT) #AI2
    GPIO.setup(40, GPIO.OUT) # AI1
    
    # Setup the serial COM port
    ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate = 115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )

    # Initialization
    MAX_LIN_VEL = 0.7147 # m/s based on 5V @ 210 RPMs
    MAX_ANG_VEL = 22 # rads/s 
    WHEEL_BASE = 0.135 # meters
    HALF_WHEEL_BASE = WHEEL_BASE * 0.5
    WHEEL_RADIUS = 0.0325 # meters
    RPM_TO_RAD_PER_S = 0.1047
    WHEEL_RADIUS_MUL_RPM_TO_RAD_PER_S = WHEEL_RADIUS * RPM_TO_RAD_PER_S
    MAX_RPM = (MAX_LIN_VEL + (HALF_WHEEL_BASE * MAX_ANG_VEL)) / (WHEEL_RADIUS_MUL_RPM_TO_RAD_PER_S)
    RPM_TO_PWM = 100 / 300 # PWM RANGE / MAX RPM  This is tuneable to determine reactivity. Smaller denominator means faster reactivity

    # Manually Tuned Parameters
    Kp = 0.2
    Kd = 0.5
    oldAngular = 0
    oldLinear = 0
    start_ms = time.time()
    lastID = 1

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break


        cv2_im = cv2.rotate(frame, cv2.ROTATE_180)

        cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
        
        cv2_im_rgb = cv2.resize(cv2_im_rgb, inference_size)
        
        run_inference(interpreter, cv2_im_rgb.tobytes())
        objs = get_objects(interpreter, args.threshold)[:args.top_k]

        # get attributes of frame
        height, width, channels = cv2_im.shape
        
        Xp = 320
        dX = 0
        dY = 0
        w = 0

        # get the first object detected from objs which is a list. Could hinder detection if it detects multiple objects in the frame and therefore deosnt draw the boxes
        # around the actual desired object
        # humans have id of 0
        # for testing mouse have id of 73
        loop = 0
        foundLastID = False
        
        for obj in objs:
            if (obj.id == 0):
                det_obj = obj
                scale_x, scale_y = width / inference_size[0], height / inference_size[1]

                bbox = det_obj.bbox.scale(scale_x, scale_y)
                
                '''
                Xp = int((int(bbox.xmax) + int(bbox.xmin)) / 2)
                dY = int((int(bbox.ymax) + int(bbox.ymin)) / 2)
                w = abs(int(bbox.xmax) - int(bbox.xmin))
                '''
                
                if (loop == 0):
                    dets = np.array([[int(bbox.xmin), int(bbox.ymin), int(bbox.xmax), int(bbox.ymax), det_obj.score]])
                else:
                    dets = np.insert(dets, 0, [int(bbox.xmin), int(bbox.ymin), int(bbox.xmax), int(bbox.ymax), det_obj.score], axis=0)
                loop = loop + 1
        
        if (loop > 0):
            track_bbs_ids = sort_tracker.update(dets)
            
            for detect in track_bbs_ids:
                if (detect[4] == lastID):
                    label = 'ID: {}'.format(detect[4])
                    foundLastID = True
                    
                    Xp = int((detect[2] + detect[0]) / 2)
                    w = abs(int(detect[2]) - int(detect[0]))
                    break
                
            if (not foundLastID):
                lastID = detect[4]
                label = 'ID: {}'.format(detect[4])

                Xp = int((detect[2] + detect[0]) / 2)
                w = abs(int(detect[2]) - int(detect[0]))
                
        if (loop == 0):
            Xp = 320
            w = 210
            ser.write(str.encode("2"))
    
        # calculate horizontal displacement for angular velocity
        dX = 320 - Xp # Xp  320
        linearVel = 0

        if (dX > 0):
          # turn counterclock wise or left. On left half side of frame
          angularVel = (MAX_ANG_VEL * (dX * dX)) / (102400)
        else:
          # turn clockwise or right. On right half side of frame
          angularVel = (-1) * ((MAX_ANG_VEL * (dX * dX)) / (102400))
        
        angularVel = Kp * angularVel + Kd * (angularVel - oldAngular)
        oldAngular = angularVel

        if (w > 220):
          # too far so linear vel should be positive and move back
          # linearVel = (-0.0015*w) - 0.12
          linearVel = (-0.001702*w) + 0.1244
          if (linearVel < -0.7147):
            linearV = -0.7147
        elif (w < 200):
          # too close so linear vel should be negative and move foward
          linearVel = (-0.003574*w) + 0.9647
          if (linearVel > 0.7147):
            linearVel = 0.7147
        else:   
            # w is between 300 and 340, which is within desired region 
            linearVel = 0
            
        linearVel = 0.45 * linearVel + 0.55 * oldLinear
        oldLinear = linearVel

        left_rpm = (linearVel - angularVel * HALF_WHEEL_BASE) / (WHEEL_RADIUS_MUL_RPM_TO_RAD_PER_S)
        
        if (left_rpm <= -300):
            left_rpm = -300
        elif (left_rpm >= 300):
            left_rpm = 300
        
    
        right_rpm = (linearVel + angularVel * HALF_WHEEL_BASE) / (WHEEL_RADIUS_MUL_RPM_TO_RAD_PER_S)
        
        if (right_rpm <= -300):
            right_rpm = -300
        elif (right_rpm >= 300):
            right_rpm = 300
        

        if (right_rpm < 0):
          # right is back
          # print('right back')
          GPIO.output(38, GPIO.LOW)
          GPIO.output(40, GPIO.HIGH)
        else:
          #right is back
          # print('right forward')
          GPIO.output(38, GPIO.HIGH)
          GPIO.output(40, GPIO.LOW)

        if (left_rpm < 0):
          # left is back
          # print('left back')
          GPIO.output(29, GPIO.LOW)
          GPIO.output(31, GPIO.HIGH)
        else:
          #left is forward
          # print('left forward')
          GPIO.output(29, GPIO.HIGH)
          GPIO.output(31, GPIO.LOW)
        
        '''
        print('left rpm pwm', left_rpm, int(abs(left_rpm) * RPM_TO_PWM))
        print('right rpm pwm', right_rpm, int(abs(right_rpm) * RPM_TO_PWM))
        '''
        
        pi_pwm.ChangeDutyCycle(int(abs(right_rpm) * RPM_TO_PWM))
        left_pwm.ChangeDutyCycle(int(abs(left_rpm) * RPM_TO_PWM))
        
        right_back_pwm.ChangeDutyCycle(int(abs(right_rpm) * RPM_TO_PWM))
        left_back_pwm.ChangeDutyCycle(int(abs(left_rpm) * RPM_TO_PWM))
        
        dets = np.empty([0,0])
        

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()