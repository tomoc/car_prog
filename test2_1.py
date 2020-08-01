import os
import sys
sys.path.append('/home/pi/Documents/Ogoshi_Togikai/togikai/togikai_function')
import togikai_drive2
import togikai_ultrasonic
import signal
import RPi.GPIO as GPIO
import Adafruit_PCA9685
import time
import numpy as np
import picamera
import picamera.array
import cv2
import datetime
import pygame
from pygame.locals import *
import csv

d = np.zeros(4)
#PWM制御の初期設定
##モータドライバ:PCA9685のPWMのアドレスを設定
pwm = Adafruit_PCA9685.PCA9685(address=0x40)
##動作周波数を設定
pwm.set_pwm_freq(60)
#アライメント調整済みPWMパラメータ読み込み
PWM_PARAM = togikai_drive2.ReadPWMPARAM(pwm)

#操舵、駆動モーターの初期化
togikai_drive2.Accel(PWM_PARAM,pwm,time,0)
togikai_drive2.Steer(PWM_PARAM,pwm,time,0)




SCREEN_SIZE = (640, 480)
pygame.init()
screen = pygame.display.set_mode(SCREEN_SIZE)
pygame.display.set_caption("window test")
X_CENTER = int(SCREEN_SIZE[0]/2)
Y_CENTER = int(SCREEN_SIZE[1]/2)
[circle_x, circle_y] = [X_CENTER, Y_CENTER]
            
pygame.joystick.init()
try:
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print("Joystick Name: " + str(joy.get_numbuttons()))
    print("Number of Button : " + str(joy.get_numaxes()))
    print("Numberr of Hats : " + str(joy.get_numhats()))
        
except pygame.error:
    print('Joystick was not detected!')


videopath = '/home/pi/Videos'
start_time = time.time()
time_log = []
accel_log =[]
steer_log = []

with picamera.PiCamera() as camera:
    camera.vflip = True
    camera.hflip = True
            
            # Camera resolution setting
    camera.resolution = (320, 240)
            
            # Real time procedure
    with picamera.array.PiRGBArray(camera) as stream:
                #Open video file for logging
        curstr = datetime.datetime.now().strftime("%Y%m%d_%H")
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(str(videopath)+'/video_'+curstr+'.avi',fourcc, 20.0, (320,240))
                
                #wait until camera funciton becomes stable
        time.sleep(2)
        
        try:
            while True: #Repeat taking picture by camera & write file to folder
                    # Obtaion camera picture
                camera.capture(stream, 'bgr', use_video_port=True)
                    
                    #Obtain the date now
                nowstr = datetime.datetime.now().strftime("%Y%m%d_%H")
                    
                    #change new video file at the time of next
                if curstr != nowstr:
                    curstr = nowstr
                    out.release()
                    out = cv2.VideoWriter(str(videopath)+'/video_'+curstr+'.avi',fourcc, 20.0, (640,480))
                        
                    
                    #Save Video
                out.write(stream.array)
                    
                    #Show taking picture result
                cv2.imshow('camera', stream.array)
                cv2.imwrite('/home/pi/capture_{}.png'.format(time.time()-start_time),stream.array)
                    
                    #Finish if you push the any key
                if cv2.waitKey(1) < 255:
                    break
                    
                    #Dispose the loading data by camera
                stream.seek(0)
                stream.truncate()
            
                screen.fill((0,0,0))
                for event in pygame.event.get():
                    if event.type == QUIT:
                        sys.exit()
            
                circle_x = int((joy.get_axis(3)+1) * X_CENTER)
                circle_y = int((joy.get_axis(1)+1) * Y_CENTER)

                steer = int((joy.get_axis(3)) * 100)
                accel = int((-joy.get_axis(1)) * 40)
            #print(str(accel))
             #   togikai_drive.Accel(PWM_PARAM,pwm,time,tyuu)
                togikai_drive2.Accel(PWM_PARAM,pwm,time,accel)
                togikai_drive2.Steer(PWM_PARAM,pwm,time,steer)
            
                time_log.append(time.time()-start_time)
                accel_log.append(accel)
                steer_log.append(steer)
            
                # Close the window
            out.release()
            cv2.destroyAllWindows()
        except KeyboardInterrupt:            
            print('stop!')
            data = [np.array(time_log),np.array(accel_log),np.array(steer_log)]
            data2 = np.array(data).T.tolist()
            # np.savetxt('/home/pi/code/record_data.csv', d, fmt='%.3e')
            with open('/home/pi/record_data.csv','w') as f:
                writer = csv.writer(f)
                writer.writerow(['time','accel','steer'])
                writer.writerows(data2)
            togikai_drive.Accel(PWM_PARAM,pwm,time,0)
            togikai_drive.Steer(PWM_PARAM,pwm,time,0)
            GPIO.cleanup()
        
        
        
