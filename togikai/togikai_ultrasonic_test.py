import os
import sys
sys.path.append('/home/pi/togikai/togikai_function/')
import togikai_func
import signal
import RPi.GPIO as GPIO
import Adafruit_PCA9685
import time
import numpy as np

GPIO.setmode(GPIO.BOARD)
GPIO.cleanup()
#初期設定
t_list=[15,13,35,32,36]
GPIO.setup(t_list,GPIO.OUT,initial=GPIO.LOW)
e_list=[26,24,37,31,38]
GPIO.setup(e_list,GPIO.IN)

def measure_test(trig,echo,target):
    stime = time.time()
    dis = np.zeros(400)
    for i in range(400):
        dis[i] = togikai_func.Mesure(GPIO,time,trig,echo)
    etime = time.time()
    ave_dis = np.mean(dis)
    if abs(ave_dis - target) < target/10:
        print('ooo --- OK!, result = {0:.1f} cm'.format(ave_dis))
        print('Sampling time = {0:.4f} s'.format((etime-stime)/200))
    else:
        print('xxx --- NG!, result = {0:.1f} cm'.format(ave_dis))


if __name__ == "__main__":
    try:
        print('Fr sensor test [30cm]  Press Enter')
        input()
        measure_test(15,26,30)
        print('FrLH sensor test [30cm]  Press Enter')
        input()
        measure_test(13,24,30)
        print('RrLH sensor test [30cm]  Press Enter')
        input()
        measure_test(35,37,30)
        print('FrRH sensor test [30cm]  Press Enter')
        input()
        measure_test(32,31,30)
        print('RrRH sensor test [30cm]  Press Enter')
        input()
        measure_test(36,38,30)
        
        
        
        print('Fr sensor test [60cm]  Press Enter')
        input()
        measure_test(15,26,60)
        print('FrLH sensor test [60cm]  Press Enter')
        input()
        measure_test(13,24,60)
        print('RrLH sensor test [60cm]  Press Enter')
        input()
        measure_test(35,37,60)
        print('FrRH sensor test [60cm]  Press Enter')
        input()
        measure_test(32,31,60)
        print('RrRH sensor test [60cm]  Press Enter')
        input()
        measure_test(36,38,60)
        
        
        
        
        
        print('Fr sensor test [120cm]  Press Enter')
        input()
        measure_test(15,26,120)
        print('FrLH sensor test [120cm]  Press Enter')
        input()
        measure_test(13,24,120)
        print('RrLH sensor test [120cm]  Press Enter')
        input()
        measure_test(35,37,120)
        print('FrRH sensor test [120cm]  Press Enter')
        input()
        measure_test(32,31,120)
        print('RrRH sensor test [120cm]  Press Enter')
        input()
        measure_test(36,38,120)
        # -------------
        GPIO.cleanup()

    except KeyboardInterrupt:
        print('stop!')
        GPIO.cleanup()
