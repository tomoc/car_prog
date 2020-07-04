import os
import sys
sys.path.append('/home/pi/togikai/togikai_function/')
import togikai_drive
import togikai_ultrasonic
import signal
import RPi.GPIO as GPIO
import Adafruit_PCA9685
import time
import numpy as np

# GPIOピン番号の指示方法
GPIO.setmode(GPIO.BOARD)

#超音波センサ初期設定
# Triger -- Fr:15, FrLH:13, RrLH:35, FrRH:32, RrRH:36
t_list=[15,13,35,32,36]
GPIO.setup(t_list,GPIO.OUT,initial=GPIO.LOW)
# Echo -- Fr:26, FrLH:24, RrLH:37, FrRH:31, RrRH:38
e_list=[26,24,37,31,38]
GPIO.setup(e_list,GPIO.IN)

#PWM制御の初期設定
##モータドライバ:PCA9685のPWMのアドレスを設定
pwm = Adafruit_PCA9685.PCA9685(address=0x40)
##動作周波数を設定
pwm.set_pwm_freq(60)

#アライメント調整済みPWMパラメータ読み込み
PWM_PARAM = togikai_drive.ReadPWMPARAM(pwm)

# HARD seiyaku
#saisyoukaiten hankei 50cm
CAR_MIN_R =50

#パラメータ
#前壁との最小距離
Cshort = 30
#右左折判定基準
short = 60
#モーター出力
tyuu = 40
kyou = 70
#データ記録用配列作成
d = np.zeros(4)
#操舵、駆動モーターの初期化
togikai_drive.Accel(PWM_PARAM,pwm,time,0)
togikai_drive.Steer(PWM_PARAM,pwm,time,0)

#一時停止（Enterを押すとプログラム実行開始）
print('Press any key to continue')
input()
time.sleep(1)
#開始時間
start_time = time.time()

def turn90(FRdis,direction):
    #direction 1Right -1left
    #Steer150 min radius 50cm
    #if FRdis >= CAR_MIN_R*1.05:
    togikai_drive.Accel(PWM_PARAM,pwm,time,40)
    togikai_drive.Steer(PWM_PARAM,pwm,time,150*direction)
    
def back():
    #direction 1Right -1left
    #Steer150 min radius 50cm
    #if FRdis >= CAR_MIN_R*1.05:
    togikai_drive.Accel(PWM_PARAM,pwm,time,-40)
    togikai_drive.Steer(PWM_PARAM,pwm,time,0)

#ここから走行用プログラム
try:
    while True:
        #Frセンサ距離
        FRdis = togikai_ultrasonic.Mesure(GPIO,time,15,26)
        #FrLHセンサ距離
        LHdis = togikai_ultrasonic.Mesure(GPIO,time,13,24)
        # #FrRHセンサ距離
        RHdis = togikai_ultrasonic.Mesure(GPIO,time,32,31)

        #togikai_drive.Accel(PWM_PARAM,pwm,time,40)
        #togikai_drive.Steer(PWM_PARAM,pwm,time,0)
        
        in_key = input()
        if in_key == 'k':
            turn90(FRdis,-1)
        elif in_key == 'l':
            turn90(FRdis,1)
        elif in_key == ',':
            back()
        
        elif FRdis <= CAR_MIN_R*1.1:
            turn90(FRdis,1)#1:Right,-1:left
        elif {FRdis > CAR_MIN_R*1.1 and in_key==''}:
            togikai_drive.Accel(PWM_PARAM,pwm,time,40)
            togikai_drive.Steer(PWM_PARAM,pwm,time,0)
        elif time.time()-start_time < 1:
            pass
        elif FRdis <=10:
            togikai_drive.Accel(PWM_PARAM,pwm,time,-100)
            togikai_drive.Steer(PWM_PARAM,pwm,time,0)
            time.sleep(0.1)
            togikai_drive.Accel(PWM_PARAM,pwm,time,0)
            togikai_drive.Steer(PWM_PARAM,pwm,time,0)
            GPIO.cleanup()
            d = np.vstack([d,[time.time()-start_time, FRdis, RHdis, LHdis]])
            np.savetxt('/home/pi/code/record_data.csv', d, fmt='%.3e')
            break
        #距離データを配列に記録
        d = np.vstack([d,[time.time()-start_time, FRdis, RHdis, LHdis]])
        #距離を表示
        print('Fr:{0:.1f} , FrRH:{1:.1f} , FrLH:{2:.1f}'.format(FRdis,RHdis,LHdis))
        time.sleep(0.05)

except KeyboardInterrupt:
    print('stop!')
    np.savetxt('/home/pi/code/record_data.csv', d, fmt='%.3e')
    togikai_drive.Accel(PWM_PARAM,pwm,time,0)
    togikai_drive.Steer(PWM_PARAM,pwm,time,0)
    GPIO.cleanup()
    

