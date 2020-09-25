import os
import sys
sys.path.append('/home/pi/togikai/togikai_function/')
#sys.path.append('D:/github/togikai_prog/togikai/togikai_function')
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

#定義
L_short = 30    #initial50
L_mid = 60
L_long = 150     #initial100
C_short = 30    #initial50
C_mid = 60
C_long = 170    #initial100
R_short = 50    #initial50
R_mid = 60
R_long = 140     #initial100
ERR_DIS = 500   #20190907
action=0

target_d=35
count =0
st_ang=0
old_dist=np.zeros([1,3])
count = 0
#Kp =100/target_d
Kp = 2
Ki =0.05
Kd = 0.1
#Kd = 100/target_d
# magarisugiru
#massukguni recover dekinaikoto

#一時停止（Enterを押すとプログラム実行開始）
print('Press any key to continue')
input()
time.sleep(1)
#開始時間
start_time = time.time()

#TODO
#障害物、壁が迫ってきている場合は右にカーブ
# 左右間のセンサー距離で判定
#センサー距離の保存

def back(ped):
    #direction 1Right -1left
    #Steer150 min radius 50cm
    #if FRdis >= CAR_MIN_R*1.05:
    togikai_drive.Accel(PWM_PARAM,pwm,time,ped*-1)
    togikai_drive.Steer(PWM_PARAM,pwm,time,0)

def Accel(ped):
    togikai_drive.Accel(PWM_PARAM,pwm,time,ped)

def Steer(ang,direction):#1:Right,-1:left
    togikai_drive.Accel(PWM_PARAM,pwm,time,50)
    togikai_drive.Steer(PWM_PARAM,pwm,time,ang*direction)
def Steer_k(ang):#1:-100~100
    togikai_drive.Accel(PWM_PARAM,pwm,time,50)
    togikai_drive.Steer(PWM_PARAM,pwm,time,ang)
    
def keep_dist(RLdis,Ldis,angl1):
    
    if RLdis>Ldis:
        now_dist = Ldis
    else:
        now_dist = RLdis
    
    if abs(old_dist[0,0]-now_dist)>100:
        now_dist = old_dist[0,0]
    #Kp =120/target_d
    angl=0
    direc=0
    dis2 = old_dist[0,2]
    dis1 = old_dist[0,1]
    dis=target_d-now_dist
    steer_ang = angl1+(dis-dis1)*Kp + Ki*dis + Kd*((dis-dis1)-(dis1-dis2))
    print('dis{0}, now_dis{1}'.format(dis,now_dist))
    #steer_ang=angl1+(dis-dis1)*Kp + Ki*dis + Kd*((dis-dis1)-(dis1-dis2))
    #steer_ang =dis/target_d*120
    #steer_ang = 100
    
    #zyouken seiri
    #if steer_ang>0:
    #    direc=1
    #else:
    #    direc=-1
    #    steer_ang = abs(steer_ang)
    #if now_dist <target_d:
    #    angl = abs(steer_ang)
    #    direc=1
    #elif now_dist>target_d:
    #    angl = abs(steer_ang)
    #    direc=-1
    old_dist[0,0] = now_dist
    old_dist[0,1] = dis
    old_dist[0,2] = dis1
    if steer_ang>100:
        steer_ang=100
    if steer_ang<-100:
        steer_ang = -100
    return steer_ang,direc


#ここから走行用プログラム
try:
    while True:
        #Frセンサ距離
        FRdis = togikai_ultrasonic.Mesure(GPIO,time,15,26)
        #FrLHセンサ距離
        L_dis = togikai_ultrasonic.Mesure(GPIO,time,13,24)
        #FrRHセンサ距離
        R_dis = togikai_ultrasonic.Mesure(GPIO,time,32,31)
        #RrLHセンサ距離
        R_L_dis = togikai_ultrasonic.Mesure(GPIO,time,35,37)
        #RrRHセンサ距離
        R_R_dis = togikai_ultrasonic.Mesure(GPIO,time,36,38)
        
        #if FRdis>C_mid:
        if count ==0:
            old_dist[0,0] = L_dis
            count = count+1
        st_ang,st_direc = keep_dist(R_L_dis,L_dis,st_ang)
        #Steer(st_ang,st_direc)
        Steer_k(st_ang)
        action =5
    
        #elif count5 == 10:
        #    count5 = 0
        #   action = 0

        #togikai_drive.Accel(PWM_PARAM,pwm,time,40)
        #togikai_drive.Steer(PWM_PARAM,pwm,time,0)


        
        # #ver2書きかけ
        # if FRdis>C_mid:
        #     if L_dis<=L_short and R_dis>R_short:#左が閾値未満
        #         action = 11#右旋回
        #     elif L_dis>L_short and R_dis<=R_short:#右が閾値未満
        #         action =21#左旋回
        #     elif L_dis>L_short and R_dis>R_short:
        #         action=0
        #     elif L_dis<=L_short and R_dis<=R_short:#両方とも狭いときは距離が長いほうに曲がる
        #         if L_dis<R_dis:
        #             action = 12
        #         else:
        #             action = 22

        #if FRdis<C_mid:#まえとの距離がみじかいとき旋回しろ大
        #    if L_dis<=L_short and R_dis<=R_short:
         #       action=4#後退
          #  elif L_dis<=L_short and R_dis>R_short:#左が閾値未満
           #     action = 1#右旋回
            #    st_ang = abs(L_dis-L_short)/L_short*100
            #elif L_dis>L_short and R_dis<=R_short:#右が閾値未満
             #   action =2#左旋回
             #   st_ang = abs(R_dis-R_short)/R_short*100
           # elif L_dis>L_short and R_dis>R_short:
            #    action=0


        #ver1
        # if FRdis>C_mid and R_dis>R_short and L_dis>L_short:
        #     #直進
        #     action=0
        # elif FRdis<C_mid and L_dis<L_short:
        #     action=3
        
        # elif FRdis<C_mid and L_dis<L_short and R_dis>R_mid:
        #     #カーブがあるとき右まわり
        #     action=1
        # elif FRdis<C_mid and L_dis>L_mid and R_dis<R_short:
        #     #right side of wall
        #     action=2
        # elif action==1 and FRdis<C_mid and L_dis>R_dis:
        #     #右からの左まわ
        #     action=2
        # elif action ==1 and FRdis>C_mid and L_dis>L_short:
        #     action=0
        # elif action==2 and FRdis<C_mid and L_dis<R_dis:
        #     #右からの左まわ
        #     action=1
            
        
#        elif FRdis>50 and R_L_dis>20 and L_dis>40:
            #壁から離れたら接近
#            Steer(50,-1)
        
        
        #elif FRdis <= CAR_MIN_R*1.1:
        #    turn90(FRdis,1)
        #elif {FRdis > CAR_MIN_R*1.1}:
        #   togikai_drive.Accel(PWM_PARAM,pwm,time,40)
        #    togikai_drive.Steer(PWM_PARAM,pwm,time,0)
        if time.time()-start_time < 1:
            pass
        elif FRdis <=10:
            togikai_drive.Accel(PWM_PARAM,pwm,time,-100)
            togikai_drive.Steer(PWM_PARAM,pwm,time,0)
            time.sleep(0.1)
            togikai_drive.Accel(PWM_PARAM,pwm,time,0)
            togikai_drive.Steer(PWM_PARAM,pwm,time,0)
            GPIO.cleanup()
            d = np.vstack([d,[time.time()-start_time, FRdis, R_dis, L_dis]])
            np.savetxt('/home/pi/code/record_data.csv', d, fmt='%.3e')
            break
        
        #action
        if action==0:
            Accel(50)
        elif action==1:
            Steer(st_ang,1)
        elif action==2:
            Steer(st_ang,-1)
        
        elif action==3:
            steer_angle = (L_short-L_dis)/L_short*100
            Steer(100,1)
        
            
        
        #距離データを配列に記録
        d = np.vstack([d,[time.time()-start_time, FRdis, R_dis, L_dis]])
        #距離を表示
        print('Fr:{0:.1f} , FrRH:{1:.1f} , FrLH:{2:.1f}, RrLH:{3:.1f}, action:{4:.1f}, ang:{5:.1f},ste:{6:.1f}'.format(FRdis,R_dis,L_dis,R_L_dis,action,st_ang,st_direc))
        time.sleep(0.02)

except KeyboardInterrupt:
    print('stop!')
    np.savetxt('/home/pi/code/record_data.csv', d, fmt='%.3e')
    togikai_drive.Accel(PWM_PARAM,pwm,time,0)
    togikai_drive.Steer(PWM_PARAM,pwm,time,0)
    GPIO.cleanup()