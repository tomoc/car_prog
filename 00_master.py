import os
import sys
# sys.path.append('/home/pi/togikai/togikai_function/')
sys.path.append('D:/github/togikai_prog/togikai/togikai_function')
import togikai_drive
import togikai_ultrasonic
import signal
import RPi.GPIO as GPIO
import Adafruit_PCA9685
import time
import numpy as np
import pandas as pd

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
L_SHORT = 15    #initial50
L_LONG = 150     #initial100
C_SHORT = 25    #initial50
C_LONG = 170    #initial100
R_SHORT = 15    #initial50
R_LONG = 140     #initial100
ERR_DIS = 500   #20190907
FR = []
L = []
R = []
RL = []
RR = []
steering = []
front = []
comment2 = []
comment3 = []
log2 = []
test = []
data = []
data2 = []
joken_log = []
dousa_log = []
time_log = []
joken = 0
dousa = 0 
STOPDIS = 10
#kaunta
countinit = 0 #20190920

#一時停止（Enterを押すとプログラム実行開始）
print('Press any key to continue')
input()
time.sleep(1)
#開始時間
start_time = time.time()

#TODO
#必要な操作
#障害物、壁が迫ってきている場合は原則してカーブに備える。
# 左右間のセンサー距離で判定
#センサー距離の保存

def turn90(FRdis,direction):
    #direction 1Right -1left
    #Steer150 min radius 50cm
    #if FRdis >= CAR_MIN_R*1.05:
    togikai_drive.Accel(PWM_PARAM,pwm,time,40)
    togikai_drive.Steer(PWM_PARAM,pwm,time,150*direction)
    
def back(ped):
    #direction 1Right -1left
    #Steer150 min radius 50cm
    #if FRdis >= CAR_MIN_R*1.05:
    togikai_drive.Accel(PWM_PARAM,pwm,time,ped*-1)
    togikai_drive.Steer(PWM_PARAM,pwm,time,0)

def Accel(ped):
    togikai_drive.Accel(PWM_PARAM,pwm,time,ped)

def Steer(ang,direction):
    togikai_drive.Accel(PWM_PARAM,pwm,time,40)
    togikai_drive.Steer(PWM_PARAM,pwm,time,ang*direction)
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

        #togikai_drive.Accel(PWM_PARAM,pwm,time,40)
        #togikai_drive.Steer(PWM_PARAM,pwm,time,0)

        ###### L R hikak ######
                
        if L_dis > R_dis:
            LR_judge = 1#turn left
        else:
            LR_judge = 2#turn right(include L_dis = R_dis)

        ###### LL RR cornerjudge  ###### 
        ################################ for steerchange
        if R_L_dis + R_R_dis >80:
            LR_cj = 1
        else:
            LR_cj = 2

        #########################
        ###### joken start ######
        if L_dis > L_LONG and FRdis > C_LONG and R_dis > R_LONG:
            joken = 1#すべてＯＫ
        elif L_dis > L_LONG and FRdis > C_LONG and R_dis <= R_LONG and R_dis >= R_SHORT:
            joken = 2#右前がR_long以下
        elif L_dis > L_LONG and FRdis > C_LONG and R_dis < R_SHORT:
            joken = 3#右前の距離がR_short以下
        elif L_dis <= L_LONG and L_dis >= L_SHORT and FRdis > C_LONG and R_dis > R_LONG:
            joken = 4#左前がlong以下
        elif L_dis < L_SHORT and FRdis > C_LONG and R_dis > R_LONG:
            joken = 5#左前がshort以下
        elif L_dis > L_LONG and FRdis <= C_LONG and FRdis >= C_SHORT and R_dis > R_LONG:
            joken = 6#前がlong以下
        elif L_dis > L_LONG and FRdis < C_SHORT and R_dis > R_LONG:
            joken = 7#前がshot以下
        elif L_dis <= L_LONG and L_dis >= L_SHORT and FRdis > C_LONG and R_dis <= R_LONG and R_dis >= R_SHORT:
            joken = 8#左がlog以下で右もlong以下
        elif L_dis <= L_LONG and L_dis >= L_SHORT and FRdis > C_LONG and R_dis < R_SHORT:
            joken = 9#左がlog以下で右がshort以下
        elif L_dis < L_SHORT and FRdis > C_LONG and R_dis <= R_LONG and R_dis >= R_SHORT:
            joken = 10#左がshort以下で右がlong以下
        elif L_dis < L_SHORT and FRdis > C_LONG and R_dis < R_SHORT:
            joken = 11#左右がshort以下
        elif L_dis > L_LONG and FRdis <= C_LONG and FRdis >= C_SHORT and R_dis <= R_LONG and R_dis >= R_SHORT:
            joken = 12#前がlong以下右がlong以下
        elif L_dis > L_LONG and FRdis <= C_LONG and FRdis >= C_SHORT and R_dis < R_SHORT:
            joken = 13#\#前がlong以下右がshort以下
        elif L_dis > L_LONG and FRdis < C_SHORT and R_dis <= R_LONG and R_dis >= R_SHORT:
            joken = 14#前がshort以下右がlong以下
        elif L_dis > L_LONG and FRdis < C_SHORT and R_dis < R_SHORT:
            joken = 15#前がlong以下右がshort以下
        elif L_dis <= L_LONG and L_dis >= L_SHORT and FRdis <= C_LONG and FRdis >= C_SHORT and R_dis <= R_LONG and R_dis >= R_SHORT:
            joken = 16#すべてlong以下
        elif L_dis <= L_LONG and L_dis >= L_SHORT and FRdis <= C_LONG and FRdis >= C_SHORT and R_dis < R_SHORT:
            joken = 17#右がshort以下他long以下
        elif L_dis < L_SHORT and FRdis <= C_LONG and FRdis >= C_SHORT and R_dis <= R_LONG and R_dis >= R_SHORT:
            joken = 18#左がshort以下他がlong以下
        elif L_dis <= L_LONG and L_dis >= L_SHORT and FRdis < C_SHORT and R_dis <= R_LONG and R_dis >= R_SHORT:
            joken = 19#前がshort以下他がlong以下
        elif L_dis < L_SHORT and FRdis <= C_LONG and FRdis >= C_SHORT and R_dis < R_SHORT:
            joken = 20#左、右がshort以下前がlong以下
        elif L_dis <= L_LONG and L_dis >= L_SHORT and FRdis < C_SHORT and R_dis < R_SHORT:
            joken = 21#前、右がshort以下、左がlong以下
        elif L_dis < L_SHORT and FRdis < C_SHORT and R_dis <= R_LONG and R_dis >= R_SHORT:
            joken = 22#左、前がshort以下、右がlong以下
        elif L_dis <= L_LONG and L_dis >= L_SHORT and FRdis <= C_LONG and FRdis >= C_SHORT and R_dis > R_LONG:
            joken = 23#左、前がlong以下、RがOK
        elif L_dis < L_SHORT and FRdis <= C_LONG and FRdis >= C_SHORT and R_dis > R_LONG:
            joken = 24#左がshort以下、前がlong以下、右がOK
        elif L_dis <= L_LONG and L_dis >= L_SHORT and FRdis < C_SHORT and R_dis > R_LONG:
            joken = 25#左がlomg以下、前がshort以下、右がOK
        elif L_dis < L_SHORT and FRdis < C_SHORT and R_dis > R_LONG:
            joken = 26#左がshort以下、前がshort以下、右がOK
        elif L_dis < L_SHORT and FRdis < C_SHORT and R_dis < R_SHORT:
            joken = 27#すべてshort以下
        else:
            joken = 99

        ###### initial syori ######
        #######################
        if countinit == 1 or countinit == 3 or countinit == 5:
            dousa = 2
        elif countinit == 2 or countinit == 4 or countinit == 6:
            dousa = 3
        else:
        ####### dousa control ######
            if dousa == 0:#initial
                dousa = 1
            elif dousa == 1:#NOW Straight ahead
                if joken == 1 or joken == 8:
                    dousa = 1#Straight ahead 
                elif joken == 2 or joken == 12 or ( ( joken == 6 or joken == 16 ) and LR_judge == 1):
                    dousa = 2#Turn left
                elif joken == 4 or joken == 23 or ( ( joken == 6 or joken == 16 ) and LR_judge == 2):
                    dousa = 3#Turn right
                elif joken == 7 or joken == 11 or joken == 15 or joken == 19 or joken == 20 or joken == 21 or joken == 22 or joken == 26 or joken == 27:
                    dousa = 6#Back 
                elif joken == 3 or joken == 9 or joken == 13 or joken == 14 or joken == 17:
                    dousa = 7#Turn left back
                elif joken == 5 or joken == 10 or joken == 18 or joken == 24 or joken == 25:
                    dousa = 8#Turn right back
                else:
                    dousa = 99 #Fail Safe
            elif dousa == 2:#NOW Turn left
                if joken == 1 or joken == 8:
                    dousa = 1#Straight ahead 
                elif joken == 2 or joken == 12 or ((joken == 6 or joken == 16) and LR_judge == 1):
                    dousa = 2#Turn left
                elif joken == 4 or joken == 23 or ((joken == 6 or joken == 16) and LR_judge == 2):
                    dousa = 3#Turn right
                elif joken == 7 or joken == 11 or joken == 15 or joken == 19 or joken == 20 or joken == 21 or joken == 22 or joken == 26 or joken == 27:
                    dousa = 6#Back 
                elif joken == 3 or joken == 9 or joken == 13 or joken == 14 or joken == 17:
                    dousa = 7#Turn left back
                elif joken == 5 or joken == 10 or joken == 18 or joken == 24 or joken == 25:
                    dousa = 8#Turn right back
                else:
                    dousa = 99 #Fail Safe
            elif dousa == 3:#NOW Turn right
                if joken == 1 or joken == 8:
                    dousa = 1#Straight ahead 
                elif joken == 2 or joken == 12  or ((joken == 6 or joken == 16) and LR_judge == 1):
                    dousa = 2#Turn left
                elif joken == 4 or ((joken == 6 or joken == 16 )and LR_judge == 2) or joken == 23 :
                    dousa = 3#Turn right
                elif joken == 7 or joken == 11 or joken == 15 or joken == 19 or joken == 20 or joken == 21 or joken == 22 or joken == 26 or joken == 27:
                    dousa = 6#Back 
                elif joken == 3 or joken == 9 or joken == 13 or joken == 14 or joken == 17:
                    dousa = 7#Turn left back
                elif joken == 5 or joken == 10 or joken == 18 or joken == 24 or joken == 25:
                    dousa = 8#Turn right back
                else:
                    dousa = 99 #Fail Safe
            elif dousa == 6:#Back
                if joken == 1 or joken == 8:
                    dousa = 1#Straight ahead 
                elif joken == 2 or joken == 12 or ( ( joken == 6 or joken == 16 ) and LR_judge == 1):
                    dousa = 2#Turn left
                elif joken == 4 or joken == 23 or ( ( joken == 6 or joken == 16 ) and LR_judge == 2):
                    dousa = 3#Turn right
                elif joken == 7 or joken == 11 or joken == 15 or joken == 19 or joken == 20 or joken == 21 or joken == 22 or joken == 26 or joken == 27:
                    dousa = 6#Back 
                elif joken == 3 or joken == 9 or joken == 13 or joken == 14 or joken == 17:
                    dousa = 7#Turn left back
                elif joken == 5 or joken == 10 or joken == 18 or joken == 24 or joken == 25:
                    dousa = 8#Turn right back
                else:
                    dousa = 99 #Fail Safe
            elif dousa == 7:#NTurn left back
                if joken == 1 or joken == 8:
                    dousa = 1#Straight ahead 
                elif joken == 2 or joken == 12 or ( ( joken == 6 or joken == 16 ) and LR_judge == 1):
                    dousa = 2#Turn left
                elif joken == 4 or joken == 23 or ( ( joken == 6 or joken == 16 ) and LR_judge == 2):
                    dousa = 3#Turn right
                elif joken == 7 or joken == 11 or joken == 15 or joken == 19 or joken == 20 or joken == 21 or joken == 22 or joken == 26 or joken == 27:
                    dousa = 6#Back 
                elif joken == 3 or joken == 9 or joken == 13 or joken == 14 or joken == 17:
                    dousa = 7#Turn left back
                elif joken == 5 or joken == 10 or joken == 18 or joken == 24 or joken == 25:
                    dousa = 8#Turn right back
                else:
                    dousa = 99 #Fail Safe
            elif dousa == 8:#Turn right back
                if joken == 1 or joken == 8:
                    dousa = 1#Straight ahead 
                elif joken == 2 or joken == 12 or ( ( joken == 6 or joken == 16 ) and LR_judge == 1):
                    dousa = 2#Turn left
                elif joken == 4 or joken == 23 or ( ( joken == 6 or joken == 16 ) and LR_judge == 2):
                    dousa = 3#Turn right
                elif joken == 7 or joken == 11 or joken == 15 or joken == 19 or joken == 20 or joken == 21 or joken == 22 or joken == 26 or joken == 27:
                    dousa = 6#Back 
                elif joken == 3 or joken == 9 or joken == 13 or joken == 14 or joken == 17:
                    dousa = 7#Turn left back
                elif joken == 5 or joken == 10 or joken == 18 or joken == 24 or joken == 25:
                    dousa = 8#Turn right back
                else:
                    dousa = 99 #Fail Safe
            else:
                dousa = 99 #Fail Safe
        
        ####### Kudou control ######
        if dousa == 1:
            Accel(100)
            Steer(0,1)
            comment = '直進中'
        elif dousa == 2:
            Accel(100)
            if LL_RR_cj == 1 and LL_dis < LLDIS: #20190922
                Steer(50,1)
                comment = '左旋回steer50'
            else:
                Steer(100,1)
                comment = '左旋回steer100' 
        elif dousa == 3:
            Accel(tyuu)
            if LL_RR_cj == 1 and RR_dis < RRDIS: #20190922
                Steer(50,-1)
                comment = '右旋回steer-50'
            else:
                Steer(100,-1)
                comment = '右旋回steer-100'
        elif dousa == 6:
            back(100)
            Steer(0)
            comment = 'バック'
            time.sleep(1.0)#I have no idea, this waiting time necessity.Initial0.2
        elif dousa == 7:
            back(100)
            Steer(100,-1)
            time.sleep(1.0)#I have no idea, this waiting time necessity.Initial0.2
            comment = '右バック'
        elif dousa == 8:
            back(100)
            Steer(100,1)
            time.sleep(1.0)#I have no idea, this waiting time necessity.Initial0.2
            comment = '左バック'
        else:
            Accel(100)
            Steer(0)
            comment = '直進中'###### countinit ######

        if countinit <= 7:
            countinit = countinit + 1
        else:
            countinit = 7

        #ループ抜ける用
        if L_dis <= STOPDIS and R_dis <= STOPDIS:
            count = count + 1
        else:
            count = 0

        if count == STOPTIME:
            print("escape roop")
            break#抜ける

        #距離記録用
        FR.append(FRdis)
        L.append(L_dis)
        R.append(R_dis)
        RL.append(R_L_dis)
        RR.append(R_R_dis)
        comment2.append(comment)
        joken_log.append(joken)#20190813
        dousa_log.append(dousa)#20190813
        time_log.append(time.time()-start_time)
            # in_key = input()
        # if in_key == 'k':
        #     turn90(FRdis,-1)
        # elif in_key == 'l':
        #     turn90(FRdis,1)
        # elif in_key == ',':
        #     back()
        
        # elif FRdis <= CAR_MIN_R*1.1:
        #     turn90(FRdis,1)#1:Right,-1:left
        # elif {FRdis > CAR_MIN_R*1.1 and in_key==''}:
        #     togikai_drive.Accel(PWM_PARAM,pwm,time,40)
        #     togikai_drive.Steer(PWM_PARAM,pwm,time,0)
        # elif time.time()-start_time < 1:
        #     pass
        # elif FRdis <=10:
        #     togikai_drive.Accel(PWM_PARAM,pwm,time,-100)
        #     togikai_drive.Steer(PWM_PARAM,pwm,time,0)
        #     time.sleep(0.1)
        #     togikai_drive.Accel(PWM_PARAM,pwm,time,0)
        #     togikai_drive.Steer(PWM_PARAM,pwm,time,0)
        #     GPIO.cleanup()
        #     d = np.vstack([d,[time.time()-start_time, FRdis, RHdis, LHdis]])
        #     np.savetxt('/home/pi/code/record_data.csv', d, fmt='%.3e')
        #     break
        #距離データを配列に記録
        # d = np.vstack([d,[time.time()-start_time, FRdis, R_dis, L_dis,R_R_dis,R_L_dis,comment,joken,dousa]])
        #距離を表示
        print('Fr:{0:.1f} , FrRH:{1:.1f} , FrLH:{2:.1f}'.format(FRdis,RHdis,LHdis))
        time.sleep(0.05)

except KeyboardInterrupt:
    print('stop!')
    data = [np.array(FR),np.array(L),np.array(R),np.array(RL),np.array(RR),np.array(comment2),np.array(joken_log),np.array(dousa_log),np.array(time_log)]
    data2 = np.array(data).T.tolist()
    # np.savetxt('/home/pi/code/record_data.csv', d, fmt='%.3e')
    with open('/home/pi/code/record_data.csv','w') as f:
        writer = csv.writer(f)
    writer.writerow(['FR','L','R','RL','RR','comment','joken','dousa','time'])
    writer.writerows(data2)
    togikai_drive.Accel(PWM_PARAM,pwm,time,0)
    togikai_drive.Steer(PWM_PARAM,pwm,time,0)
    GPIO.cleanup()
    

