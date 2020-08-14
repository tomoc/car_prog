#Update history
#2019/08/08 develop program of basic control(judge and some of defines).1-1
#2019/08/10 develop program of basic control(dousa contorol).1-2
#2019/08/13 develop program of basic control(kudou contorol).1-3
#2019/08/14 change const, jyoken.1-4,1-5
#2019/09/07 sensor error riset,smoothing 1-6

###program start###

import RPi.GPIO as GPIO
import time
import os
import numpy as np
import math#20190813
#pipimport pandas as pd

GPIO.setmode( GPIO.BCM )
#駆動モータIO定義
PWMA=21
Ain1=26
Ain2=20
#モータ動作許可IO定義
STBY=16
#操舵モータIO定義
PWMB=6
Bin1=19
Bin2=13
#まとめて初期設定
m_list=[PWMA,Ain1,Ain2,STBY,PWMB,Bin1,Bin2]
GPIO.setup(m_list, GPIO.OUT, initial=GPIO.LOW )

#超音波センサ１中央定義
trig1=23
echo1=8
#超音波センサ２左前定義
trig2=15
echo2=9
#超音波センサ3右前定義
trig3=18
echo3=7
#超音波センサ4左横定義
trig4=14
echo4=11
#超音波センサ5右横定義
trig5=24
echo5=1
#まとめて初期設定
t_list=[trig1,trig2,trig3,trig4,trig5]
GPIO.setup(t_list,GPIO.OUT,initial=GPIO.LOW)
e_list=[echo1,echo2,echo3,echo4,echo5]
GPIO.setup(e_list,GPIO.IN)

#定義
L_SHORT = 15    #initial50
L_LONG = 150     #initial100
C_SHORT = 25    #initial50
C_LONG = 170    #initial100
R_SHORT = 15    #initial50
R_LONG = 140     #initial100
ERR_DIS = 500   #20190907

joken = 0
dousa = 0 

# test
t1 = 0
t2 = 0
supersteer = 0
C = []
L = []
R = []
LL = []
RR = []
steering = []
front = []
comment2 = []
comment3 = []
log2 = []
test = []
data = []
data2 = []
joken_log = []#20190813
dousa_log = []#20190813
time_log = []#20190907

#kaunta
countinit = 0 #20190920

#駆動モータ制御関数
def Accel(Duty):
    if Duty >= 0:
        GPIO.output( STBY, 1 )
        GPIO.output( Ain1, 0 )
        GPIO.output( Ain2, 1 )
        p_A.ChangeDutyCycle(Duty)
    else:
        Duty = abs(Duty)
        GPIO.output( STBY, 1 )
        GPIO.output( Ain1, 1 )
        GPIO.output( Ain2, 0 )
        p_A.ChangeDutyCycle(Duty)

#操舵モータ制御関数
def Steer(Duty):
    if Duty >= 0:
        supersteer = Duty
        GPIO.output( STBY, 1 )
        GPIO.output( Bin1, 0 )
        GPIO.output( Bin2, 1 )
        p_B.ChangeDutyCycle(Duty)
        
    else:
        supersteer = Duty
        Duty = abs(Duty)
        GPIO.output( STBY, 1 )
        GPIO.output( Bin1, 1 )
        GPIO.output( Bin2, 0 )
        p_B.ChangeDutyCycle(Duty)
        
        
#障害物センサ測定関数
def Mesure(trig,echo):
    sigoff = 0
    sigon = 0
    GPIO.output(trig,GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig,GPIO.LOW)
    while(GPIO.input(echo)==GPIO.LOW):
        sigoff=time.time()
    while(GPIO.input(echo)==GPIO.HIGH):
        sigon=time.time()
    return round(((sigon - sigoff)*34000/2))


print("set ini para")
#For escape from sensing roop
STOPTIME = 3
STOPDIS = 10

waittime = 0.01



#駆動DUTY
jyaku = 100
tyuu = 100
kyou = 100

#PWM設定
p_A = GPIO.PWM(PWMA, 1000);
p_B = GPIO.PWM(PWMB, 1000);

#PWM初期化
p_A.start(0)
p_B.start(0)
      


count = 0
count1 = 0
print ("start roop")
#動作プログラム


while True :
#count time
    t1 = t2
    t2 = time.time()
    t0 = t2 - t1
    C_dis = Mesure(trig1,echo1)     #cal distance 今回値が400を超えたら400を採用要追加
    L_dis = Mesure(trig2,echo2)     #cal distance
    R_dis = Mesure(trig3,echo3)     #cal distance
    LL_dis = Mesure(trig4,echo4)    #cal distance
    RR_dis = Mesure(trig5,echo5)    #cal distance
    C_dis_prvs = C_dis              #store previos data cal distance
    L_dis_prvs = L_dis              #store previos data cal distance
    R_dis_prvs = R_dis              #store previos data cal distance
    LL_dis_prvs = LL_dis            #store previos data cal distance
    RR_dis_prvs = RR_dis            #store previos data cal distance

#######################
###### L R hikak ######
            
    if L_dis > R_dis:
        LR_judge = 1#turn left
    else:
        LR_judge = 2#turn right(include L_dis = R_dis)

###### L R hikak end ######
###########################

###### LL RR cornerjudge  ###### 20190922
################################ for steerchange
    if LL_dis + RR_dis >80:
        LL_RR_cj = 1
    else:
        LL_RR_cj = 2
        
###### LL RR cornerjudge  end###
################################

#########################
###### joken start ######
    if L_dis > L_LONG and C_dis > C_LONG and R_dis > R_LONG:
        joken = 1#すべてＯＫ
    elif L_dis > L_LONG and C_dis > C_LONG and R_dis <= R_LONG and R_dis >= R_SHORT:
        joken = 2#右前がR_long以下
    elif L_dis > L_LONG and C_dis > C_LONG and R_dis < R_SHORT:
        joken = 3#右前の距離がR_short以下
    elif L_dis <= L_LONG and L_dis >= L_SHORT and C_dis > C_LONG and R_dis > R_LONG:
        joken = 4#左前がlong以下
    elif L_dis < L_SHORT and C_dis > C_LONG and R_dis > R_LONG:
        joken = 5#左前がshort以下
    elif L_dis > L_LONG and C_dis <= C_LONG and C_dis >= C_SHORT and R_dis > R_LONG:
        joken = 6#前がlong以下
    elif L_dis > L_LONG and C_dis < C_SHORT and R_dis > R_LONG:
        joken = 7#前がshot以下
    elif L_dis <= L_LONG and L_dis >= L_SHORT and C_dis > C_LONG and R_dis <= R_LONG and R_dis >= R_SHORT:
        joken = 8#左がlog以下で右もlong以下
    elif L_dis <= L_LONG and L_dis >= L_SHORT and C_dis > C_LONG and R_dis < R_SHORT:
        joken = 9#左がlog以下で右がshort以下
    elif L_dis < L_SHORT and C_dis > C_LONG and R_dis <= R_LONG and R_dis >= R_SHORT:
        joken = 10#左がshort以下で右がlong以下
    elif L_dis < L_SHORT and C_dis > C_LONG and R_dis < R_SHORT:
        joken = 11#左右がshort以下
    elif L_dis > L_LONG and C_dis <= C_LONG and C_dis >= C_SHORT and R_dis <= R_LONG and R_dis >= R_SHORT:
        joken = 12#前がlong以下右がlong以下
    elif L_dis > L_LONG and C_dis <= C_LONG and C_dis >= C_SHORT and R_dis < R_SHORT:
        joken = 13#\#前がlong以下右がshort以下
    elif L_dis > L_LONG and C_dis < C_SHORT and R_dis <= R_LONG and R_dis >= R_SHORT:
        joken = 14#前がshort以下右がlong以下
    elif L_dis > L_LONG and C_dis < C_SHORT and R_dis < R_SHORT:
        joken = 15#前がlong以下右がshort以下
    elif L_dis <= L_LONG and L_dis >= L_SHORT and C_dis <= C_LONG and C_dis >= C_SHORT and R_dis <= R_LONG and R_dis >= R_SHORT:
        joken = 16#すべてlong以下
    elif L_dis <= L_LONG and L_dis >= L_SHORT and C_dis <= C_LONG and C_dis >= C_SHORT and R_dis < R_SHORT:
        joken = 17#右がshort以下他long以下
    elif L_dis < L_SHORT and C_dis <= C_LONG and C_dis >= C_SHORT and R_dis <= R_LONG and R_dis >= R_SHORT:
        joken = 18#左がshort以下他がlong以下
    elif L_dis <= L_LONG and L_dis >= L_SHORT and C_dis < C_SHORT and R_dis <= R_LONG and R_dis >= R_SHORT:
        joken = 19#前がshort以下他がlong以下
    elif L_dis < L_SHORT and C_dis <= C_LONG and C_dis >= C_SHORT and R_dis < R_SHORT:
        joken = 20#左、右がshort以下前がlong以下
    elif L_dis <= L_LONG and L_dis >= L_SHORT and C_dis < C_SHORT and R_dis < R_SHORT:
        joken = 21#前、右がshort以下、左がlong以下
    elif L_dis < L_SHORT and C_dis < C_SHORT and R_dis <= R_LONG and R_dis >= R_SHORT:
        joken = 22#左、前がshort以下、右がlong以下
    elif L_dis <= L_LONG and L_dis >= L_SHORT and C_dis <= C_LONG and C_dis >= C_SHORT and R_dis > R_LONG:
        joken = 23#左、前がlong以下、RがOK
    elif L_dis < L_SHORT and C_dis <= C_LONG and C_dis >= C_SHORT and R_dis > R_LONG:
        joken = 24#左がshort以下、前がlong以下、右がOK
    elif L_dis <= L_LONG and L_dis >= L_SHORT and C_dis < C_SHORT and R_dis > R_LONG:
        joken = 25#左がlomg以下、前がshort以下、右がOK
    elif L_dis < L_SHORT and C_dis < C_SHORT and R_dis > R_LONG:
        joken = 26#左がshort以下、前がshort以下、右がOK
    elif L_dis < L_SHORT and C_dis < C_SHORT and R_dis < R_SHORT:
        joken = 27#すべてshort以下
    else:
        joken = 99

###### joken end ######
#######################

###### initial syori ######
#######################
    if countinit == 1 or countinit == 3 or countinit == 5:
       dousa = 2
    elif countinit == 2 or countinit == 4 or countinit == 6:
       dousa = 3
    else:

############################
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

####### dousa control end ######
################################


############################
####### Kudou control ######
    if dousa == 1:
        Accel(tyuu)
        Steer(0)
        comment = '直進中'
    elif dousa == 2:
        Accel(tyuu)
        if LL_RR_cj == 1 and LL_dis < LLDIS: #20190922
            Steer(50)
            comment = '左旋回steer50'
        else:
            Steer(100)
            comment = '左旋回steer100' 
    elif dousa == 3:
        Accel(tyuu)
        if LL_RR_cj == 1 and RR_dis < RRDIS: #20190922
            Steer(-50)
            comment = '右旋回steer-50'
        else:
            Steer(-100)
            comment = '右旋回steer-100'
    elif dousa == 6:
        Accel(-jyaku)
        Steer(0)
        comment = 'バック'
        time.sleep(1.0)#I have no idea, this waiting time necessity.Initial0.2
    elif dousa == 7:
        Accel(-jyaku)
        Steer(-100)
        time.sleep(1.0)#I have no idea, this waiting time necessity.Initial0.2
        comment = '右バック'
    elif dousa == 8:
        Accel(-jyaku)
        Steer(100)
        time.sleep(1.0)#I have no idea, this waiting time necessity.Initial0.2
        comment = '左バック'
    else:
        Accel(tyuu)
        Steer(0)
        comment = '直進中'

###### Kudou control end ######
###############################           

###############################
###### countinit ######

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

#測定値表示用
    count1 = count1 + 1
#    time.sleep(0.2)
    
    if count1 == 1:
        count1 = 0

#距離記録用
    C.append(C_dis)
    L.append(L_dis)
    R.append(R_dis)
    LL.append(LL_dis)
    RR.append(RR_dis)
    steering.append(supersteer)
    front.append(C_dis)
    comment2.append(comment)
    joken_log.append(joken)#20190813
    dousa_log.append(dousa)#20190813
    time_log.append(t0)
#    log2.append(log)

#    time.sleep(0.01) # 190918_add
    
#    steering = STBY

#while抜けた後の停止処理


# For csv_S

data = [np.array(C),np.array(L),np.array(R),np.array(LL),np.array(RR),np.array(steering),np.array(front),np.array(comment2),np.array(joken_log),np.array(dousa_log),np.array(time_log)]
data2 = np.array(data).T.tolist()


import csv

with open('data.csv','w') as f:
        writer = csv.writer(f)
    writer.writerow(['C','L','R','LL','RR','steering','front/back','comment','joken','dousa','time'])
    writer.writerows(data2)
    



# For csv_E
p_A.stop()
p_B.stop()
GPIO.cleanup()
print("End!!")


#SPEC HC-SR04(Sensor)
#測距範囲：2~400cm
#センサー基板正面を中心とした15度の範囲、lsb:0.3ｃｍ
#電源電圧：DC 5.0V
#動作電流：1.5mA
#動作周波数：40kHz
#トリガ信号：10msec（TTL level-pulse）
#エコー出力信号：反射（往復）時間
#サイズ：45×20×15ｍｍ

#バックは基本失敗なのでそれまでの入力と判定を不正解値として学習させる
#学習は前後左右センサ入力値による判断結果良しあし