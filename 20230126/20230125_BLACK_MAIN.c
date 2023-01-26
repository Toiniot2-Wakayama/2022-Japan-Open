#include "D_Main.h"
#include "D_I2C.h"
#include "D_SIO.h"
#include "D_EIO.h"

/*
メインCPU
CN1[0]  ボール前
CN2[1]  ボール左前
CN3[2]  ボール左
CN4[3]  ボール左後ろ
CN5[4]  ボール後ろ
CN6[5]  ボール右後ろ
CN7[6]  ボール右
CN8[7]  ボール右前
CN9[8]  N.C.
CN10[9] N.C.
I2C 4chモータードライバ

サブCPU
CN1[0]  ライン前
CN2[1]  ライン左
CN3[2]  ライン後ろ
CN4[3]  ライン右
CN5[4]  N.C.
CN6[5]  Bボタン（決定）
CN7[6]  Aボタン（選択）
CN8[7]  （超音波右）
CN9[8]  （超音波後ろ）
CN10[9] （超音波左）
I2C 地磁気センサ

4chモータードライバ
gPwm[0] 
gPwm[1] 
gPwm[2] 
gPwm[3] 
gPwm[4] N.C.
gPwm[5] N.C.
*/

//モーターの最大値と最小値
#define MOTOR_MAX 85
#define MOTOR_MIN 25

//ボールセンサの定数値
/*
ボールセンサの反応下限しきい値
ボールセンサのボール距離測定時の合計値の範囲算出しきい値 遠距離-中距離
ボールセンサのボール距離測定時の合計値の範囲算出しきい値 中距離-近距離
*/
#define BALL_LOW 350
#define BALL_FAR_MEDIUM 800
#define BALL_MEDIUM_NEAR 900

//ラインセンサの定数値
/*
ラインセンサの反応下限しきい値
前ラインセンサに加算する値
左ラインセンサに加算する値
後ろラインセンサに加算する値
右ラインセンサに加算する値
*/
#define LINE_LOW 20
#define LINE_FRONTP 50
#define LINE_LEFTP 50
#define LINE_BACKP 50
#define LINE_RIGHTP 50

//姿勢制御時の各定数値
/*
地磁気センサの前方範囲しきい値
CPUの処理周期
比例定数
微分定数
*/
#define FRONT_RANGE 10
#define DELTA_T 0.01
#define KP 0.8
#define KD 0.075

//白線避け用
/*
*/

//キーパー機用
/*
後退した後に前進するときの前進回数
自動的に後ろに下がるときの後退秒数
この秒数になると自動的に後ろに下がる
左右それぞれのセンサ値の合計値の差の範囲指定
ボールセンサのボール距離測定時の合計値の範囲算出しきい値 遠距離-中距離
ボールセンサのボール距離測定時の合計値の範囲算出しきい値 中距離-近距離
*/
#define KEEPER_FRONT 10
#define KEEPER_BACK_TIME 1000
#define KEEPER_BACK_TIMELIMIT 5000
#define DIF_LR 100
#define KEEPER_BALL_FAR_MEDIUM 875
#define KEEPER_BALL_MEDIUM_NEAR 1000





//for文のループ用変数
/*
一般使用用変数1
一般使用用変数2
一般使用用変数3

ボールセンサ値比較用変数
モーター制御用変数
*/
int i;
int j;
int k;

int b;
int m;

//モーター制御用変数
/*
モーターパワー目標値 [0]~[3]
モーターパワー代入値 [0]~[3]
姿勢制御時に実際に出力するモーターパワー
*/
int m0;
int m1;
int m2;
int m3;
int inm0;
int inm1;
int inm2;
int inm3;
int difMotor;

//ボールセンサの比較に使用する変数の宣言
/*
最もセンサ値が大きいセンサのインデックス
最もセンサ値が大きいセンサのセンサ値
最もセンサが大きいセンサ値の値とその左右のセンサの値の合計
左側にある全センサのセンサ値の合計値
右側にある全センサのセンサ値の合計値
センサ値の合計値の範囲を記録する変数
しきい値より小さい値を出力したセンサの個数
比較値を記録する変数
*/
int B_highI;
int B_highV;
int B_sumV;
int B_sumL;
int B_sumR;
int B_distance;
int B_lowQ;
int B_comparingV;

//ラインセンサの比較に使用する変数の宣言
/*
値更新時の白線感知状況の記録変数
*/
int L_situation;

//姿勢制御に使用する変数の宣言
/*
地磁気センサの前方方向
現在角度値
直近過去角度値
前方方向と現在値の角度差
現在の微分係数
直近過去の微分係数
*/
int front;
int now;
int previous;
int dif;
int nowD;
int previousD;


//ボタン制御に使用する変数の宣言
/*
サブCPUのCN6のボタン BTN_B
サブCPUのCN7のボタン BTN_A
ボタンが押されているモードを記録する変数
ロボットのモードを設定する変数
Aボタンが押された回数を記録する変数
*/
int btnB;
int btnA;
int btnMode;
int mode;
int btnTime;

//サブCPUの起動状態を記録する変数
int subFlag;

//各セクションフラグセンサ
int dirOK;
int lineOK;





//あったら便利関数（使わなくてもOK）
//絶対値出力
int abs(int num) {
    if (num < 0) {
        num *= -1;
    }

    return(num);
}




//センサ値更新関数類
//ボールセンサ値の更新関数
void refreshBallSensor(void) {
    //最もセンサ値が大きいセンサを取得
    B_lowQ = 0;

    B_highI = 0;
    B_highV = gAD[B_highI];

    if (B_highV < BALL_LOW) {
        B_lowQ += 1;
    }

    for (b = 0; b < 7; b++) {
        B_comparingV = gAD[b + 1];

        if (B_comparingV < BALL_LOW) {
            B_lowQ += 1;
        }

        if (B_comparingV > B_highV) {
            B_highI = b + 1;
            B_highV = B_comparingV;
        }
    }

    //左側と右側のセンサ値合計を算出
    B_sumL = 0;
    B_sumR = 0;

    for (b = 0; b < 3; b++) {
        B_sumL += gAD[b + 1];
        B_sumR += gAD[b + 5];
    }

    //三項演算子を用いたボールの測距
    B_sumV = B_highI - 1 < 0 ? gAD[7] : gAD[B_highI - 1];
    B_sumV += B_highI + 1 > 7 ? gAD[0] : gAD[B_highI + 1];

    //測距したボールの距離を区分分け
    if (B_sumV < BALL_FAR_MEDIUM) {
        B_distance = 0;
    }
    else if (BALL_FAR_MEDIUM <= B_sumV && B_sumV < BALL_MEDIUM_NEAR) {
        B_distance = 1;
    }
    else {
        B_distance = 2;
    }

    return;
}

//ラインセンサ値の更新関数
void refreshLineSensor(void) {
    //サブCPUの変数を読み取る
    L_situation = sub_io_get_gV(1, VAR_A);

    return;
}



//地磁気センサのセットアップ関数
void dirSetup(void){
    //地磁気センサの初期化・セットアップを行う
    set_bno();

    //姿勢制御に使用する変数の初期化・値取得
    /*
    前方方向の角度を地磁気センサで取得する
    仮で直近過去角度値に現在角度値を代入
    仮で直近過去の微分係数に0を代入
    */
    front = get_bno(0);
    previous = get_bno(0);
    previousD = 0;

    return;
}



//モーター関連関数類
//モーターの目標値をモーターの最大値・最小値と照らし合わせて適正値に直す関数
void motorCheck(int motor0, int motor1, int motor2, int motor3) {

    //引数の値をグローバル変数に代入して疑似グローバル化
    m0 = motor0;
    m1 = motor1 * -1;
    m2 = motor2;
    m3 = motor3;

    //もし目標値の絶対値が最大値よりも大きいなら最大値にモーターパワーを直す
    if (m0 < MOTOR_MAX * -1) {
        m0 = MOTOR_MAX * -1;
    }
    else if (m0 > MOTOR_MAX) {
        m0 = MOTOR_MAX;
    }

    //もし目標値の絶対値が0でなく、最小値よりも小さいなら最小値にモーターパワーを直す
    if (0 < m0 && m0 < MOTOR_MIN) {
        m0 = MOTOR_MIN;
    }
    else if (MOTOR_MIN * -1 < m0 && m0 < 0) {
        m0 = MOTOR_MIN * -1;
    }

    if (m1 < MOTOR_MAX * -1) {
        m1 = MOTOR_MAX * -1;
    }
    else if (m1 > MOTOR_MAX) {
        m1 = MOTOR_MAX;
    }

    if (0 < m1 && m1 < MOTOR_MIN) {
        m1 = MOTOR_MIN;
    }
    else if (MOTOR_MIN * -1 < m1 && m1 < 0) {
        m1 = MOTOR_MIN * -1;
    }

    if (m2 < MOTOR_MAX * -1) {
        m2 = MOTOR_MAX * -1;
    }
    else if (m2 > MOTOR_MAX) {
        m2 = MOTOR_MAX;
    }

    if (0 < m2 && m2 < MOTOR_MIN) {
        m2 = MOTOR_MIN;
    }
    else if (MOTOR_MIN * -1 < m2 && m2 < 0) {
        m2 = MOTOR_MIN * -1;
    }

    if (m3 < MOTOR_MAX * -1) {
        m3 = MOTOR_MAX * -1;
    }
    else if (m3 > MOTOR_MAX) {
        m3 = MOTOR_MAX;
    }

    if (0 < m3 && m3 < MOTOR_MIN) {
        m3 = MOTOR_MIN;
    }
    else if (MOTOR_MIN * -1 < m3 && m3 < 0) {
        m3 = MOTOR_MIN * -1;
    }

    return;
}

//比例制御なしでモーターに出力する関数
void move(int motor0, int motor1, int motor2, int motor3) {

    //引数のモーターパワーを適正化する
    motorCheck(motor0, motor1, motor2, motor3);

    //モーターパワーを代入する
    gPwm[3] = m0 < 0 ? m0 * -1 : m0 | 0x80;
    gPwm[1] = m1 < 0 ? m1 * -1 : m1 | 0x80;
    gPwm[5] = m2 < 0 ? m2 * -1 : m2 | 0x80;
    gPwm[0] = m3 < 0 ? m3 * -1 : m3 | 0x80;

    //モーターを動かす
    pwm_out();

    return;
}

//モーターオフ関数
void motorOff(void) {
    gPwm[0] = 0x00;
    gPwm[1] = 0x00;
    gPwm[2] = 0x00;
    gPwm[3] = 0x00;

    pwm_out();

    return;
}

//モーター急ブレーキ関数
void motorBreak(void) {
    gPwm[0] = 0x00 | 0x80;
    gPwm[1] = 0x00 | 0x80;
    gPwm[2] = 0x00 | 0x80;
    gPwm[3] = 0x00 | 0x80;

    pwm_out();

    return;
}



//各セクション関数類
//地磁気センサセクション関数
void dirSec(void) {
    /*
    現在値を取得
    前方方向と現在値との角度差を算出する

    角度差の値を比較しやすいように調整する（-180~180）

    もし角度差が前方方向の範囲しきい値の中にないなら
        サブCPUの白色LEDを点灯させる
        微分係数を算出する
        モーターパワーを算出する

        比例制御の入っていない関数でモーターに出力する

        直近過去の角度を更新する
        直近過去の微分係数を更新する

        地磁気センサセクションフラグ変数を更新する

    地磁気センサセクションフラグ変数を更新する

    関数を終了する
    */
    now = get_bno(0);
    dif = front - now;

    if (dif <= -180) {
        dif += 360;
    }
    else if (dif >= 180) {
        dif -= 360;
    }

    if ((-1 * FRONT_RANGE <= dif && dif <= FRONT_RANGE) == FALSE) {
        sub_io_set_Led(1, 0, on);
        nowD = (now - previous) / DELTA_T;
        difMotor = (KP * dif) + (KD * (nowD - previousD));

        move(difMotor, difMotor, difMotor, difMotor);

        previous = now;
        previousD = nowD;

        dirOK = 0;

        return;
    }

    dirOK = 1;

    return;
}

void lineSec(void) {
    
    refreshLineSensor();

    switch (L_situation) {
    case 1111:
        //ラインセンサが反応していない
        lineOK = 1;
        return;

    case 9111:
        //前のみ
        motorBreak();
        move(-80, -80, 80, 80);
        break;

    case 1911:
        //左のみ
        motorBreak();
        move(80, -80, -80, 80);
        break;

    case 1191:
        //後ろのみ
        motorBreak();
        move(80, 80, -80, -80);
        break;

    case 1119:
        //右のみ
        motorBreak();
        move(-80, 80, 80, -80);
        break;

    case 9911:
        //前-左
        motorBreak();
        move(0, -80, 0, 80);
        break;

    case 1991:
        //左-後ろ
        motorBreak();
        move(80, 0, -80, 0);
        break;

    case 1199:
        //後ろ-右
        motorBreak();
        move(0, 80, 0, -80);
        break;

    case 9119:
        //右-前
        motorBreak();
        move(-80, 0, 80, 0);
        break;

    case 9991:
        //前-左-後ろ
        motorBreak();
        move(80, -80, -80, 80);
        break;

    case 1999:
        //左-後ろ-右
        motorBreak();
        move(80, 80, -80, -80);
        break;

    case 9199:
        //後ろ-右-前
        motorBreak();
        move(-80, 80, 80, -80);
        break;

    case 9919:
        //右-前-左
        motorBreak();
        move(-80, -80, 80, 80);
        break;

    case 9999:
        //すべてのラインセンサが反応している
        motorOff();
        break;
    }

    sub_io_set_Led(1, 1, on);
    lineOK = 0;

    return;
}







//メイン関数
void user_main(void) {

    //サブCPUの変数にラインセンサのしきい値を代入する
    sub_io_set_gV(1, VAR_S, LINE_LOW);

    //サブCPUの変数に各ラインセンサ比較時に加算する値を代入する
    sub_io_set_gV(1, VAR_B, LINE_FRONTP);
    sub_io_set_gV(1, VAR_C, LINE_LEFTP);
    sub_io_set_gV(1, VAR_D, LINE_BACKP);
    sub_io_set_gV(1, VAR_E, LINE_RIGHTP);

    //地磁気センサのセットアップを行う
    dirSetup();

    //無条件ループ
    while (TRUE) {

        //==================================================
        //サブCPUチェックセクション

        //サブCPUの変数を読み取る（読み取れない場合アリ）
        subFlag = sub_io_get_gV(1, VAR_Z);

        //もしサブCPUの変数が1以外（読み取れない、他の値など）なら
        if (subFlag != 1) {

            //モーターを停止させる
            motorOff();

            //無条件ループの最初に戻り続ける
            continue;
            
        }

        //==================================================


        //==================================================
        //地磁気センサセクション

        dirSec();

        if (dirOK != 1) {
            continue;
        }

        sub_io_set_Led(1, 0, off);

        //==================================================

        //==================================================
        //ラインセンサセクション

        lineSec();

        if (lineOK != 1) {
            continue;
        }

        sub_io_set_Led(1, 1, off);

        //==================================================

        //==================================================
        //ボールセンサセクション

        refreshBallSensor();

        if (B_lowQ == 8) {
            motorOff();
            set_Led(1, on);

            continue;
        }

        set_Led(1, off);

        switch (B_distance) {
        case 0:
        case 1:
            //遠距離
            //中距離
            switch (B_highI) {
            case 0:
                if ((abs(B_sumL - B_sumR) < DIF_LR) == FALSE) {
                    if (B_sumL > B_sumR) {
                        move(-50, 50, 50, -50);
                    }
                    else if (B_sumL <= B_sumR) {
                        move(50, -50, -50, 50);
                    }
                }
                else {
                    motorOff();
                }
                break;

            case 1:
            case 2:
                move(-80, 80, 80, -80);
                break;

            case 6:
            case 7:
                move(80, -80, -80, 80);
                break;

            case 3:
            case 4:
            case 5:
                motorOff();
                break;
            }
            
            break;

        case 2:
            //近距離
            switch (B_highI) {
            case 0:
                move(80, 80, -80, -80);
                break;

            case 1:
                move(-80, 80, 80, -80);
                break;

            case 2:
                move(-80, 0, 80, 0);
                break;

            case 3:
            case 5:
                move(-80, -80, 80, 80);
                break;

            case 4:
            case 7:
                move(80, -80, -80, 80);
                break;
                
            case 6:
                move(0, -80, 0, 80);
                break;
            }
            break;
        }

        //==================================================
    }
}