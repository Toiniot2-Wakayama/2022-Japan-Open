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
CN10[9] （ブザー）
I2C 4chモータードライバ

サブCPU
CN1[0]  ライン前
CN2[1]  ライン左
CN3[2]  ライン後ろ
CN4[3]  ライン右
CN5[4]  Aボタン
CN6[5]  N.C.
CN7[6]  N.C.
CN8[7]  N.C.
CN9[8]  N.C.
CN10[9] Bボタン
I2C 地磁気センサ

4chモータードライバ
gPwm[0]
gPwm[1]
gPwm[2]
gPwm[3]
gPwm[4] N.C.
gPwm[5] N.C.
*/

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//定数の宣言・定義

//モーターの最大値と最小値
#define MOTOR_MAX 85
#define MOTOR_MIN 25

//姿勢制御時の各定数値
/*
地磁気センサの前方範囲しきい値
CPUの処理周期
比例定数
微分定数
積分定数
*/
#define FRONT_RANGE 10
#define DELTA_T 0.01
#define KP 0.8
#define KD 0.01
#define KI 0.5

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//変数の宣言

//for文のループ用変数
/*
一般使用用変数1
一般使用用変数2
一般使用用変数3

モーター制御用変数
*/
int i;
int j;
int k;

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

//姿勢制御に使用する変数の宣言
/*
地磁気センサの前方方向
現在角度値
直近過去角度値
前方方向と現在値の角度差
現在の微分係数
直近過去の微分係数
積分係数
*/
int front;
int now;
int previous;
int dif;
int nowD;
int previousD;
int integral;

int p;
int i;
int d;

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//自作関数宣言・定義

//あったら便利関数（使わなくてもOK）
//絶対値
int abs(int num) {

    //もし引数の値が0より小さい（負の数）なら
    if (num < 0) {
        //引数の正負を入れ替えて更新する
        num *= -1;
    }

    //返り値を返して関数終了
    return(num);
}



//地磁気センサのセットアップ関数
int dirSetup(void) {
    //地磁気センサの初期化・セットアップを行う
    set_bno();

    wait_ms(1000);

    //姿勢制御に使用する変数の初期化・値取得
    /*
    前方方向の角度を地磁気センサで取得する
    仮で直近過去角度値に現在角度値を代入
    仮で直近過去の微分係数に0を代入
    積分係数を0で初期化
    */
    front = get_bno(0);
    previous = get_bno(0);
    previousD = 0;
    integral = 0;

    //関数終了
    return(0);
}



//モーター関連関数類
//モーターの目標値をモーターの最大値・最小値と照らし合わせて適正値に直す関数
int motorCheck(int motor0, int motor1, int motor2, int motor3) {

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

    //関数終了
    return(0);
}

//比例制御なしでモーターに出力する関数
int move(int motor0, int motor1, int motor2, int motor3) {

    //引数のモーターパワーを適正化する
    motorCheck(motor0, motor1, motor2, motor3);

    //モーターパワーを代入する
    gPwm[3] = m0 < 0 ? m0 * -1 : m0 | 0x80;
    gPwm[1] = m1 < 0 ? m1 * -1 : m1 | 0x80;
    gPwm[5] = m2 < 0 ? m2 * -1 : m2 | 0x80;
    gPwm[0] = m3 < 0 ? m3 * -1 : m3 | 0x80;

    //モーターに出力
    pwm_out();

    //関数終了
    return(0);
}

//モーターオフ関数
int motorOff(void) {

    //"reverse"のチェックを外した状態でモーターパワーをすべて0にする
    gPwm[0] = 0x00;
    gPwm[1] = 0x00;
    gPwm[2] = 0x00;
    gPwm[3] = 0x00;

    //モーターに出力
    pwm_out();

    //関数終了
    return(0);
}

//モーター急ブレーキ関数
int motorBreak(void) {

    //"reverse"のチェックを入れた状態でモーターパワーをすべて0にする
    gPwm[0] = 0x00 | 0x80;
    gPwm[1] = 0x00 | 0x80;
    gPwm[2] = 0x00 | 0x80;
    gPwm[3] = 0x00 | 0x80;

    //モーターに出力
    pwm_out();

    //関数終了
    return(0);
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@



//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//メイン関数

void user_main(void) {

    //地磁気センサのセットアップを行う
    dirSetup();

    //無条件ループ
    while (TRUE) {

        //現在値を取得
        now = get_bno(0);

        //前方方向と現在値との角度差を算出する
        dif = front - now;

        //角度差の値を比較しやすいように調整する（-180~180）
        if (dif <= -180) {
            dif += 360;
        }
        else if (dif >= 180) {
            dif -= 360;
        }

        //差の絶対値が前方範囲の中にないなら
        if ((abs(dif) < FRONT_RANGE) == FALSE) {

            //サブCPUの白色LEDを点灯させる
            sub_io_set_Led(1, 0, on);

            //微分係数を算出する
            nowD = (now - previous) / DELTA_T;

            //積分係数を算出する
            integral += (now + previous) * DELTA_T / 2;

            //定数値に応じてモーターパワーを算出する
            p = dif * KP;
            i = integral * KI;
            d = (nowD - previousD) * KD;

            difMotor = p + i + d;

            //モーターに出力する
            move(difMotor, difMotor, difMotor, difMotor);

            //直近過去の角度値を更新する
            previous = now;

            //直近過去の微分係数を更新する
            previousD = nowD;

            //無条件ループの最初に戻る
            continue;
        }

        motorOff();
    }
}