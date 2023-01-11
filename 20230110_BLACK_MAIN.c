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
I2C 6chモータードライバ

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

6chモータードライバ
gPwm[0] 右前
gPwm[1] 左後ろ
gPwm[2] N.C.
gPwm[3] 左前
gPwm[4] N.C.
gPwm[5] 右後ろ
*/

//モーターの最大値と最小値
#define MOTOR_MAX 85
#define MOTOR_MIN 25

//比例制御のループの回数
#define LINE 1
#define BALL 1

//センサの反応しきい値
/*
ラインセンサのしきい値設定時に初期値に加算する値
ボールセンサの反応下限しきい値
ボールセンサのボール距離測定時の合計値の範囲算出しきい値 遠距離-中距離
ボールセンサのボール距離測定時の合計値の範囲算出しきい値 中距離-近距離
ボールセンサの2番目に大きいセンサの値の検出可能領域設定時に使用する乗算値
地磁気センサの前方範囲しきい値
*/
#define LINE_LOW 20
#define BALL_LOW 400
#define BALL_FAR_MEDIUM 800
#define BALL_MEDIUM_NEAR 900
#define HIGH2 0.875
#define FRONT_RANGE 10

//姿勢制御時の各ゲイン値
/*
CPUの処理周期
比例定数
微分定数
*/
#define DELTA_T 0.01
#define KP 0.8
#define KD 0.075

//白線避け用
/*
同一のラインセンサが反応したときに反応を持続させる回数
白線避けのモーションを持続させる時間（デフォルトms）
白線避けのモーションを持続させる時間（ロングms）
白線避けで白線を感知したときに一旦停止する時間（ms）
コート角にロボットが来たときに角と判断する際の条件の秒数設定（ms）
*/
#define LINE_EXTEND 1
#define LINEAVOID_DEFAULT 175
#define LINEAVOID_LONG 175
#define LINESTOP 25
#define LINETIME_LR 1000

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

//@@@@@@@@@@
//距離センサ用タイマー
int timer;

//モーター制御用変数
/*
モーターパワー目標値 [0]~[3]
モーターパワー直近過去値 [0]~[3]
モーターパワー代入値 [0]~[3]
ロボットの直近運動方向の記録配列
直近の運動方向に応じたモーション延長のフラグ変数
姿勢制御時に実際に出力するモーターパワー
超音波センサを使用してコート中央へ回帰する時に実際に出力するモーターパワー
キーパー機がボールを検知しなかったときに後退するかどうかのフラグ変数
*/
int m0;
int m1;
int m2;
int m3;
int lm0;
int lm1;
int lm2;
int lm3;
int inm0;
int inm1;
int inm2;
int inm3;
int lastMotion[6];
int motionLongFlag;
int difMotor;
int disMotor;
int backStop;

//センサ値の比較に使用する変数の宣言
/*
最もセンサ値が大きいセンサのインデックス
最もセンサ値が大きいセンサのセンサ値
2番目にセンサ値が大きいセンサのインデックス
2番目にセンサ値が大きいセンサのセンサ値
2番目にセンサ値が大きいセンサの値が検出可能領域にあるかどうかのフラグ変数
最もセンサが大きいセンサ値の値とその左右のセンサの値の合計
左側にある全センサのセンサ値の合計値
右側にある全センサのセンサ値の合計値
センサ値の合計値の範囲を記録する変数
しきい値より小さい値を出力したセンサの個数
比較値を記録する変数
*/
int highIndex;
int highValue;
int highIndex2;
int highValue2;
int high2Check;
int sumValue;
int sumLeft;
int sumRight;
int sumFlag;
int lowQuantity;
int comparingValue;

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





//センサ入力値の取得関数
//関数の引数に取得センサタイプを入力すること
int refreshSensor(int sensortype, int ballfardetect) {

    switch (sensortype) {
    case 0:
        //ボールセンサの値を更新する
        //しきい値より小さい値を出力したセンサの個数を初期化
        lowQuantity = 0;

        //センサ値が最も大きいセンサのインデックスを初期化
        highIndex = 0;
        //初期化したインデックスに応じたセンサ値を仮で最も大きいセンサ値とおく
        highValue = gAD[highIndex];

        //もし仮で置いたセンサ値がしきい値よりも小さいなら
        if (highValue < BALL_LOW) {
            //しきい値より小さい値を出力したセンサの個数に加算
            lowQuantity += 1;
        }

        //左右それぞれのセンサ値の合計値を初期化
        sumLeft = 0;
        sumRight = 0;

        //CN2からCN8までの7回繰り返す
        for (b = 0; b < 7; b++) {

            //比較する値を取得
            comparingValue = gAD[b + 1];

            //もし比較中の値がしきい値より小さいなら
            if (comparingValue < BALL_LOW) {
                //しきい値より小さい値を出力したセンサの個数に加算
                lowQuantity += 1;
            }

            //もし比較中の値が最も大きいセンサ値よりも大きいなら
            if (comparingValue > highValue) {
                //センサ値が最も大きいセンサのインデックスとセンサ値を更新
                highIndex = b + 1;
                highValue = comparingValue;
            }
        }

        //左右それぞれのセンサの合計値を算出する
        for (b = 0; b < 3; b++) {
            sumLeft += gAD[b + 1];
            sumRight += gAD[b + 5];
        }

        //2番目にセンサ値が大きいセンサを検出する
        switch (highIndex) {
        case 0:
            highIndex2 = 1;
            break;

        default:
            highIndex2 = 0;
            break;
        }
        highValue2 = gAD[highIndex2];

        for (b = 0; b < 8; b++) {

            if (b == highIndex) {
                continue;
            }

            comparingValue = gAD[b];

            if (comparingValue > highValue2) {
                highIndex2 = b;
                highValue = comparingValue;
            }
        }

        //2番目にセンサ値が大きいセンサの値が検出可能領域にあるか検査
        if (highValue2 >= highValue * HIGH2) {
            high2Check = 1;
        }
        else {
            high2Check = 0;
        }

        //最もセンサ値が大きいセンサの左右のセンサ値の合計を算出する
        if (highIndex - 1 < 0) {
            sumValue = gAD[7];
        }
        else {
            sumValue = gAD[highIndex - 1];
        }

        if (highIndex + 1 > 7) {
            sumValue += gAD[0];
        }
        else {
            sumValue += gAD[highIndex + 1];
        }

        //センサ値の合計値から範囲を算出・記録する
        if (ballfardetect == 0) {
            if (sumValue < BALL_FAR_MEDIUM) {
                //ボールまでが遠いとき
                sumFlag = 0;
            }
            else if (BALL_FAR_MEDIUM <= sumValue && sumValue < BALL_MEDIUM_NEAR) {
                //ボールまでが中距離のとき
                sumFlag = 1;
            }
            else if (BALL_MEDIUM_NEAR <= sumValue) {
                //ボールまでが近いとき
                sumFlag = 2;
            }
        }
        else if (ballfardetect == 1) {
            if (sumValue < KEEPER_BALL_FAR_MEDIUM) {
                //ボールまでが遠いとき
                sumFlag = 0;
            }
            else if (KEEPER_BALL_FAR_MEDIUM <= sumValue && sumValue < KEEPER_BALL_MEDIUM_NEAR) {
                //ボールまでが中距離のとき
                sumFlag = 1;
            }
            else if (KEEPER_BALL_MEDIUM_NEAR <= sumValue) {
                //ボールまでが近いとき
                sumFlag = 2;
            }
        }

        break;

    case 1:
        //ラインセンサの値取得
        //サブCPUの変数を取得して更新する
        highIndex = sub_io_get_gV(1, VAR_A);
        highValue = sub_io_get_gV(1, VAR_B);
        lowQuantity = sub_io_get_gV(1, VAR_C);
        highIndex2 = sub_io_get_gV(1, VAR_D);
        highValue2 = sub_io_get_gV(1, VAR_E);

        break;
    }

    //関数を終了する
    return(0);
}





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

    //関数を終了する
    return(0);
}





//比例制御ありでモーターに出力する関数
int move(int motor0, int motor1, int motor2, int motor3, int loopMotor) {

    //引数の目標値を適正化する
    motorCheck(motor0, motor1, motor2, motor3);

    //引数のループ回数で比例制御のループを回す
    for (m = 0; m < loopMotor; m++) {

        //モーターパワーとして代入する値を算出する
        inm0 = ((m0 - lm0) / (loopMotor - m) + lm0);
        inm1 = ((m1 - lm1) / (loopMotor - m) + lm1);
        inm2 = ((m2 - lm2) / (loopMotor - m) + lm2);
        inm3 = ((m3 - lm3) / (loopMotor - m) + lm3);

        //モーターパワーを代入する
        gPwm[3] = inm0 < 0 ? inm0 * -1 : inm0 | 0x80;
        gPwm[1] = inm1 < 0 ? inm1 * -1 : inm1 | 0x80;
        gPwm[5] = inm2 < 0 ? inm2 * -1 : inm2 | 0x80;
        gPwm[0] = inm3 < 0 ? inm3 * -1 : inm3 | 0x80;

        //モーターを動かす
        pwm_out();

        //モーターパワーの直近過去値を更新する
        lm0 = inm0;
        lm1 = inm1;
        lm2 = inm2;
        lm3 = inm3;
    }

    //モーターパワーの直近過去値を更新する
    lm0 = m0;
    lm1 = m1;
    lm2 = m2;
    lm3 = m3;

    //関数を終了する
    return(0);
}





//比例制御なしでモーターに出力する関数
int moveNatural(int motor0, int motor1, int motor2, int motor3) {

    //引数のモーターパワーを適正化する
    motorCheck(motor0, motor1, motor2, motor3);

    //モーターパワーを代入する
    gPwm[3] = m0 < 0 ? m0 * -1 : m0 | 0x80;
    gPwm[1] = m1 < 0 ? m1 * -1 : m1 | 0x80;
    gPwm[5] = m2 < 0 ? m2 * -1 : m2 | 0x80;
    gPwm[0] = m3 < 0 ? m3 * -1 : m3 | 0x80;

    //モーターを動かす
    pwm_out();

    //モーターパワーの直近過去値を更新する
    lm0 = m0;
    lm1 = m1;
    lm2 = m2;
    lm3 = m3;

    //関数を終了する
    return(0);
}





//コート角から抜け出すための関数
int motionWrite(int motionDir) {

    if (lastMotion[0] == 2 && motionDir == 4 && gT[1] < LINETIME_LR) {
        motionLongFlag = 1;
    }
    else if (lastMotion[0] == 4 && motionDir == 2 && gT[1] < LINETIME_LR) {
        motionLongFlag = 1;
    }
    else {
        motionLongFlag = 0;
    }

    lastMotion[0] = motionDir;

    return(0);
}





//メイン関数
void user_main(void) {

    //比例制御用の直近過去値の初期化
    lm0 = 0;
    lm1 = 0;
    lm2 = 0;
    lm3 = 0;

    //直近の運動方向を初期化
    for (i = 0; i < 6; i++) {
        lastMotion[i] = 0;
    }

    //地磁気センサのセットアップを行う
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

    //サブCPUの変数にラインセンサのしきい値を代入する
    sub_io_set_gV(1, VAR_S, LINE_LOW);

    //直近の運動方向に応じたモーション延長のフラグ変数を初期化
    motionLongFlag = 0;

    //Aボタンが押された回数・ボタンが押されたモードを初期化
    btnTime = 0;
    btnMode = 0;

    //キーパー機がボールを検知しなかったときに後退するかどうかのフラグ変数を初期化する
    backStop = 0;

    //モード設定に飛ぶ
    goto modeSetting;





    //********************************************************************************
modeSetting:
    //★★★モード設定部分★★★

    //Aボタンが押された回数を記録、Bボタンでモード決定
    while (TRUE) {

        //メイン・サブCPUに付属しているすべてのLEDを点灯させる
        set_Led(0, on);
        set_Led(1, on);
        sub_io_set_Led(1, 0, on);
        sub_io_set_Led(1, 1, on);

        btnB = sub_io_get_sensor(1, 5);
        if (btnB > 550) {
            goto main;
        }

        btnA = sub_io_get_sensor(1, 6);
        if (btnA > 550 && btnMode == 0) {
            btnTime += 1;
            btnMode = 1;
        }

        if (btnA <= 550) {
            btnMode = 0;
        }
    }

    //********************************************************************************

main:
    //★★★メイン部分★★★

    set_Led(0, off);
    set_Led(1, off);
    sub_io_set_Led(1, 0, off);
    sub_io_set_Led(1, 1, off);

    switch (btnTime) {
    case 0:
    case 1:
        //攻撃モード
        //無条件ループ
        while (TRUE) {

            //============================================================

            //サブCPUが起動しているかを測定する

            //サブのCPUの変数を読み取る（読み取れない場合アリ）
            subFlag = sub_io_get_gV(1, VAR_Z);
            w
            //もしサブCPUの変数が1以外（読み取れない、他の値など）なら
            if (subFlag != 1) {

                //モーターを停止させる
                move(0, 0, 0, 0, 1);

                //無条件ループの最初に戻る
                continue;
            }

            //============================================================

            //地磁気センサを使用して姿勢制御を行う

            //現在値を取得する
            now = get_bno(0);

            //前方方向と現在値との角度差を算出する
            dif = front - now;

            //角度差の値を比較しやすいように調整する
            if (dif <= -180) {
                dif += 360;
            }
            else if (dif >= 180) {
                dif -= 360;
            }

            //もし角度差が前方方向の範囲しきい値の中にないなら
            if ((-1 * FRONT_RANGE <= dif && dif <= FRONT_RANGE) == FALSE) {

                //サブCPUの白色LEDを点灯させる
                sub_io_set_Led(1, 0, on);

                //微分係数を算出する
                nowD = (now - previous) / DELTA_T;

                //モーターパワーを算出する
                difMotor = KP * dif;
                //difMotor = KP * dif + KD * (nowD - previousD);

                //比例制御の入っていない関数でモーターに出力する
                moveNatural(difMotor, difMotor, difMotor, difMotor);

                //直近過去の角度を更新する
                previous = now;
                //直近過去の微分係数を更新する
                previousD = nowD;

                //無条件ループの最初に戻る
                continue;

            }

            //サブCPUの白色LEDを点灯させる
            sub_io_set_Led(1, 0, off);

            //============================================================

            //ラインセンサを使用して白線避けを行う

            //ラインセンサの入力値を取得してグローバル変数をラインセンサ用に更新する
            refreshSensor(1, 0);

            //------------------------------------------------------------

            //もしラインセンサのどれかが反応していたら
            if (lowQuantity != 4) {

                //サブCPUの赤色LEDを点灯させる
                sub_io_set_Led(1, 1, on);

                move(0, 0, 0, 0, BALL);
                wait_ms(LINESTOP);

                //ラインセンサが1つ反応している
                if (lowQuantity == 3) {

                    //反応しているラインセンサに応じて動く
                    switch (highIndex) {

                    case 0:
                        //前ラインセンサが反応
                        move(-80, -80, 80, 80, LINE);
                        wait_ms(LINEAVOID_DEFAULT);

                        motionWrite(1);

                        break;

                    case 1:
                        //左ラインセンサが反応
                        move(80, -80, -80, 80, LINE);
                        wait_ms(LINEAVOID_LONG);

                        motionWrite(2);
                        if (motionLongFlag == 1) {

                            clr_timer(2);

                            while (TRUE) {
                                refreshSensor(1, 0);
                                if (lowQuantity != 4) {
                                    switch (highIndex) {
                                    case 0:
                                        clr_timer(2);
                                        while (TRUE) {
                                            refreshSensor(1, 0);
                                            if (lowQuantity != 4) {
                                                switch (highIndex) {
                                                case 0:
                                                    move(0, 0, 0, 0, BALL);
                                                    wait_ms(LINESTOP);
                                                    move(-80, -80, 80, 80, LINE);
                                                    wait_ms(LINEAVOID_DEFAULT);
                                                    continue;

                                                case 1:
                                                    move(0, 0, 0, 0, BALL);
                                                    wait_ms(LINESTOP);
                                                    move(80, -80, -80, 80, LINE);
                                                    wait_ms(LINEAVOID_DEFAULT);
                                                    continue;

                                                case 2:
                                                    move(0, 0, 0, 0, BALL);
                                                    wait_ms(LINESTOP);
                                                    move(80, 80, -80, -80, LINE);
                                                    wait_ms(LINEAVOID_DEFAULT);
                                                    continue;

                                                case 3:
                                                    move(0, 0, 0, 0, BALL);
                                                    wait_ms(LINESTOP);
                                                    move(-80, 80, 80, -80, LINE);
                                                    wait_ms(LINEAVOID_DEFAULT);
                                                    continue;
                                                }
                                            }

                                            if (gT[2] > 500) {
                                                break;
                                            }

                                            move(-80, -80, 80, 80, BALL);
                                        }

                                        break;

                                    case 1:
                                        move(0, 0, 0, 0, BALL);
                                        wait_ms(LINESTOP);
                                        move(80, -80, -80, 80, LINE);
                                        wait_ms(LINEAVOID_DEFAULT);
                                        continue;

                                    case 2:
                                        move(0, 0, 0, 0, BALL);
                                        wait_ms(LINESTOP);
                                        move(80, 80, -80, -80, LINE);
                                        wait_ms(LINEAVOID_DEFAULT);
                                        continue;

                                    case 3:
                                        move(0, 0, 0, 0, BALL);
                                        wait_ms(LINESTOP);
                                        move(-80, 80, 80, -80, LINE);
                                        wait_ms(LINEAVOID_DEFAULT);
                                        continue;
                                    }
                                    break;
                                }

                                if (gT[2] > 500) {
                                    break;
                                }

                                move(80, 80, -80, -80, BALL);
                            }
                            motionLongFlag = 0;
                        }

                        clr_timer(1);

                        break;

                    case 2:
                        //後ろラインセンサが反応
                        move(80, 80, -80, -80, LINE);
                        wait_ms(LINEAVOID_DEFAULT);

                        motionWrite(3);

                        break;

                    case 3:
                        //右ラインセンサが反応
                        move(-80, 80, 80, -80, LINE);
                        wait_ms(LINEAVOID_LONG);

                        motionWrite(4);
                        if (motionLongFlag == 1) {

                            clr_timer(2);

                            while (TRUE) {
                                refreshSensor(1, 0);
                                if (lowQuantity != 4) {
                                    switch (highIndex) {
                                    case 0:
                                        clr_timer(2);
                                        while (TRUE) {
                                            refreshSensor(1, 0);
                                            if (lowQuantity != 4) {
                                                switch (highIndex) {
                                                case 0:
                                                    move(0, 0, 0, 0, BALL);
                                                    wait_ms(LINESTOP);
                                                    move(-80, -80, 80, 80, LINE);
                                                    wait_ms(LINEAVOID_DEFAULT);
                                                    continue;

                                                case 1:
                                                    move(0, 0, 0, 0, BALL);
                                                    wait_ms(LINESTOP);
                                                    move(80, -80, -80, 80, LINE);
                                                    wait_ms(LINEAVOID_DEFAULT);
                                                    continue;

                                                case 2:
                                                    move(0, 0, 0, 0, BALL);
                                                    wait_ms(LINESTOP);
                                                    move(80, 80, -80, -80, LINE);
                                                    wait_ms(LINEAVOID_DEFAULT);
                                                    continue;

                                                case 3:
                                                    move(0, 0, 0, 0, BALL);
                                                    wait_ms(LINESTOP);
                                                    move(-80, 80, 80, -80, LINE);
                                                    wait_ms(LINEAVOID_DEFAULT);
                                                    continue;
                                                }
                                            }

                                            if (gT[2] > 500) {
                                                break;
                                            }

                                            move(-80, -80, 80, 80, BALL);
                                        }

                                        break;

                                    case 1:
                                        move(0, 0, 0, 0, BALL);
                                        wait_ms(LINESTOP);
                                        move(80, -80, -80, 80, LINE);
                                        wait_ms(LINEAVOID_DEFAULT);
                                        continue;

                                    case 2:
                                        move(0, 0, 0, 0, BALL);
                                        wait_ms(LINESTOP);
                                        move(80, 80, -80, -80, LINE);
                                        wait_ms(LINEAVOID_DEFAULT);
                                        continue;

                                    case 3:
                                        move(0, 0, 0, 0, BALL);
                                        wait_ms(LINESTOP);
                                        move(-80, 80, 80, -80, LINE);
                                        wait_ms(LINEAVOID_DEFAULT);
                                        continue;
                                    }
                                    break;
                                }

                                if (gT[2] > 500) {
                                    break;
                                }

                                move(80, 80, -80, -80, BALL);
                            }
                            motionLongFlag = 0;
                        }

                        clr_timer(1);

                        break;
                    }
                }

                //------------------------------------------------------------
                /*
                //ラインセンサが2つ反応している
                else if (lowQuantity == 2) {
                    switch (highIndex) {
                    case 0:
                        switch (highIndex2) {
                        case 1:
                            move(0, -80, 0, 80, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;

                        case 3:
                            move(-80, 0, 80, 0, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;

                        default:
                            move(-80, -80, 80, 80, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;
                        }
                        break;

                    case 1:
                        switch (highIndex2) {
                        case 0:
                            move(0, -80, 0, 80, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;

                        case 2:
                            move(80, 0, -80, 0, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;

                        default:
                            move(80, -80, -80, 80, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;
                        }
                        break;

                    case 2:
                        switch (highIndex2) {
                        case 1:
                            move(80, 0, -80, 0, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;

                        case 3:
                            move(0, 80, 0, -80, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;

                        default:
                            move(80, 80, -80, -80, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;
                        }
                        break;

                    case 3:
                        switch (highIndex2) {
                        case 0:
                            move(-80, 0, 80, 0, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;

                        case 2:
                            move(0, 80, 0, -80, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;

                        default:
                            move(-80, 80, 80, -80, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;
                        }
                        break;
                    }
                }
                */
                //無条件ループの最初に戻る
                continue;
            }

            //サブCPUの赤色LEDを消灯させる
            sub_io_set_Led(1, 1, off);

            //============================================================

            //ボールセンサのセンサ値を更新する
            refreshSensor(0, 0);

            //------------------------------------------------------------

            //もしボールセンサがすべて反応していないなら
            if (lowQuantity == 8) {
                //ブレーキをかける
                move(0, 0, 0, 0, 1);

                //メインCPUの赤色LEDを点灯させる
                set_Led(1, on);

                //無条件ループの最初に戻る
                continue;
            }

            //メインCPUの赤色LEDを消灯させる
            set_Led(1, off);

            //------------------------------------------------------------
            //距離測定なしで、最も大きいセンサのインデックスに応じたボール追従

            switch (highIndex) {
            case 0:
                move(80, 80, -80, -80, BALL);
                break;

            case 1:
                move(-80, 80, 80, -80, BALL);
                break;

            case 2:
                move(-80, 0, 80, 0, BALL);
                break;

            case 3:
            case 5:
                move(-80, -80, 80, 80, BALL);
                break;

            case 4:
            case 7:
                move(80, -80, -80, 80, BALL);
                break;

            case 6:
                move(0, -80, 0, 80, BALL);
                break;
            }

            //------------------------------------------------------------
            //距離測定ありで、最も大きいセンサのインデックスに応じたボール追従
            /*
            switch (sumFlag) {
            case 0:
                //遠距離
                switch (highIndex) {
                case 0:
                    //前:前
                    move(85, 85, -85, -85, BALL);
                    break;

                case 1:
                    //左前:左前
                    move(0, 85, 0, -85, BALL);
                    break;

                case 2:
                    //左:左
                    move(-85, 85, 85, -85, BALL);
                    break;

                case 3:
                case 4:
                    //左後ろ/後ろ:左後ろ
                    move(-85, 0, 85, 0, BALL);
                    break;

                case 5:
                    //右後ろ:右後ろ
                    move(0, -85, 0, 85, BALL);
                    break;

                case 6:
                    //右:右
                    move(85, -85, -85, 85, BALL);
                    break;

                case 7:
                    //右前:右前
                    move(85, 0, -85, 0, BALL);
                    break;
                }
                break;

            case 1:
                //中距離
                switch (highIndex) {
                case 0:
                    //前:前
                    move(80, 80, -80, -80, BALL);
                    break;

                case 1:
                    //左前:左
                    move(-80, 80, 80, -80, BALL);
                    break;

                case 2:
                    //左:左後ろ
                    move(-80, 0, 80, 0, BALL);
                    break;

                case 3:
                case 5:
                    //左後ろ/右後ろ:後ろ
                    move(-80, -80, 80, 80, BALL);
                    break;

                case 4:
                case 7:
                    //後ろ/右前:右
                    move(80, -80, -80, 80, BALL);
                    break;

                case 6:
                    //右:右後ろ
                    move(0, -80, 0, 80, BALL);
                    break;
                }
                break;

            case 2:
                //近距離
                switch (highIndex) {
                case 0:
                    //前:前
                    move(80, 80, -80, -80, BALL);
                    break;

                case 1:
                    //左前:左
                    move(-80, 80, 80, -80, BALL);
                    break;

                case 2:
                case 6:
                    //左/右:後ろ
                    move(-80, -80, 80, 80, BALL);
                    break;

                case 3:
                    //左後ろ:右後ろ
                    move(0, -80, 0, 80, BALL);
                    break;

                case 4:
                case 7:
                    //後ろ/右前:右
                    move(80, -80, -80, 80, BALL);
                    break;

                case 5:
                    //右後ろ:左後ろ
                    move(-80, 0, 80, 0, BALL);
                    break;
                }
                break;
            }
            */
            //============================================================

            //無条件ループの最初に戻る
            continue;
        }
        break;

        //********************************************************************************

    case 2:
    case 3:
        //守備モード
        //無条件ループ
        while (TRUE) {

            //============================================================

            //サブCPUが起動しているかを測定する

            //サブのCPUの変数を読み取る（読み取れない場合アリ）
            subFlag = sub_io_get_gV(1, VAR_Z);

            //もしサブCPUの変数が1以外（読み取れない、他の値など）なら
            if (subFlag != 1) {

                //モーターを停止させる
                move(0, 0, 0, 0, 1);

                //無条件ループの最初に戻る
                continue;
            }

            //============================================================

            //地磁気センサを使用して姿勢制御を行う

            //現在値を取得する
            now = get_bno(0);

            //前方方向と現在値との角度差を算出する
            dif = front - now;

            //角度差の値を比較しやすいように調整する
            if (dif <= -180) {
                dif += 360;
            }
            else if (dif >= 180) {
                dif -= 360;
            }

            //もし角度差が前方方向の範囲しきい値の中にないなら
            if ((-1 * FRONT_RANGE <= dif && dif <= FRONT_RANGE) == FALSE) {

                //サブCPUの白色LEDを点灯させる
                sub_io_set_Led(1, 0, on);

                //微分係数を算出する
                nowD = (now - previous) / DELTA_T;

                //モーターパワーを算出する
                difMotor = KP * dif;
                //difMotor = KP * dif + KD * (nowD - previousD);

                //比例制御の入っていない関数でモーターに出力する
                moveNatural(difMotor, difMotor, difMotor, difMotor);

                //直近過去の角度を更新する
                previous = now;
                //直近過去の微分係数を更新する
                previousD = nowD;

                //無条件ループの最初に戻る
                continue;

            }

            //サブCPUの白色LEDを点灯させる
            sub_io_set_Led(1, 0, off);

            //============================================================

            //ラインセンサを使用して白線避けを行う

            //ラインセンサの入力値を取得してグローバル変数をラインセンサ用に更新する
            refreshSensor(1, 0);

            //------------------------------------------------------------

            //もしラインセンサのどれかが反応していたら
            if (lowQuantity != 4) {

                //サブCPUの赤色LEDを点灯させる
                sub_io_set_Led(1, 1, on);

                move(0, 0, 0, 0, BALL);
                wait_ms(LINESTOP);

                //ラインセンサが1つ反応している
                if (lowQuantity == 3) {

                    //反応しているラインセンサに応じて動く
                    switch (highIndex) {

                    case 0:
                        //前ラインセンサが反応
                        move(-80, -80, 80, 80, LINE);
                        wait_ms(LINEAVOID_DEFAULT);

                        motionWrite(1);

                        break;

                    case 1:
                        //左ラインセンサが反応
                        move(80, -80, -80, 80, LINE);
                        wait_ms(LINEAVOID_LONG);

                        motionWrite(2);
                        if (motionLongFlag == 1) {

                            clr_timer(2);

                            while (TRUE) {
                                refreshSensor(1, 0);
                                if (lowQuantity != 4) {
                                    switch (highIndex) {
                                    case 0:
                                        move(0, 0, 0, 0, BALL);
                                        wait_ms(LINESTOP);
                                        move(80, 80, -80, -80, LINE);
                                        wait_ms(LINEAVOID_DEFAULT);
                                        break;

                                    case 1:
                                        move(0, 0, 0, 0, BALL);
                                        wait_ms(LINESTOP);
                                        move(80, -80, -80, 80, LINE);
                                        wait_ms(LINEAVOID_DEFAULT);
                                        continue;

                                    case 2:
                                        move(0, 0, 0, 0, BALL);
                                        wait_ms(LINESTOP);
                                        move(80, 80, -80, -80, LINE);
                                        wait_ms(LINEAVOID_DEFAULT);
                                        continue;

                                    case 3:
                                        move(0, 0, 0, 0, BALL);
                                        wait_ms(LINESTOP);
                                        move(-80, 80, 80, -80, LINE);
                                        wait_ms(LINEAVOID_DEFAULT);
                                        continue;
                                    }
                                    break;
                                }

                                if (gT[2] > 500) {
                                    break;
                                }

                                move(80, 80, -80, -80, BALL);
                            }
                            motionLongFlag = 0;
                        }

                        clr_timer(1);

                        break;

                    case 2:
                        //後ろラインセンサが反応
                        move(80, 80, -80, -80, LINE);
                        wait_ms(LINEAVOID_DEFAULT);

                        motionWrite(3);

                        break;

                    case 3:
                        //右ラインセンサが反応
                        move(-80, 80, 80, -80, LINE);
                        wait_ms(LINEAVOID_LONG);

                        motionWrite(4);
                        if (motionLongFlag == 1) {

                            clr_timer(2);

                            while (TRUE) {
                                refreshSensor(1, 0);
                                if (lowQuantity != 4) {
                                    switch (highIndex) {
                                    case 0:
                                        move(0, 0, 0, 0, BALL);
                                        wait_ms(LINESTOP);
                                        move(80, 80, -80, -80, LINE);
                                        wait_ms(LINEAVOID_DEFAULT);
                                        break;

                                    case 1:
                                        move(0, 0, 0, 0, BALL);
                                        wait_ms(LINESTOP);
                                        move(80, -80, -80, 80, LINE);
                                        wait_ms(LINEAVOID_DEFAULT);
                                        continue;

                                    case 2:
                                        move(0, 0, 0, 0, BALL);
                                        wait_ms(LINESTOP);
                                        move(80, 80, -80, -80, LINE);
                                        wait_ms(LINEAVOID_DEFAULT);
                                        continue;

                                    case 3:
                                        move(0, 0, 0, 0, BALL);
                                        wait_ms(LINESTOP);
                                        move(-80, 80, 80, -80, LINE);
                                        wait_ms(LINEAVOID_DEFAULT);
                                        continue;
                                    }
                                    break;
                                }

                                if (gT[2] > 500) {
                                    break;
                                }

                                move(80, 80, -80, -80, BALL);
                            }
                            motionLongFlag = 0;
                        }

                        clr_timer(1);

                        break;
                    }
                }

                //------------------------------------------------------------

                //ラインセンサが2つ反応している
                else if (lowQuantity == 2) {
                    switch (highIndex) {
                    case 0:
                        switch (highIndex2) {
                        case 1:
                            move(0, -80, 0, 80, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;

                        case 3:
                            move(-80, 0, 80, 0, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;

                        default:
                            move(-80, -80, 80, 80, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;
                        }
                        break;

                    case 1:
                        switch (highIndex2) {
                        case 0:
                            move(0, -80, 0, 80, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;

                        case 2:
                            move(80, 0, -80, 0, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;

                        default:
                            move(80, -80, -80, 80, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;
                        }
                        break;

                    case 2:
                        switch (highIndex2) {
                        case 1:
                            move(80, 0, -80, 0, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;

                        case 3:
                            move(0, 80, 0, -80, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;

                        default:
                            move(80, 80, -80, -80, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;
                        }
                        break;

                    case 3:
                        switch (highIndex2) {
                        case 0:
                            move(-80, 0, 80, 0, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;

                        case 2:
                            move(0, 80, 0, -80, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;

                        default:
                            move(-80, 80, 80, -80, LINE);
                            wait_ms(LINEAVOID_DEFAULT);
                            break;
                        }
                        break;
                    }
                }

                //無条件ループの最初に戻る
                continue;
            }

            //サブCPUの赤色LEDを消灯させる
            sub_io_set_Led(1, 1, off);

            //============================================================

            //ボールセンサの入力値を取得してグローバル変数をボールセンサ用に更新する
            refreshSensor(0, 1);

            //------------------------------------------------------------

            //もしボールセンサのどれもが反応していなかったら
            if (lowQuantity == 8) {

                //------------------------------------------------------------
                //後方に下がり続けるとき
                if (backStop == 0) {

                    //無条件ループ
                    while (TRUE) {
                        //タイマー1をリセット
                        clr_timer(0);

                        //後ろに下がり続ける
                        move(-80, -80, 80, 80, BALL);

                        //ラインセンサの値を更新
                        refreshSensor(1, 0);

                        //もし1つのラインセンサが反応していたら
                        if (lowQuantity == 3) {
                            switch (highIndex) {
                                //後ろ以外のラインセンサが反応していたら白線避けの方向に動く
                            case 0:
                                move(0, 0, 0, 0, LINE);
                                move(-80, -80, 80, 80, LINE);
                                break;

                            case 1:
                                move(0, 0, 0, 0, LINE);
                                move(80, -80, -80, 80, LINE);
                                break;

                            case 3:
                                move(0, 0, 0, 0, LINE);
                                move(-80, 80, 80, -80, LINE);
                                break;

                            case 2:
                                //後ろラインセンサが反応していたらブレーキをかけてから
                                move(0, 0, 0, 0, LINE);

                                //一定回数前進する
                                for (k = 0; k < KEEPER_FRONT; k++) {
                                    move(80, 80, -80, -80, LINE);
                                }

                                //その場で停止するかどうかのフラグ変数を変更する
                                backStop = 1;

                                break;
                            }

                            //フラグ変数が変更されていたら
                            if (backStop == 1) {
                                //無条件ループから抜け出す
                                break;
                            }

                            //無条件ループの最初に戻る
                            continue;
                        }
                    }
                }

                //------------------------------------------------------------
                //その場で停止するとき
                else if (backStop == 1) {
                    //その場で停止する
                    move(0, 0, 0, 0, BALL);
                }

                //タイマー1をリセット
                clr_timer(0);

                //無条件ループの最初に戻る
                continue;
            }

            //------------------------------------------------------------

            //ボールを検知し次第、ボールを検知しなくなったときに後退するようにフラグ変数を初期化
            backStop = 0;

            //------------------------------------------------------------
            //距離測定ありで、最も大きいセンサのインデックスに応じたボール追従

            //タイマーを使用した一定時間経過後に後退するプログラム（ボール位置は干渉しない）
            if (gT[0] >= KEEPER_BACK_TIMELIMIT) {
                //後ろのラインセンサが反応するまで後退する
                while (TRUE) {
                    //ラインセンサの値を更新する

                    refreshSensor(1, 0);

                    if (lowQuantity == 3) {
                        switch (highIndex) {
                        case 0:
                            move(0, 0, 0, 0, LINE);
                            move(-80, -80, 80, 80, LINE);

                            //無条件ループの最初に戻る
                            continue;


                        case 1:
                            move(0, 0, 0, 0, LINE);
                            move(80, -80, -80, 80, LINE);

                            //無条件ループの最初に戻る
                            continue;

                        case 3:
                            move(0, 0, 0, 0, LINE);
                            move(-80, 80, 80, -80, LINE);

                            //無条件ループの最初に戻る
                            continue;

                        case 2:
                            for (k = 0; k < KEEPER_FRONT; k++) {
                                move(80, 80, -80, -80, LINE);
                            }

                            //switch文の先に進む
                            break;
                        }

                        //無条件ループから抜け出す
                        break;
                    }


                    if (gT[0] > KEEPER_BACK_TIMELIMIT + KEEPER_BACK_TIME) {
                        break;
                    }

                    move(-80, -80, 80, 80, BALL);

                    //無条件ループの最初に戻る
                    continue;
                }

                //タイマー1をリセットする
                clr_timer(0);
            }

            //------------------------------------------------------------

            switch (sumFlag) {
            case 0:
                //遠距離
            case 1:
                //中距離

                //自陣ゴールへのシュートをブロック
                //もし左右それぞれの合計値の差がマイナスなら
                if (sumLeft - sumRight < 0) {
                    //もし合計値の差が任意の範囲に入っていないなら
                    if ((-1 * DIF_LR < sumLeft - sumRight) == FALSE) {
                        //最もセンサ値が大きいセンサのインデックスに応じて動きを変える
                        switch (highIndex) {
                        case 0:
                        case 4:
                            //ボールがロボット目の前にあるときは停止
                            move(0, 0, 0, 0, BALL * 5);
                            clr_timer(0);
                            break;

                        case 3:
                        case 5:
                            //ボールがロボット後方にあるときは停止
                            move(0, 0, 0, 0, BALL * 5);
                            break;

                        default:
                            //そうでなければ合計値の大きかった右に進む
                            move(85, -85, -85, 85, BALL);
                            clr_timer(0);
                            break;
                        }

                        //無条件ループの最初に戻る
                        continue;
                    }
                    //合計値の差が任意の範囲の中なら
                    else {
                        //ロボットを停止させる
                        move(0, 0, 0, 0, BALL * 5);

                        //無条件ループの最初に戻る
                        continue;
                    }
                }

                //左右それぞれの合計値の差がプラスなら
                else if (sumLeft - sumRight > 0) {
                    //もし合計値の差が任意の範囲に入っていないなら
                    if ((sumLeft - sumRight < DIF_LR) == FALSE) {
                        //最もセンサ値が大きいセンサのインデックスに応じて動きを変える
                        switch (highIndex) {
                        case 0:
                        case 4:
                            //ボールがロボット目の前にあるときは停止
                            move(0, 0, 0, 0, BALL * 5);
                            clr_timer(0);
                            break;

                        case 3:
                        case 5:
                            //ボールがロボット後方にあるときは停止
                            move(0, 0, 0, 0, BALL * 5);
                            break;

                        default:
                            //そうでなければ合計値の大きかった左に進む
                            move(-85, 85, 85, -85, BALL);
                            clr_timer(0);
                            break;
                        }

                        //無条件ループの最初に戻る
                        continue;
                    }
                    //合計値の差が任意の範囲の中なら
                    else {
                        //ロボットを停止させる
                        move(0, 0, 0, 0, BALL * 5);

                        //無条件ループの最初に戻る
                        continue;
                    }
                }
                break;
                //------------------------------------------------------------

            case 2:
                //近距離
                //ボールを捕捉・ゴールへ

                //最もセンサ値が大きいセンサのインデックスに応じて動きを変える
                switch (highIndex) {
                case 0:
                    //前：前
                    move(80, 80, -80, -80, BALL);
                    break;

                case 1:
                    //左前：左
                    move(-80, 80, 80, -80, BALL);
                    break;

                case 2:
                case 3:
                case 5:
                case 6:
                    //左/左後ろ/右後ろ/右：後ろ
                    move(0, 0, 0, 0, BALL);
                    clr_timer(0);
                    break;

                case 4:
                case 7:
                    //後ろ/右前：右
                    move(80, -80, -80, 80, BALL);
                    break;
                }
                break;
            }

            //============================================================

            //無条件ループの最初に戻る
            continue;
        }
        break;

        //********************************************************************************

    case 4:
        //ボールの遠さチェッカーモード
        while (TRUE) {

            //============================================================

            //サブCPUが起動しているかを測定する

            //サブのCPUの変数を読み取る（読み取れない場合アリ）
            subFlag = sub_io_get_gV(1, VAR_Z);

            //もしサブCPUの変数が1以外（読み取れない、他の値など）なら
            if (subFlag != 1) {

                //メインCPUとサブCPUのすべてのLEDを消灯させる
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);

                //ボタンが押された回数を初期化する
                btnTime = 0;

                //モード設定セクションに移動する
                goto modeSetting;
            }

            //============================================================

            refreshSensor(0, 1);

            if (lowQuantity == 8) {
                set_Led(1, on);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                continue;
            }

            set_Led(1, off);

            switch (sumFlag) {
            case 0:
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;

            case 1:
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);
                break;

            case 2:
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, on);
                break;
            }

            //============================================================

            continue;
        }
        break;

        //********************************************************************************

    case 5:
        //ボールの8方位角度チェッカーモード
        while (TRUE) {

            //============================================================

            //サブCPUが起動しているかを測定する

            //サブのCPUの変数を読み取る（読み取れない場合アリ）
            subFlag = sub_io_get_gV(1, VAR_Z);

            //もしサブCPUの変数が1以外（読み取れない、他の値など）なら
            if (subFlag != 1) {

                //メインCPUとサブCPUのすべてのLEDを消灯させる
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);

                //ボタンが押された回数を初期化する
                btnTime = 0;

                //モード設定セクションに移動する
                goto modeSetting;
            }

            //============================================================

            refreshSensor(0, 1);

            if (lowQuantity == 8) {
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);
                continue;
            }

            switch (highIndex) {
            case 0:
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                break;

            case 1:
                set_Led(0, on);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                break;

            case 2:
                set_Led(0, off);
                set_Led(1, on);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                break;

            case 3:
                set_Led(0, on);
                set_Led(1, on);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                break;

            case 4:
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;

            case 5:
                set_Led(0, on);
                set_Led(1, off);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;

            case 6:
                set_Led(0, off);
                set_Led(1, on);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;

            case 7:
                set_Led(0, on);
                set_Led(1, on);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;
            }

            //============================================================

            continue;
        }
        break;

        //********************************************************************************

    case 6:
        //ボールの16方位角度チェッカーモード
        while (TRUE) {

            //============================================================

            //サブCPUが起動しているかを測定する

            //サブのCPUの変数を読み取る（読み取れない場合アリ）
            subFlag = sub_io_get_gV(1, VAR_Z);

            //もしサブCPUの変数が1以外（読み取れない、他の値など）なら
            if (subFlag != 1) {

                //メインCPUとサブCPUのすべてのLEDを消灯させる
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);

                //ボタンが押された回数を初期化する
                btnTime = 0;

                //モード設定セクションに移動する
                goto modeSetting;
            }

            //============================================================

            refreshSensor(0, 1);

            //------------------------------------------------------------

            if (lowQuantity == 8) {
                set_Led(0, on);
                set_Led(1, on);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, on);
                continue;
            }

            //------------------------------------------------------------
            //最もセンサ値が大きいセンサのインデックスのみで検知

            if (high2Check == 0) {
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);
            }

            //------------------------------------------------------------
            //最もセンサ値が大きいセンサと2番目にセンサ値が大きいセンサのインデックスで検知

            else if (high2Check == 1) {

                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
            }
            //============================================================

            continue;
        }
        break;

        //********************************************************************************

    case 7:
        //ボールの左右各合計値比較チェッカーモード
        while (TRUE) {

            //============================================================

            //サブCPUが起動しているかを測定する

            //サブのCPUの変数を読み取る（読み取れない場合アリ）
            subFlag = sub_io_get_gV(1, VAR_Z);

            //もしサブCPUの変数が1以外（読み取れない、他の値など）なら
            if (subFlag != 1) {

                //メインCPUとサブCPUのすべてのLEDを消灯させる
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);

                //ボタンが押された回数を初期化する
                btnTime = 0;

                //モード設定セクションに移動する
                goto modeSetting;
            }

            //============================================================

            refreshSensor(0, 1);

            //------------------------------------------------------------

            if (lowQuantity == 8) {
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                continue;
            }

            //------------------------------------------------------------

            /*
            左右の合計値が範囲内/外　　　　　　メイン白
            左右の合計値で左側が大きい/小さい　メイン赤
            ボールを検知していない/いる　　　　サブ白
            ボールはロボット後方にある/ない　　サブ赤
            */

            if (sumLeft - sumRight < 0) {
                if ((-1 * DIF_LR < sumLeft - sumRight) == FALSE) {
                    switch (highIndex) {
                    case 3:
                    case 4:
                    case 5:
                        //ボールは右側 & ロボット後方
                        set_Led(0, on);
                        set_Led(1, off);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, on);
                        continue;

                    default:
                        //ボールは右側
                        set_Led(0, on);
                        set_Led(1, off);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, off);
                        continue;
                    }
                }
                else {
                    //左右の合計値の差が範囲内
                    set_Led(0, off);
                    set_Led(1, off);
                    sub_io_set_Led(1, 0, off);
                    sub_io_set_Led(1, 1, off);
                    continue;
                }
            }
            else if (sumLeft - sumRight > 0) {
                if ((sumLeft - sumRight < DIF_LR) == FALSE) {
                    switch (highIndex) {
                    case 3:
                    case 4:
                    case 5:
                        //ボールは左側 & ロボット後方
                        set_Led(0, on);
                        set_Led(1, on);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, on);
                        continue;

                    default:
                        //ボールは左側
                        set_Led(0, on);
                        set_Led(1, on);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, off);
                        continue;
                    }
                }
                else {
                    //左右の合計値の差が範囲内
                    set_Led(0, off);
                    set_Led(1, off);
                    sub_io_set_Led(1, 0, off);
                    sub_io_set_Led(1, 1, off);
                    continue;
                }
            }

            //============================================================

            continue;
        }
        break;

        //********************************************************************************

    case 8:
        //ラインセンサチェッカーモード
        while (TRUE) {

            //============================================================

            //サブCPUが起動しているかを測定する

            //サブのCPUの変数を読み取る（読み取れない場合アリ）
            subFlag = sub_io_get_gV(1, VAR_Z);

            //もしサブCPUの変数が1以外（読み取れない、他の値など）なら
            if (subFlag != 1) {

                //メインCPUとサブCPUのすべてのLEDを消灯させる
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);

                //ボタンが押された回数を初期化する
                btnTime = 0;

                //モード設定セクションに移動する
                goto modeSetting;
            }

            //============================================================

            refreshSensor(1, 0);

            //------------------------------------------------------------

            if (lowQuantity == 4) {
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
            }

            //------------------------------------------------------------

            switch (lowQuantity) {
            case 3:
                //1つのラインセンサが反応
                switch (highIndex) {
                case 0:
                    set_Led(0, off);
                    set_Led(1, off);
                    sub_io_set_Led(1, 0, on);
                    sub_io_set_Led(1, 1, off);
                    break;

                case 1:
                    set_Led(0, on);
                    set_Led(1, off);
                    sub_io_set_Led(1, 0, on);
                    sub_io_set_Led(1, 1, off);
                    break;

                case 2:
                    set_Led(0, off);
                    set_Led(1, on);
                    sub_io_set_Led(1, 0, on);
                    sub_io_set_Led(1, 1, off);
                    break;

                case 3:
                    set_Led(0, on);
                    set_Led(1, on);
                    sub_io_set_Led(1, 0, on);
                    sub_io_set_Led(1, 1, off);
                    break;
                }
                break;

            case 2:
                //2つのラインセンサが反応
                switch (highIndex) {
                case 0:
                    switch (highIndex2) {
                    case 1:
                        set_Led(0, off);
                        set_Led(1, off);
                        sub_io_set_Led(1, 0, on);
                        sub_io_set_Led(1, 1, off);
                        break;

                    case 2:
                        set_Led(0, off);
                        set_Led(1, off);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, on);
                        break;

                    case 3:
                        set_Led(0, off);
                        set_Led(1, off);
                        sub_io_set_Led(1, 0, on);
                        sub_io_set_Led(1, 1, on);
                        break;
                    }
                    break;

                case 1:
                    switch (highIndex2) {
                    case 0:
                        set_Led(0, on);
                        set_Led(1, off);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, off);
                        break;

                    case 2:
                        set_Led(0, on);
                        set_Led(1, off);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, on);
                        break;

                    case 3:
                        set_Led(0, on);
                        set_Led(1, off);
                        sub_io_set_Led(1, 0, on);
                        sub_io_set_Led(1, 1, on);
                        break;
                    }
                    break;

                case 2:
                    switch (highIndex2) {
                    case 0:
                        set_Led(0, off);
                        set_Led(1, on);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, off);
                        break;

                    case 1:
                        set_Led(0, off);
                        set_Led(1, on);
                        sub_io_set_Led(1, 0, on);
                        sub_io_set_Led(1, 1, off);
                        break;

                    case 3:
                        set_Led(0, off);
                        set_Led(1, on);
                        sub_io_set_Led(1, 0, on);
                        sub_io_set_Led(1, 1, on);
                        break;
                    }
                    break;

                case 3:
                    switch (highIndex2) {
                    case 0:
                        set_Led(0, on);
                        set_Led(1, on);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, off);
                        break;

                    case 1:
                        set_Led(0, on);
                        set_Led(1, on);
                        sub_io_set_Led(1, 0, on);
                        sub_io_set_Led(1, 1, off);
                        break;

                    case 2:
                        set_Led(0, on);
                        set_Led(1, on);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, on);
                        break;
                    }
                    break;
                }
                break;

            case 1:
            case 0:
                //3つまたは4つのラインセンサが反応
                set_Led(0, on);
                set_Led(1, on);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, on);
                break;
            }

            //============================================================

            continue;
        }
        break;

        //********************************************************************************

    case 9:
        //地磁気センサチェッカーモード
        while (TRUE) {

            //============================================================

            //サブCPUが起動しているかを測定する

            //サブのCPUの変数を読み取る（読み取れない場合アリ）
            subFlag = sub_io_get_gV(1, VAR_Z);

            //もしサブCPUの変数が1以外（読み取れない、他の値など）なら
            if (subFlag != 1) {

                //メインCPUとサブCPUのすべてのLEDを消灯させる
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);

                //ボタンが押された回数を初期化する
                btnTime = 0;

                //モード設定セクションに移動する
                goto modeSetting;
            }

            //============================================================

            //現在値を取得する
            now = get_bno(0);

            //前方方向と現在値との角度差を算出する
            dif = front - now;

            //角度差の値を比較しやすいように調整する
            if (dif <= -180) {
                dif += 360;
            }
            else if (dif >= 180) {
                dif -= 360;
            }

            //------------------------------------------------------------

            //もし角度差が前方方向の範囲しきい値の中にないなら
            if ((-1 * FRONT_RANGE <= dif && dif <= FRONT_RANGE) == FALSE) {

                //サブCPUの白色LEDを点灯させる
                sub_io_set_Led(1, 0, on);

                //もし角度差が範囲しきい値よりも小さいなら
                if (dif < -1 * FRONT_RANGE) {
                    //サブCPUの赤色LEDを点灯させる
                    sub_io_set_Led(1, 1, on);
                }

                //もし角度差が範囲しきい値よりも大きいなら
                else if (FRONT_RANGE < dif) {
                    //サブCPUの赤色LEDを消灯させる
                    sub_io_set_Led(1, 1, off);
                }

                //無条件ループの最初に戻る
                continue;
            }

            //サブCPUの白色LEDと赤色LEDを消灯させる
            sub_io_set_Led(1, 0, off);
            sub_io_set_Led(1, 1, off);

            //============================================================
        }
        break;

        //********************************************************************************  

    default:
        //モーターを停止させる
        while (TRUE) {
            move(0, 0, 0, 0, 1);
        }
        break;
    }
}