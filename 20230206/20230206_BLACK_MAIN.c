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
#define LINE_LOW 575
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
P制御かPD制御かのフラグ変数（P:0、PD:1）
*/
#define FRONT_RANGE 10
#define DELTA_T 0.01
#define KP 0.8
#define KD 0.075
#define P_PD 0

//白線避け用
/*
*/

//キーパー機用
/*
コート後方に戻るまでの制限時間（ms）
*/
#define KEEPER_BACK_TIMELIMIT 5000

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//変数の宣言

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
センサ値の合計値の範囲を記録する変数
しきい値より小さい値を出力したセンサの個数
比較値を記録する変数
*/
int B_highI;
int B_highV;
int B_sumV;
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
サブCPUのCN5のセンサ値（ボタンA）
サブCPUのCN10のセンサ値（ボタンB）
ボタンAが押されているモードを記録する変数
ボタンBが押されているモードを記録する変数
ロボットのモードを設定する変数
ボタンAが押された回数を記録する変数
ボタンBが押された回数を記録する変数
*/
int btnB;
int btnA;
int btnModeA;
int btnModeB;
int btnTimeA;
int btnTimeB;

//サブCPUの起動状態を記録する変数
int subFlag;

//各セクションフラグ変数
int dirOK;
int lineOK;

//チェッカー用変数
int kp;
int kd;

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



//センサ値更新関数類
//ボールセンサ値の更新関数
int refreshBallSensor(void) {

    //最もセンサ値が大きいセンサを取得
    //しきい値より低い値を出力したセンサの個数を初期化
    B_lowQ = 0;

    //仮で最も大きいセンサ値を出力したセンサのインデックスを初期化
    B_highI = 0;

    //仮で最も大きいセンサ値を出力したセンサのセンサ値を取得
    B_highV = gAD[B_highI];

    //仮の最も大きいセンサ値がしきい値より小さいか検査
    if (B_highV < BALL_LOW) {
        B_lowQ += 1;
    }

    //CN2からCN7まで7回繰り返す
    for (b = 0; b < 7; b++) {

        //比較する値を取得
        B_comparingV = gAD[b + 1];

        //比較する値がしきい値より小さいか検査
        if (B_comparingV < BALL_LOW) {
            B_lowQ += 1;
        }

        //比較する値が最も大きいセンサ値より大きいか検査
        if (B_comparingV > B_highV) {

            //最も大きいセンサ値を出力したセンサのインデックスを更新
            B_highI = b + 1;

            //最も大きいセンサ値を出力したセンサのセンサ値を更新
            B_highV = B_comparingV;
        }
    }

    //三項演算子を用いたボールの測距
    //最も大きいセンサ値を出力したセンサのインデックスから1引いた数が0より小さいならCN8を、でなければ1つ前のインデックスのセンサ値を取得
    B_sumV = B_highI - 1 < 0 ? gAD[7] : gAD[B_highI - 1];
    //最も大きいセンサ値を出力したセンサのインデックスに1足した数が7より大きいならCN1を、でなければ1つ後のインデックスのセンサ値を加算
    B_sumV += B_highI + 1 > 7 ? gAD[0] : gAD[B_highI + 1];

    //測距したボールの距離を区分分け
    //ボールまでの距離が遠距離-中距離のしきい値より小さいか検査
    if (B_sumV < BALL_FAR_MEDIUM) {

        //ボールは遠距離
        B_distance = 0;
    }
    //ボールまでの距離が遠距離-中距離のしきい値と中距離-近距離のしきい値の間にあるか検査
    else if (BALL_FAR_MEDIUM <= B_sumV && B_sumV < BALL_MEDIUM_NEAR) {

        //ボールは中距離
        B_distance = 1;
    }
    else {
        
        //ボールは近距離
        B_distance = 2;
    }

    //関数終了
    return(0);
}

//ラインセンサ値の更新関数
int refreshLineSensor(void) {

    //サブCPUの変数を読み取ってメイン側のグローバル変数を更新する
    L_situation = sub_io_get_gV(1, VAR_A);

    //関数終了
    return(0);
}

//ボタンA・ボタンBの状態更新関数
int refreshButton(void) {

    //サブCPUのセンサ値を直接取得
    //CN5の値をbtnAに代入
    btnA = sub_io_get_sensor(1, 4);
    //CN10の値をbtnBに代入
    btnB = sub_io_get_sensor(1, 9);

    //関数終了
    return(0);
}


//地磁気センサのセットアップ関数
int dirSetup(void){
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



//各セクション関数類
//地磁気センサセクション関数
int dirSec(void) {

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

        //定数値に応じてモーターパワーを算出する
        if (P_PD == 0) {
            difMotor = (KP * dif);
        }
        else if (P_PD == 1) {
            difMotor = (KP * dif) + (KD * (nowD - previousD));
        }

        //モーターに出力する
        move(difMotor, difMotor, difMotor, difMotor);

        //直近過去の角度値を更新する
        previous = now;

        //直近過去の微分係数を更新する
        previousD = nowD;

        //フラグ変数を下ろす
        dirOK = 0;
        
        //関数終了
        return(0);
    }

    //フラグ変数を立てる
    dirOK = 1;

    //関数終了
    return(0);
}

int lineSec(void) {
    
    //白線感知状況を更新
    refreshLineSensor();

    //白線感知状況の変数の値によって動きを変える]
    /*
    〇〇〇〇
    ｜｜｜｜
    ｜｜｜ーー 右ラインセンサ（1:未反応、9:反応）
    ｜｜ーーー 後ろラインセンサ（1:未反応、9:反応）
    ｜ーーーー 左ラインセンサ（1:未反応、9:反応）
    ーーーーー 前ラインセンサ（1:未反応、9:反応）
    */

    switch (L_situation) {
    case 1111:
        //ラインセンサが反応していない

        //フラグ変数を立てる
        lineOK = 1;

        //関数終了
        return(0);

    case 9111:
        //前のみ

        //急ブレーキ
        motorBreak();

        //後ろに動く
        move(-80, -80, 80, 80);

        break;

    case 1911:
        //左のみ

        //急ブレーキ
        motorBreak();

        //右に動く
        move(80, -80, -80, 80);

        break;

    case 1191:
        //後ろのみ

        //急ブレーキ
        motorBreak();
        
        //前に動く
        move(80, 80, -80, -80);

        break;

    case 1119:
        //右のみ

        //急ブレーキ
        motorBreak();

        //左に動く
        move(-80, 80, 80, -80);

        break;

    case 9911:
        //前-左

        //急ブレーキ
        motorBreak();

        //右後ろに動く
        move(0, -80, 0, 80);

        break;

    case 1991:
        //左-後ろ

        //急ブレーキ
        motorBreak();

        //右前に動く
        move(80, 0, -80, 0);

        break;

    case 1199:
        //後ろ-右

        //急ブレーキ
        motorBreak();

        //左前に動く
        move(0, 80, 0, -80);

        break;

    case 9119:
        //右-前

        //急ブレーキ
        motorBreak();

        //左後ろに動く
        move(-80, 0, 80, 0);
        
        break;

    case 9991:
        //前-左-後ろ

        //急ブレーキ
        motorBreak();

        //右に動く
        move(80, -80, -80, 80);
        
        break;

    case 1999:
        //左-後ろ-右

        //急ブレーキ
        motorBreak();

        //前に動く
        move(80, 80, -80, -80);

        break;

    case 9199:
        //後ろ-右-前

        //急ブレーキ
        motorBreak();

        //左に動く
        move(-80, 80, 80, -80);

        break;

    case 9919:
        //右-前-左

        //急ブレーキ
        motorBreak();
        
        //後ろに動く
        move(-80, -80, 80, 80);

        break;

    case 9999:
        //すべてのラインセンサが反応している

        //モーターオフ
        motorOff();

        break;
    }

    //サブCPUの赤色LEDを点灯させる
    sub_io_set_Led(1, 1, on);

    //フラグ変数を下ろす
    lineOK = 0;

    //関数終了
    return(0);

}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@



//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
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

    //Aボタン・Bボタンのモード初期化
    btnTimeA = 0;
    btnTimeB = 0;
    btnModeA = 0;
    btnModeB = 0;
    
    //モード選択
    //仮の無条件ループを回す

    //メインCPUとサブCPUのすべてのLEDを点灯させる
    set_Led(0, on);
    set_Led(1, on);
    sub_io_set_Led(1, 0, on);
    sub_io_set_Led(1, 1, on);

    while (TRUE) {

        //ボタンの状態変数を更新
        refreshButton();

        //ボタンAのモードが押されていない状態でボタンAが押されたら
        if (btnA > 500 && btnModeA == 0) {
            
            //ボタンAが押された回数を1加算
            btnTimeA += 1;

            //ボタンAのモードを押されている状態に更新
            btnModeA = 1;
        }
        
        //ボタンAが押されていなかったら
        else if (btnA < 500) {

            //ボタンAのモードを押されていない状態に更新
            btnModeA = 0;
        }

        //ボタンBのモードが押されていない状態でボタンBが押されたら
        if (btnB > 500 && btnModeB == 0) {

            //ボタンBのモードを押されている状態に更新
            btnModeB = 1;
        }

        //ボタンBが押されておらず、ボタンBのモードが押されていない状態なら
        else if (btnB < 500 && btnModeB == 1) {

            //仮の無条件ループを抜け出す
            break;
        }
    }

    //メインCPUとサブCPUのすべてのLEDを消灯させる
    set_Led(0, off);
    set_Led(1, off);
    sub_io_set_Led(1, 0, off);
    sub_io_set_Led(1, 1, off);

    //ボタンAが押された回数によって動き方を変える
    switch (btnTimeA) {
    case 0:
    case 1:
        //攻撃モード
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

            switch (B_distance) {
            case 0:
                switch (B_highI) {
                case 0:
                    move(80, 80, -80, -80);
                    break;

                case 1:
                    move(0, 80, 0, -80);
                    break;

                case 2:
                    move(-80, 80, 80, -80);
                    break;

                case 3:
                case 4:
                    move(-80, 0, 80, 0);
                    break;

                case 5:
                    move(0, -80, 0, 80);
                    break;

                case 6:
                    move(80, -80, -80, 80);
                    break;

                case 7:
                    move(80, 0, -80, 0);
                    break;
                }
                break;

            case 1:
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

            case 2:
                switch (B_highI) {
                case 0:
                    move(80, 80, -80, -80);
                    break;

                case 1:
                    move(-80, 0, 80, 0);
                    break;

                case 2:
                case 6:
                    move(-80, -80, 80, 80);
                    break;

                case 3:
                    move(0, -80, 0, 80);
                    break;

                case 4:
                    move(80, -80, -80, 80);
                    break;

                case 5:
                    move(-80, 0, 80, 0);
                    break;

                case 7:
                    move(0, -80, 0, 80);
                    break;
                }
                break;
            }

            //==================================================
        }
        break;

    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    case 2:
    case 3:
        //守備モード

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
                    motorOff();
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

                clr_timer(T1);
                
                break;

            case 2:
                //近距離
                
                //制限時間を過ぎていたら
                if (T1 > KEEPER_BACK_TIMELIMIT) {
                    //後ろラインセンサが反応するまで後ろに下がる
                    while (TRUE) {
                        //ラインセンサの値を更新
                        refreshLineSensor();

                        //白線感知状況の記録変数に応じて動きを変える
                        switch (L_situation) {
                        case 1191:
                            //後退をやめる
                            break;

                        case 1911:
                            move(0, -80, 0, 80);
                            //無条件ループの最初に戻る
                            continue;

                        case 1119:
                            move(-80, 0, 80, 0);
                            //無条件ループの最初に戻る
                            continue;
                            
                        default:
                            move(-80, -80, 80, 80);
                            //無条件ループの最初に戻る
                            continue;
                        }

                        //無条件ループから抜け出す
                        break;
                    }

                    //1000ms前進する
                    move(80, 80, -80, -80);
                    wait_ms(1000);

                    //無条件ループの最初に戻る
                    continue;
                }

                //制限時間以内ならボールを追従する
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
        break;

    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    //ここからチェッカーモード

    case 4:
        //ボール距離チェッカー

        //無条件ループ
        while (TRUE) {

            refreshBallSensor();

            if (B_lowQ == 8) {

                //ボールを検知していない
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);

                //無条件ループの最初に戻る
                continue;
            }

            switch (B_distance) {
            case 0:
                //遠距離
                set_Led(0, on);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                break;

            case 1:
                //中距離
                set_Led(0, off);
                set_Led(1, on);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                break;

            case 2:
                //近距離
                set_Led(0, on);
                set_Led(1, on);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                break;
            }
        }
        break;

    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    case 5:
        //ボール8方位角度チェッカー

        //無条件ループ
        while (TRUE) {

            refreshBallSensor();

            if(B_lowQ == 8){

                //ボールを検知していない
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);

                //無条件ループの最初に戻る
                continue;
            }

            switch (B_highI) {
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
        }
        break;

    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    case 6:
        //地磁気センサゲイン変化チェッカー

        btnTimeA = 0;
        btnTimeB = 0;

        //無条件ループ
        while (TRUE) {

            //============================================================

            refreshButton();

            if (btnA > 500 && btnModeA == 0) {

                btnTimeA += 1;

                btnModeA = 1;
            }

            else if (btnA < 500) {

                btnModeA = 0;
            }

            if (btnB > 500 && btnModeB == 0) {

                btnTimeB += 1;

                btnModeB = 1;
            }

            else if (btnB < 500) {

                btnModeB = 0;
            }

            kp = KP + 0.05 * btnTimeA;
            kd = KD + 0.05 * btnTimeB;

            //============================================================

            now = get_bno(0);

            dif = front - now;

            if (dif <= -180) {
                dif += 360;
            }
            else if (dif >= 180) {
                dif -= 360;
            }

            if ((abs(dif) < FRONT_RANGE) == FALSE) {

                sub_io_set_Led(1, 0, on);

                nowD = (now - previous) / DELTA_T;

                if (P_PD == 0) {
                    difMotor = (KP * dif);
                }
                else if (P_PD == 1) {
                    difMotor = (kp * dif) + (kd * (nowD - previousD));
                }

                move(difMotor, difMotor, difMotor, difMotor);

                previous = now;

                previousD = nowD;

                continue;
            }

            motorOff();

            sub_io_set_Led(1, 0, off);

            //============================================================
        }
        break;

    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    case 7:
        //ボール左右各合計値比較チェッカー
        //（なし）
        break;

    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    case 8:
        //ラインセンサチェッカー

        //無条件ループ
        while (TRUE) {

            refreshLineSensor();

            switch (L_situation) {
            case 1111:
                //白線を検知していない
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                break;

            case 9111:
                //前のみ
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);
                break;

            case 1911:
                //左のみ
                set_Led(0, on);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);
                break;

            case 1191:
                //後ろのみ
                set_Led(0, off);
                set_Led(1, on);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);
                break;

            case 1119:
                //右のみ
                set_Led(0, on);
                set_Led(1, on);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);
                break;

            case 9911:
                //前・左
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;

            case 1991:
                //左・後ろ
                set_Led(0, on);
                set_Led(1, off);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;

            case 1199:
                //後ろ・右
                set_Led(0, off);
                set_Led(1, on);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;

            case 9119:
                //右・前
                set_Led(0, on);
                set_Led(1, on);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;

            case 9991:
            case 1999:
            case 9199:
            case 9919:
            case 9999:
                //3つまたは4つのラインセンサが白線を検知している
                set_Led(0, on);
                set_Led(1, on);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, on);
                break;
            }
        }
        break;

    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    case 9:
        //地磁気センサチェッカー

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

                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, on);

                //前方方向と現在値との角度差がマイナスなら
                if (dif < 0) {
                    set_Led(0, on);
                    set_Led(1, off);
                }
                //前方方向と現在値との角度差がプラスなら
                else if (dif > 0) {
                    set_Led(0, off);
                    set_Led(1, on);
                }

                //無条件ループの最初に戻る
                continue;
            }

            set_Led(0, off);
            set_Led(1, off);
            sub_io_set_Led(1, 0, off);
            sub_io_set_Led(1, 1, off);

        }
        break;

    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    default:
        //ロボットは無条件に停止

        //無条件ループ
        while (TRUE) {

            //モーターオフ
            motorOff();
        }
        break;
    }
}

/*
プログラミングのヒント

● 一週間後の自分は赤の他人だと思ってコメントを残しておくべし

・コメント // はプログラム最終行に来てはならない
・自作関数の返り値型をvoidにしてはならない（intがオススメ）
・math.hに含まれる関数（三角関数など）を使うことはできそう
・ビルドエラーが発生したときはエラー発生行の直前行でセミコロン ; が抜けていないかチェック
・ビルドエラーが発生したときにエラー発生行がプログラム最終行ならコメントアウトの終わりが残っていたり逆に終わりがなかったりしないかチェック
・ビルドエラーが発生したときに文法エラーが見当たらない場合はC-styleを再起動してもう一度ビルドしてみる

*/