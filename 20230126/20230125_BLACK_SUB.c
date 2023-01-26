#include "D_Main.h"
#include "D_I2C.h"
#include "D_SIO.h"
#include "D_EIO.h"

/*
[0]CN1  : 前
[1]CN2  : 左
[2]CN3  : 後ろ
[3]CN4  : 右
[4]CN5  : Aボタン
[5]CN6  : なし
[6]CN7  : なし
[7]CN8  : なし
[8]CN9  : なし
[9]CN10 : Bボタン
*/

//ラインセンサの比較に使用する変数の宣言
/*
ラインセンサしきい値
前ラインセンサ用フラグ変数
左ラインセンサ用フラグ変数
後ろラインセンサ用フラグ変数
右ラインセンサ用フラグ変数
値更新時の白線感知状況の記録変数
*/
int LINE_LOW;
int L_front;
int L_frontP;
int L_left;
int L_leftP;
int L_back;
int L_backP;
int L_right;
int L_rightP;
int L_situation;


//メイン関数
void user_main(void) {

    //サブCPUの状態（起動状態）をグローバル化されたフラグ変数に記録
    gV[VAR_Z] = 1;

    //無条件ループ
    while (TRUE) {

        //しきい値を設定する
        LINE_LOW = gV[VAR_S];

        L_frontP = gV[VAR_B];
        L_leftP = gV[VAR_C];
        L_backP = gV[VAR_D];
        L_rightP = gV[VAR_E];

        //==================================================
        //前ラインセンサ

        if (gAD[0] + L_frontP < LINE_LOW) {
            L_front = 1;
        }
        else {
            L_front = 9;
        }

        //==================================================

        //==================================================
        //左ラインセンサ

        if (gAD[1] + L_leftP < LINE_LOW) {
            L_left = 1;
        }
        else {
            L_left = 9;
        }

        //==================================================

        //==================================================
        //後ろラインセンサ

        if (gAD[2] + L_backP < LINE_LOW) {
            L_back = 1;
        }
        else {
            L_back = 9;
        }

        //==================================================

        //==================================================
        //右ラインセンサ

        if (gAD[3] + L_rightP < LINE_LOW) {
            L_right = 1;
        }
        else {
            L_right = 9;
        }

        //==================================================

        L_situation = 1000 * L_front + 100 * L_left + 10 * L_back + 1 * L_right;

        gV[VAR_A] = L_situation;
    }
}