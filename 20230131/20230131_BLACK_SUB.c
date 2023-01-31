#include "D_Main.h"
#include "D_I2C.h"
#include "D_SIO.h"
#include "D_EIO.h"

/*
[0]CN1  : �O
[1]CN2  : ��
[2]CN3  : ���
[3]CN4  : �E
[4]CN5  : A�{�^��
[5]CN6  : �Ȃ�
[6]CN7  : �Ȃ�
[7]CN8  : �Ȃ�
[8]CN9  : �Ȃ�
[9]CN10 : B�{�^��
*/

//���C���Z���T�̔�r�Ɏg�p����ϐ��̐錾
/*
���C���Z���T�������l
�O���C���Z���T�p�t���O�ϐ�
�����C���Z���T�p�t���O�ϐ�
��냉�C���Z���T�p�t���O�ϐ�
�E���C���Z���T�p�t���O�ϐ�
�l�X�V���̔������m�󋵂̋L�^�ϐ�
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


//���C���֐�
void user_main(void) {

    //�T�uCPU�̏�ԁi�N����ԁj���O���[�o�������ꂽ�t���O�ϐ��ɋL�^
    gV[VAR_Z] = 1;

    //���������[�v
    while (TRUE) {

        //�������l��ݒ肷��
        LINE_LOW = gV[VAR_S];

        L_frontP = gV[VAR_B];
        L_leftP = gV[VAR_C];
        L_backP = gV[VAR_D];
        L_rightP = gV[VAR_E];

        //==================================================
        //�O���C���Z���T

        if (gAD[0] + L_frontP < LINE_LOW) {
            L_front = 1;
        }
        else {
            L_front = 9;
        }

        //==================================================

        //==================================================
        //�����C���Z���T

        if (gAD[1] + L_leftP < LINE_LOW) {
            L_left = 1;
        }
        else {
            L_left = 9;
        }

        //==================================================

        //==================================================
        //��냉�C���Z���T

        if (gAD[2] + L_backP < LINE_LOW) {
            L_back = 1;
        }
        else {
            L_back = 9;
        }

        //==================================================

        //==================================================
        //�E���C���Z���T

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