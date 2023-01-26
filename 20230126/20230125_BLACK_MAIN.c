#include "D_Main.h"
#include "D_I2C.h"
#include "D_SIO.h"
#include "D_EIO.h"

/*
���C��CPU
CN1[0]  �{�[���O
CN2[1]  �{�[�����O
CN3[2]  �{�[����
CN4[3]  �{�[�������
CN5[4]  �{�[�����
CN6[5]  �{�[���E���
CN7[6]  �{�[���E
CN8[7]  �{�[���E�O
CN9[8]  N.C.
CN10[9] N.C.
I2C 4ch���[�^�[�h���C�o

�T�uCPU
CN1[0]  ���C���O
CN2[1]  ���C����
CN3[2]  ���C�����
CN4[3]  ���C���E
CN5[4]  N.C.
CN6[5]  B�{�^���i����j
CN7[6]  A�{�^���i�I���j
CN8[7]  �i�����g�E�j
CN9[8]  �i�����g���j
CN10[9] �i�����g���j
I2C �n���C�Z���T

4ch���[�^�[�h���C�o
gPwm[0] 
gPwm[1] 
gPwm[2] 
gPwm[3] 
gPwm[4] N.C.
gPwm[5] N.C.
*/

//���[�^�[�̍ő�l�ƍŏ��l
#define MOTOR_MAX 85
#define MOTOR_MIN 25

//�{�[���Z���T�̒萔�l
/*
�{�[���Z���T�̔��������������l
�{�[���Z���T�̃{�[���������莞�̍��v�l�͈͎̔Z�o�������l ������-������
�{�[���Z���T�̃{�[���������莞�̍��v�l�͈͎̔Z�o�������l ������-�ߋ���
*/
#define BALL_LOW 350
#define BALL_FAR_MEDIUM 800
#define BALL_MEDIUM_NEAR 900

//���C���Z���T�̒萔�l
/*
���C���Z���T�̔��������������l
�O���C���Z���T�ɉ��Z����l
�����C���Z���T�ɉ��Z����l
��냉�C���Z���T�ɉ��Z����l
�E���C���Z���T�ɉ��Z����l
*/
#define LINE_LOW 20
#define LINE_FRONTP 50
#define LINE_LEFTP 50
#define LINE_BACKP 50
#define LINE_RIGHTP 50

//�p�����䎞�̊e�萔�l
/*
�n���C�Z���T�̑O���͈͂������l
CPU�̏�������
���萔
�����萔
*/
#define FRONT_RANGE 10
#define DELTA_T 0.01
#define KP 0.8
#define KD 0.075

//���������p
/*
*/

//�L�[�p�[�@�p
/*
��ނ�����ɑO�i����Ƃ��̑O�i��
�����I�Ɍ��ɉ�����Ƃ��̌�ޕb��
���̕b���ɂȂ�Ǝ����I�Ɍ��ɉ�����
���E���ꂼ��̃Z���T�l�̍��v�l�̍��͈͎̔w��
�{�[���Z���T�̃{�[���������莞�̍��v�l�͈͎̔Z�o�������l ������-������
�{�[���Z���T�̃{�[���������莞�̍��v�l�͈͎̔Z�o�������l ������-�ߋ���
*/
#define KEEPER_FRONT 10
#define KEEPER_BACK_TIME 1000
#define KEEPER_BACK_TIMELIMIT 5000
#define DIF_LR 100
#define KEEPER_BALL_FAR_MEDIUM 875
#define KEEPER_BALL_MEDIUM_NEAR 1000





//for���̃��[�v�p�ϐ�
/*
��ʎg�p�p�ϐ�1
��ʎg�p�p�ϐ�2
��ʎg�p�p�ϐ�3

�{�[���Z���T�l��r�p�ϐ�
���[�^�[����p�ϐ�
*/
int i;
int j;
int k;

int b;
int m;

//���[�^�[����p�ϐ�
/*
���[�^�[�p���[�ڕW�l [0]~[3]
���[�^�[�p���[����l [0]~[3]
�p�����䎞�Ɏ��ۂɏo�͂��郂�[�^�[�p���[
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

//�{�[���Z���T�̔�r�Ɏg�p����ϐ��̐錾
/*
�ł��Z���T�l���傫���Z���T�̃C���f�b�N�X
�ł��Z���T�l���傫���Z���T�̃Z���T�l
�ł��Z���T���傫���Z���T�l�̒l�Ƃ��̍��E�̃Z���T�̒l�̍��v
�����ɂ���S�Z���T�̃Z���T�l�̍��v�l
�E���ɂ���S�Z���T�̃Z���T�l�̍��v�l
�Z���T�l�̍��v�l�͈̔͂��L�^����ϐ�
�������l��菬�����l���o�͂����Z���T�̌�
��r�l���L�^����ϐ�
*/
int B_highI;
int B_highV;
int B_sumV;
int B_sumL;
int B_sumR;
int B_distance;
int B_lowQ;
int B_comparingV;

//���C���Z���T�̔�r�Ɏg�p����ϐ��̐錾
/*
�l�X�V���̔������m�󋵂̋L�^�ϐ�
*/
int L_situation;

//�p������Ɏg�p����ϐ��̐錾
/*
�n���C�Z���T�̑O������
���݊p�x�l
���߉ߋ��p�x�l
�O�������ƌ��ݒl�̊p�x��
���݂̔����W��
���߉ߋ��̔����W��
*/
int front;
int now;
int previous;
int dif;
int nowD;
int previousD;


//�{�^������Ɏg�p����ϐ��̐錾
/*
�T�uCPU��CN6�̃{�^�� BTN_B
�T�uCPU��CN7�̃{�^�� BTN_A
�{�^����������Ă��郂�[�h���L�^����ϐ�
���{�b�g�̃��[�h��ݒ肷��ϐ�
A�{�^���������ꂽ�񐔂��L�^����ϐ�
*/
int btnB;
int btnA;
int btnMode;
int mode;
int btnTime;

//�T�uCPU�̋N����Ԃ��L�^����ϐ�
int subFlag;

//�e�Z�N�V�����t���O�Z���T
int dirOK;
int lineOK;





//��������֗��֐��i�g��Ȃ��Ă�OK�j
//��Βl�o��
int abs(int num) {
    if (num < 0) {
        num *= -1;
    }

    return(num);
}




//�Z���T�l�X�V�֐���
//�{�[���Z���T�l�̍X�V�֐�
void refreshBallSensor(void) {
    //�ł��Z���T�l���傫���Z���T���擾
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

    //�����ƉE���̃Z���T�l���v���Z�o
    B_sumL = 0;
    B_sumR = 0;

    for (b = 0; b < 3; b++) {
        B_sumL += gAD[b + 1];
        B_sumR += gAD[b + 5];
    }

    //�O�����Z�q��p�����{�[���̑���
    B_sumV = B_highI - 1 < 0 ? gAD[7] : gAD[B_highI - 1];
    B_sumV += B_highI + 1 > 7 ? gAD[0] : gAD[B_highI + 1];

    //���������{�[���̋������敪����
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

//���C���Z���T�l�̍X�V�֐�
void refreshLineSensor(void) {
    //�T�uCPU�̕ϐ���ǂݎ��
    L_situation = sub_io_get_gV(1, VAR_A);

    return;
}



//�n���C�Z���T�̃Z�b�g�A�b�v�֐�
void dirSetup(void){
    //�n���C�Z���T�̏������E�Z�b�g�A�b�v���s��
    set_bno();

    //�p������Ɏg�p����ϐ��̏������E�l�擾
    /*
    �O�������̊p�x��n���C�Z���T�Ŏ擾����
    ���Œ��߉ߋ��p�x�l�Ɍ��݊p�x�l����
    ���Œ��߉ߋ��̔����W����0����
    */
    front = get_bno(0);
    previous = get_bno(0);
    previousD = 0;

    return;
}



//���[�^�[�֘A�֐���
//���[�^�[�̖ڕW�l�����[�^�[�̍ő�l�E�ŏ��l�ƏƂ炵���킹�ēK���l�ɒ����֐�
void motorCheck(int motor0, int motor1, int motor2, int motor3) {

    //�����̒l���O���[�o���ϐ��ɑ�����ċ^���O���[�o����
    m0 = motor0;
    m1 = motor1 * -1;
    m2 = motor2;
    m3 = motor3;

    //�����ڕW�l�̐�Βl���ő�l�����傫���Ȃ�ő�l�Ƀ��[�^�[�p���[�𒼂�
    if (m0 < MOTOR_MAX * -1) {
        m0 = MOTOR_MAX * -1;
    }
    else if (m0 > MOTOR_MAX) {
        m0 = MOTOR_MAX;
    }

    //�����ڕW�l�̐�Βl��0�łȂ��A�ŏ��l�����������Ȃ�ŏ��l�Ƀ��[�^�[�p���[�𒼂�
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

//��ᐧ��Ȃ��Ń��[�^�[�ɏo�͂���֐�
void move(int motor0, int motor1, int motor2, int motor3) {

    //�����̃��[�^�[�p���[��K��������
    motorCheck(motor0, motor1, motor2, motor3);

    //���[�^�[�p���[��������
    gPwm[3] = m0 < 0 ? m0 * -1 : m0 | 0x80;
    gPwm[1] = m1 < 0 ? m1 * -1 : m1 | 0x80;
    gPwm[5] = m2 < 0 ? m2 * -1 : m2 | 0x80;
    gPwm[0] = m3 < 0 ? m3 * -1 : m3 | 0x80;

    //���[�^�[�𓮂���
    pwm_out();

    return;
}

//���[�^�[�I�t�֐�
void motorOff(void) {
    gPwm[0] = 0x00;
    gPwm[1] = 0x00;
    gPwm[2] = 0x00;
    gPwm[3] = 0x00;

    pwm_out();

    return;
}

//���[�^�[�}�u���[�L�֐�
void motorBreak(void) {
    gPwm[0] = 0x00 | 0x80;
    gPwm[1] = 0x00 | 0x80;
    gPwm[2] = 0x00 | 0x80;
    gPwm[3] = 0x00 | 0x80;

    pwm_out();

    return;
}



//�e�Z�N�V�����֐���
//�n���C�Z���T�Z�N�V�����֐�
void dirSec(void) {
    /*
    ���ݒl���擾
    �O�������ƌ��ݒl�Ƃ̊p�x�����Z�o����

    �p�x���̒l���r���₷���悤�ɒ�������i-180~180�j

    �����p�x�����O�������͈̔͂������l�̒��ɂȂ��Ȃ�
        �T�uCPU�̔��FLED��_��������
        �����W�����Z�o����
        ���[�^�[�p���[���Z�o����

        ��ᐧ��̓����Ă��Ȃ��֐��Ń��[�^�[�ɏo�͂���

        ���߉ߋ��̊p�x���X�V����
        ���߉ߋ��̔����W�����X�V����

        �n���C�Z���T�Z�N�V�����t���O�ϐ����X�V����

    �n���C�Z���T�Z�N�V�����t���O�ϐ����X�V����

    �֐����I������
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
        //���C���Z���T���������Ă��Ȃ�
        lineOK = 1;
        return;

    case 9111:
        //�O�̂�
        motorBreak();
        move(-80, -80, 80, 80);
        break;

    case 1911:
        //���̂�
        motorBreak();
        move(80, -80, -80, 80);
        break;

    case 1191:
        //���̂�
        motorBreak();
        move(80, 80, -80, -80);
        break;

    case 1119:
        //�E�̂�
        motorBreak();
        move(-80, 80, 80, -80);
        break;

    case 9911:
        //�O-��
        motorBreak();
        move(0, -80, 0, 80);
        break;

    case 1991:
        //��-���
        motorBreak();
        move(80, 0, -80, 0);
        break;

    case 1199:
        //���-�E
        motorBreak();
        move(0, 80, 0, -80);
        break;

    case 9119:
        //�E-�O
        motorBreak();
        move(-80, 0, 80, 0);
        break;

    case 9991:
        //�O-��-���
        motorBreak();
        move(80, -80, -80, 80);
        break;

    case 1999:
        //��-���-�E
        motorBreak();
        move(80, 80, -80, -80);
        break;

    case 9199:
        //���-�E-�O
        motorBreak();
        move(-80, 80, 80, -80);
        break;

    case 9919:
        //�E-�O-��
        motorBreak();
        move(-80, -80, 80, 80);
        break;

    case 9999:
        //���ׂẴ��C���Z���T���������Ă���
        motorOff();
        break;
    }

    sub_io_set_Led(1, 1, on);
    lineOK = 0;

    return;
}







//���C���֐�
void user_main(void) {

    //�T�uCPU�̕ϐ��Ƀ��C���Z���T�̂������l��������
    sub_io_set_gV(1, VAR_S, LINE_LOW);

    //�T�uCPU�̕ϐ��Ɋe���C���Z���T��r���ɉ��Z����l��������
    sub_io_set_gV(1, VAR_B, LINE_FRONTP);
    sub_io_set_gV(1, VAR_C, LINE_LEFTP);
    sub_io_set_gV(1, VAR_D, LINE_BACKP);
    sub_io_set_gV(1, VAR_E, LINE_RIGHTP);

    //�n���C�Z���T�̃Z�b�g�A�b�v���s��
    dirSetup();

    //���������[�v
    while (TRUE) {

        //==================================================
        //�T�uCPU�`�F�b�N�Z�N�V����

        //�T�uCPU�̕ϐ���ǂݎ��i�ǂݎ��Ȃ��ꍇ�A���j
        subFlag = sub_io_get_gV(1, VAR_Z);

        //�����T�uCPU�̕ϐ���1�ȊO�i�ǂݎ��Ȃ��A���̒l�Ȃǁj�Ȃ�
        if (subFlag != 1) {

            //���[�^�[���~������
            motorOff();

            //���������[�v�̍ŏ��ɖ߂葱����
            continue;
            
        }

        //==================================================


        //==================================================
        //�n���C�Z���T�Z�N�V����

        dirSec();

        if (dirOK != 1) {
            continue;
        }

        sub_io_set_Led(1, 0, off);

        //==================================================

        //==================================================
        //���C���Z���T�Z�N�V����

        lineSec();

        if (lineOK != 1) {
            continue;
        }

        sub_io_set_Led(1, 1, off);

        //==================================================

        //==================================================
        //�{�[���Z���T�Z�N�V����

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
            //������
            //������
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
            //�ߋ���
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