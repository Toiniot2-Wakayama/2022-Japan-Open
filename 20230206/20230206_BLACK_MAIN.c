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
CN10[9] �i�u�U�[�j
I2C 4ch���[�^�[�h���C�o

�T�uCPU
CN1[0]  ���C���O
CN2[1]  ���C����
CN3[2]  ���C�����
CN4[3]  ���C���E
CN5[4]  A�{�^��
CN6[5]  N.C.
CN7[6]  N.C.
CN8[7]  N.C.
CN9[8]  N.C.
CN10[9] B�{�^��
I2C �n���C�Z���T

4ch���[�^�[�h���C�o
gPwm[0] 
gPwm[1] 
gPwm[2] 
gPwm[3] 
gPwm[4] N.C.
gPwm[5] N.C.
*/

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//�萔�̐錾�E��`

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
#define LINE_LOW 575
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
P���䂩PD���䂩�̃t���O�ϐ��iP:0�APD:1�j
*/
#define FRONT_RANGE 10
#define DELTA_T 0.01
#define KP 0.8
#define KD 0.075
#define P_PD 0

//���������p
/*
*/

//�L�[�p�[�@�p
/*
�R�[�g����ɖ߂�܂ł̐������ԁims�j
*/
#define KEEPER_BACK_TIMELIMIT 5000

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//�ϐ��̐錾

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
�Z���T�l�̍��v�l�͈̔͂��L�^����ϐ�
�������l��菬�����l���o�͂����Z���T�̌�
��r�l���L�^����ϐ�
*/
int B_highI;
int B_highV;
int B_sumV;
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
�T�uCPU��CN5�̃Z���T�l�i�{�^��A�j
�T�uCPU��CN10�̃Z���T�l�i�{�^��B�j
�{�^��A��������Ă��郂�[�h���L�^����ϐ�
�{�^��B��������Ă��郂�[�h���L�^����ϐ�
���{�b�g�̃��[�h��ݒ肷��ϐ�
�{�^��A�������ꂽ�񐔂��L�^����ϐ�
�{�^��B�������ꂽ�񐔂��L�^����ϐ�
*/
int btnB;
int btnA;
int btnModeA;
int btnModeB;
int btnTimeA;
int btnTimeB;

//�T�uCPU�̋N����Ԃ��L�^����ϐ�
int subFlag;

//�e�Z�N�V�����t���O�ϐ�
int dirOK;
int lineOK;

//�`�F�b�J�[�p�ϐ�
int kp;
int kd;

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//����֐��錾�E��`

//��������֗��֐��i�g��Ȃ��Ă�OK�j
//��Βl
int abs(int num) {
    
    //���������̒l��0��菬�����i���̐��j�Ȃ�
    if (num < 0) {
        //�����̐��������ւ��čX�V����
        num *= -1;
    }

    //�Ԃ�l��Ԃ��Ċ֐��I��
    return(num);
}



//�Z���T�l�X�V�֐���
//�{�[���Z���T�l�̍X�V�֐�
int refreshBallSensor(void) {

    //�ł��Z���T�l���傫���Z���T���擾
    //�������l���Ⴂ�l���o�͂����Z���T�̌���������
    B_lowQ = 0;

    //���ōł��傫���Z���T�l���o�͂����Z���T�̃C���f�b�N�X��������
    B_highI = 0;

    //���ōł��傫���Z���T�l���o�͂����Z���T�̃Z���T�l���擾
    B_highV = gAD[B_highI];

    //���̍ł��傫���Z���T�l���������l��菬����������
    if (B_highV < BALL_LOW) {
        B_lowQ += 1;
    }

    //CN2����CN7�܂�7��J��Ԃ�
    for (b = 0; b < 7; b++) {

        //��r����l���擾
        B_comparingV = gAD[b + 1];

        //��r����l���������l��菬����������
        if (B_comparingV < BALL_LOW) {
            B_lowQ += 1;
        }

        //��r����l���ł��傫���Z���T�l���傫��������
        if (B_comparingV > B_highV) {

            //�ł��傫���Z���T�l���o�͂����Z���T�̃C���f�b�N�X���X�V
            B_highI = b + 1;

            //�ł��傫���Z���T�l���o�͂����Z���T�̃Z���T�l���X�V
            B_highV = B_comparingV;
        }
    }

    //�O�����Z�q��p�����{�[���̑���
    //�ł��傫���Z���T�l���o�͂����Z���T�̃C���f�b�N�X����1����������0��菬�����Ȃ�CN8���A�łȂ����1�O�̃C���f�b�N�X�̃Z���T�l���擾
    B_sumV = B_highI - 1 < 0 ? gAD[7] : gAD[B_highI - 1];
    //�ł��傫���Z���T�l���o�͂����Z���T�̃C���f�b�N�X��1����������7���傫���Ȃ�CN1���A�łȂ����1��̃C���f�b�N�X�̃Z���T�l�����Z
    B_sumV += B_highI + 1 > 7 ? gAD[0] : gAD[B_highI + 1];

    //���������{�[���̋������敪����
    //�{�[���܂ł̋�����������-�������̂������l��菬����������
    if (B_sumV < BALL_FAR_MEDIUM) {

        //�{�[���͉�����
        B_distance = 0;
    }
    //�{�[���܂ł̋�����������-�������̂������l�ƒ�����-�ߋ����̂������l�̊Ԃɂ��邩����
    else if (BALL_FAR_MEDIUM <= B_sumV && B_sumV < BALL_MEDIUM_NEAR) {

        //�{�[���͒�����
        B_distance = 1;
    }
    else {
        
        //�{�[���͋ߋ���
        B_distance = 2;
    }

    //�֐��I��
    return(0);
}

//���C���Z���T�l�̍X�V�֐�
int refreshLineSensor(void) {

    //�T�uCPU�̕ϐ���ǂݎ���ă��C�����̃O���[�o���ϐ����X�V����
    L_situation = sub_io_get_gV(1, VAR_A);

    //�֐��I��
    return(0);
}

//�{�^��A�E�{�^��B�̏�ԍX�V�֐�
int refreshButton(void) {

    //�T�uCPU�̃Z���T�l�𒼐ڎ擾
    //CN5�̒l��btnA�ɑ��
    btnA = sub_io_get_sensor(1, 4);
    //CN10�̒l��btnB�ɑ��
    btnB = sub_io_get_sensor(1, 9);

    //�֐��I��
    return(0);
}


//�n���C�Z���T�̃Z�b�g�A�b�v�֐�
int dirSetup(void){
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

    //�֐��I��
    return(0);
}



//���[�^�[�֘A�֐���
//���[�^�[�̖ڕW�l�����[�^�[�̍ő�l�E�ŏ��l�ƏƂ炵���킹�ēK���l�ɒ����֐�
int motorCheck(int motor0, int motor1, int motor2, int motor3) {

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

    //�֐��I��
    return(0);
}

//��ᐧ��Ȃ��Ń��[�^�[�ɏo�͂���֐�
int move(int motor0, int motor1, int motor2, int motor3) {

    //�����̃��[�^�[�p���[��K��������
    motorCheck(motor0, motor1, motor2, motor3);

    //���[�^�[�p���[��������
    gPwm[3] = m0 < 0 ? m0 * -1 : m0 | 0x80;
    gPwm[1] = m1 < 0 ? m1 * -1 : m1 | 0x80;
    gPwm[5] = m2 < 0 ? m2 * -1 : m2 | 0x80;
    gPwm[0] = m3 < 0 ? m3 * -1 : m3 | 0x80;

    //���[�^�[�ɏo��
    pwm_out();

    //�֐��I��
    return(0);
}

//���[�^�[�I�t�֐�
int motorOff(void) {

    //"reverse"�̃`�F�b�N���O������ԂŃ��[�^�[�p���[�����ׂ�0�ɂ���
    gPwm[0] = 0x00;
    gPwm[1] = 0x00;
    gPwm[2] = 0x00;
    gPwm[3] = 0x00;

    //���[�^�[�ɏo��
    pwm_out();

    //�֐��I��
    return(0);
}

//���[�^�[�}�u���[�L�֐�
int motorBreak(void) {
    
    //"reverse"�̃`�F�b�N����ꂽ��ԂŃ��[�^�[�p���[�����ׂ�0�ɂ���
    gPwm[0] = 0x00 | 0x80;
    gPwm[1] = 0x00 | 0x80;
    gPwm[2] = 0x00 | 0x80;
    gPwm[3] = 0x00 | 0x80;

    //���[�^�[�ɏo��
    pwm_out();

    //�֐��I��
    return(0);
}



//�e�Z�N�V�����֐���
//�n���C�Z���T�Z�N�V�����֐�
int dirSec(void) {

    //���ݒl���擾
    now = get_bno(0);

    //�O�������ƌ��ݒl�Ƃ̊p�x�����Z�o����
    dif = front - now;

    //�p�x���̒l���r���₷���悤�ɒ�������i-180~180�j
    if (dif <= -180) {
        dif += 360;
    }
    else if (dif >= 180) {
        dif -= 360;
    }

    //���̐�Βl���O���͈͂̒��ɂȂ��Ȃ�
    if ((abs(dif) < FRONT_RANGE) == FALSE) {

        //�T�uCPU�̔��FLED��_��������
        sub_io_set_Led(1, 0, on);

        //�����W�����Z�o����
        nowD = (now - previous) / DELTA_T;

        //�萔�l�ɉ����ă��[�^�[�p���[���Z�o����
        if (P_PD == 0) {
            difMotor = (KP * dif);
        }
        else if (P_PD == 1) {
            difMotor = (KP * dif) + (KD * (nowD - previousD));
        }

        //���[�^�[�ɏo�͂���
        move(difMotor, difMotor, difMotor, difMotor);

        //���߉ߋ��̊p�x�l���X�V����
        previous = now;

        //���߉ߋ��̔����W�����X�V����
        previousD = nowD;

        //�t���O�ϐ������낷
        dirOK = 0;
        
        //�֐��I��
        return(0);
    }

    //�t���O�ϐ��𗧂Ă�
    dirOK = 1;

    //�֐��I��
    return(0);
}

int lineSec(void) {
    
    //�������m�󋵂��X�V
    refreshLineSensor();

    //�������m�󋵂̕ϐ��̒l�ɂ���ē�����ς���]
    /*
    �Z�Z�Z�Z
    �b�b�b�b
    �b�b�b�[�[ �E���C���Z���T�i1:�������A9:�����j
    �b�b�[�[�[ ��냉�C���Z���T�i1:�������A9:�����j
    �b�[�[�[�[ �����C���Z���T�i1:�������A9:�����j
    �[�[�[�[�[ �O���C���Z���T�i1:�������A9:�����j
    */

    switch (L_situation) {
    case 1111:
        //���C���Z���T���������Ă��Ȃ�

        //�t���O�ϐ��𗧂Ă�
        lineOK = 1;

        //�֐��I��
        return(0);

    case 9111:
        //�O�̂�

        //�}�u���[�L
        motorBreak();

        //���ɓ���
        move(-80, -80, 80, 80);

        break;

    case 1911:
        //���̂�

        //�}�u���[�L
        motorBreak();

        //�E�ɓ���
        move(80, -80, -80, 80);

        break;

    case 1191:
        //���̂�

        //�}�u���[�L
        motorBreak();
        
        //�O�ɓ���
        move(80, 80, -80, -80);

        break;

    case 1119:
        //�E�̂�

        //�}�u���[�L
        motorBreak();

        //���ɓ���
        move(-80, 80, 80, -80);

        break;

    case 9911:
        //�O-��

        //�}�u���[�L
        motorBreak();

        //�E���ɓ���
        move(0, -80, 0, 80);

        break;

    case 1991:
        //��-���

        //�}�u���[�L
        motorBreak();

        //�E�O�ɓ���
        move(80, 0, -80, 0);

        break;

    case 1199:
        //���-�E

        //�}�u���[�L
        motorBreak();

        //���O�ɓ���
        move(0, 80, 0, -80);

        break;

    case 9119:
        //�E-�O

        //�}�u���[�L
        motorBreak();

        //�����ɓ���
        move(-80, 0, 80, 0);
        
        break;

    case 9991:
        //�O-��-���

        //�}�u���[�L
        motorBreak();

        //�E�ɓ���
        move(80, -80, -80, 80);
        
        break;

    case 1999:
        //��-���-�E

        //�}�u���[�L
        motorBreak();

        //�O�ɓ���
        move(80, 80, -80, -80);

        break;

    case 9199:
        //���-�E-�O

        //�}�u���[�L
        motorBreak();

        //���ɓ���
        move(-80, 80, 80, -80);

        break;

    case 9919:
        //�E-�O-��

        //�}�u���[�L
        motorBreak();
        
        //���ɓ���
        move(-80, -80, 80, 80);

        break;

    case 9999:
        //���ׂẴ��C���Z���T���������Ă���

        //���[�^�[�I�t
        motorOff();

        break;
    }

    //�T�uCPU�̐ԐFLED��_��������
    sub_io_set_Led(1, 1, on);

    //�t���O�ϐ������낷
    lineOK = 0;

    //�֐��I��
    return(0);

}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@



//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
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

    //A�{�^���EB�{�^���̃��[�h������
    btnTimeA = 0;
    btnTimeB = 0;
    btnModeA = 0;
    btnModeB = 0;
    
    //���[�h�I��
    //���̖��������[�v����

    //���C��CPU�ƃT�uCPU�̂��ׂĂ�LED��_��������
    set_Led(0, on);
    set_Led(1, on);
    sub_io_set_Led(1, 0, on);
    sub_io_set_Led(1, 1, on);

    while (TRUE) {

        //�{�^���̏�ԕϐ����X�V
        refreshButton();

        //�{�^��A�̃��[�h��������Ă��Ȃ���ԂŃ{�^��A�������ꂽ��
        if (btnA > 500 && btnModeA == 0) {
            
            //�{�^��A�������ꂽ�񐔂�1���Z
            btnTimeA += 1;

            //�{�^��A�̃��[�h��������Ă����ԂɍX�V
            btnModeA = 1;
        }
        
        //�{�^��A��������Ă��Ȃ�������
        else if (btnA < 500) {

            //�{�^��A�̃��[�h��������Ă��Ȃ���ԂɍX�V
            btnModeA = 0;
        }

        //�{�^��B�̃��[�h��������Ă��Ȃ���ԂŃ{�^��B�������ꂽ��
        if (btnB > 500 && btnModeB == 0) {

            //�{�^��B�̃��[�h��������Ă����ԂɍX�V
            btnModeB = 1;
        }

        //�{�^��B��������Ă��炸�A�{�^��B�̃��[�h��������Ă��Ȃ���ԂȂ�
        else if (btnB < 500 && btnModeB == 1) {

            //���̖��������[�v�𔲂��o��
            break;
        }
    }

    //���C��CPU�ƃT�uCPU�̂��ׂĂ�LED������������
    set_Led(0, off);
    set_Led(1, off);
    sub_io_set_Led(1, 0, off);
    sub_io_set_Led(1, 1, off);

    //�{�^��A�������ꂽ�񐔂ɂ���ē�������ς���
    switch (btnTimeA) {
    case 0:
    case 1:
        //�U�����[�h
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
        //������[�h

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
                //�ߋ���
                
                //�������Ԃ��߂��Ă�����
                if (T1 > KEEPER_BACK_TIMELIMIT) {
                    //��냉�C���Z���T����������܂Ō��ɉ�����
                    while (TRUE) {
                        //���C���Z���T�̒l���X�V
                        refreshLineSensor();

                        //�������m�󋵂̋L�^�ϐ��ɉ����ē�����ς���
                        switch (L_situation) {
                        case 1191:
                            //��ނ���߂�
                            break;

                        case 1911:
                            move(0, -80, 0, 80);
                            //���������[�v�̍ŏ��ɖ߂�
                            continue;

                        case 1119:
                            move(-80, 0, 80, 0);
                            //���������[�v�̍ŏ��ɖ߂�
                            continue;
                            
                        default:
                            move(-80, -80, 80, 80);
                            //���������[�v�̍ŏ��ɖ߂�
                            continue;
                        }

                        //���������[�v���甲���o��
                        break;
                    }

                    //1000ms�O�i����
                    move(80, 80, -80, -80);
                    wait_ms(1000);

                    //���������[�v�̍ŏ��ɖ߂�
                    continue;
                }

                //�������Ԉȓ��Ȃ�{�[����Ǐ]����
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
    //��������`�F�b�J�[���[�h

    case 4:
        //�{�[�������`�F�b�J�[

        //���������[�v
        while (TRUE) {

            refreshBallSensor();

            if (B_lowQ == 8) {

                //�{�[�������m���Ă��Ȃ�
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);

                //���������[�v�̍ŏ��ɖ߂�
                continue;
            }

            switch (B_distance) {
            case 0:
                //������
                set_Led(0, on);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                break;

            case 1:
                //������
                set_Led(0, off);
                set_Led(1, on);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                break;

            case 2:
                //�ߋ���
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
        //�{�[��8���ʊp�x�`�F�b�J�[

        //���������[�v
        while (TRUE) {

            refreshBallSensor();

            if(B_lowQ == 8){

                //�{�[�������m���Ă��Ȃ�
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);

                //���������[�v�̍ŏ��ɖ߂�
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
        //�n���C�Z���T�Q�C���ω��`�F�b�J�[

        btnTimeA = 0;
        btnTimeB = 0;

        //���������[�v
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
        //�{�[�����E�e���v�l��r�`�F�b�J�[
        //�i�Ȃ��j
        break;

    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    case 8:
        //���C���Z���T�`�F�b�J�[

        //���������[�v
        while (TRUE) {

            refreshLineSensor();

            switch (L_situation) {
            case 1111:
                //���������m���Ă��Ȃ�
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                break;

            case 9111:
                //�O�̂�
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);
                break;

            case 1911:
                //���̂�
                set_Led(0, on);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);
                break;

            case 1191:
                //���̂�
                set_Led(0, off);
                set_Led(1, on);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);
                break;

            case 1119:
                //�E�̂�
                set_Led(0, on);
                set_Led(1, on);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);
                break;

            case 9911:
                //�O�E��
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;

            case 1991:
                //���E���
                set_Led(0, on);
                set_Led(1, off);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;

            case 1199:
                //���E�E
                set_Led(0, off);
                set_Led(1, on);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;

            case 9119:
                //�E�E�O
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
                //3�܂���4�̃��C���Z���T�����������m���Ă���
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
        //�n���C�Z���T�`�F�b�J�[

        //���������[�v
        while (TRUE) {

            //���ݒl���擾
            now = get_bno(0);

            //�O�������ƌ��ݒl�Ƃ̊p�x�����Z�o����
            dif = front - now;

            //�p�x���̒l���r���₷���悤�ɒ�������i-180~180�j
            if (dif <= -180) {
                dif += 360;
            }
            else if (dif >= 180) {
                dif -= 360;
            }

            //���̐�Βl���O���͈͂̒��ɂȂ��Ȃ�
            if ((abs(dif) < FRONT_RANGE) == FALSE) {

                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, on);

                //�O�������ƌ��ݒl�Ƃ̊p�x�����}�C�i�X�Ȃ�
                if (dif < 0) {
                    set_Led(0, on);
                    set_Led(1, off);
                }
                //�O�������ƌ��ݒl�Ƃ̊p�x�����v���X�Ȃ�
                else if (dif > 0) {
                    set_Led(0, off);
                    set_Led(1, on);
                }

                //���������[�v�̍ŏ��ɖ߂�
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
        //���{�b�g�͖������ɒ�~

        //���������[�v
        while (TRUE) {

            //���[�^�[�I�t
            motorOff();
        }
        break;
    }
}

/*
�v���O���~���O�̃q���g

�� ��T�Ԍ�̎����͐Ԃ̑��l���Ǝv���ăR�����g���c���Ă����ׂ�

�E�R�����g // �̓v���O�����ŏI�s�ɗ��Ă͂Ȃ�Ȃ�
�E����֐��̕Ԃ�l�^��void�ɂ��Ă͂Ȃ�Ȃ��iint���I�X�X���j
�Emath.h�Ɋ܂܂��֐��i�O�p�֐��Ȃǁj���g�����Ƃ͂ł�����
�E�r���h�G���[�����������Ƃ��̓G���[�����s�̒��O�s�ŃZ�~�R���� ; �������Ă��Ȃ����`�F�b�N
�E�r���h�G���[�����������Ƃ��ɃG���[�����s���v���O�����ŏI�s�Ȃ�R�����g�A�E�g�̏I��肪�c���Ă�����t�ɏI��肪�Ȃ������肵�Ȃ����`�F�b�N
�E�r���h�G���[�����������Ƃ��ɕ��@�G���[����������Ȃ��ꍇ��C-style���ċN�����Ă�����x�r���h���Ă݂�

*/