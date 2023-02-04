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

//�p�����䎞�̊e�萔�l
/*
�n���C�Z���T�̑O���͈͂������l
CPU�̏�������
���萔
�����萔
�ϕ��萔
*/
#define FRONT_RANGE 10
#define DELTA_T 0.01
#define KP 0.8
#define KD 0.01
#define KI 0.5

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//�ϐ��̐錾

//for���̃��[�v�p�ϐ�
/*
��ʎg�p�p�ϐ�1
��ʎg�p�p�ϐ�2
��ʎg�p�p�ϐ�3

���[�^�[����p�ϐ�
*/
int i;
int j;
int k;

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

//�p������Ɏg�p����ϐ��̐錾
/*
�n���C�Z���T�̑O������
���݊p�x�l
���߉ߋ��p�x�l
�O�������ƌ��ݒl�̊p�x��
���݂̔����W��
���߉ߋ��̔����W��
�ϕ��W��
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



//�n���C�Z���T�̃Z�b�g�A�b�v�֐�
int dirSetup(void) {
    //�n���C�Z���T�̏������E�Z�b�g�A�b�v���s��
    set_bno();

    wait_ms(1000);

    //�p������Ɏg�p����ϐ��̏������E�l�擾
    /*
    �O�������̊p�x��n���C�Z���T�Ŏ擾����
    ���Œ��߉ߋ��p�x�l�Ɍ��݊p�x�l����
    ���Œ��߉ߋ��̔����W����0����
    �ϕ��W����0�ŏ�����
    */
    front = get_bno(0);
    previous = get_bno(0);
    previousD = 0;
    integral = 0;

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

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@



//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//���C���֐�

void user_main(void) {

    //�n���C�Z���T�̃Z�b�g�A�b�v���s��
    dirSetup();

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

            //�T�uCPU�̔��FLED��_��������
            sub_io_set_Led(1, 0, on);

            //�����W�����Z�o����
            nowD = (now - previous) / DELTA_T;

            //�ϕ��W�����Z�o����
            integral += (now + previous) * DELTA_T / 2;

            //�萔�l�ɉ����ă��[�^�[�p���[���Z�o����
            p = dif * KP;
            i = integral * KI;
            d = (nowD - previousD) * KD;

            difMotor = p + i + d;

            //���[�^�[�ɏo�͂���
            move(difMotor, difMotor, difMotor, difMotor);

            //���߉ߋ��̊p�x�l���X�V����
            previous = now;

            //���߉ߋ��̔����W�����X�V����
            previousD = nowD;

            //���������[�v�̍ŏ��ɖ߂�
            continue;
        }

        motorOff();
    }
}