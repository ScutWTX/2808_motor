/**
 * \file
 * \brief motor control header file
 *
 * \internal
 * \par modification history
 * - 1.00 2021-04-12  lwx, first implementation
 * \endinternal
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "prjconfig.h"

/** \brief ����˶�ָ�� */
enum servo_motor_motion {
    SVO_MOTOR_STOP = 0,      /**< \brief stop                              */
    SVO_MOTOR_MOVE_TO_LUF,   /**< \brief move to left, up, forward side    */
    SVO_MOTOR_MOVE_TO_RDB    /**< \brief move to right, down, backard side */
};

/** \brief ������ָֹͣ�� */
enum servo_cmd {
    SVO_CMD_POWERON = 0, /**< \brief �ŷ����ڶ������� */
	SOV_CMD_POWEROFF     /**< \brief �ŷ����ڶ���ֹͣ */
};

/** \brief �ŷ���ʼ�� */
void servo_init(void);

/**
 * \brief �ŷ�����
 * \param[in] cmd: ��ָͣ�� 
 */
void servo_welding_motor_ctrl(enum serov_cmd cmd);

/** 
 * \brief �ŷ�������� 
 * \param[in] idx: PRJCONF_ROUGH_FB, PRJCONF_ROUGH_UD, PRJCONF_FINE_LR, PRJCONF_FINE_UD, etc. define in prjconfig.h
 * \param[in] motion: stop, move to luf or move to rdb \ref servo_motor_dir
 *
 * \return true:  success
 * \return false: invalid param(s)
 */
bool servo_motor_ctrl(enum motor_axis_idx idx, enum servo_motor_motion motion);

/**
 * \brief �����ֶ��ٶ�
 * \param[in] idx: PRJCONF_ROUGH_FB, PRJCONF_ROUGH_UD, PRJCONF_FINE_LR, PRJCONF_FINE_UD, etc. define in prjconfig.h
 * \param[in] speed: �ٶ�,��λ:mm/min
 * \return true:  success
 * \return false: invalid param(s) 
 */
bool servo_motor_set_manual_speed(enum motor_axis_idx idx, uint16_t speed);

/**
 * \brief ���ú���ʱ���ٶ�
 * \param[in] idx: PRJCONF_ROUGH_FB, PRJCONF_ROUGH_UD, PRJCONF_FINE_LR, PRJCONF_FINE_UD, etc. define in prjconfig.h
 * \param[in] speed: �ٶ�,��λ:mm/min
 * \return true:  success
 * \return false: invalid param(s) 
 */
bool servo_motor_set_auto_speed(enum motor_axis_idx idx, uint16_t speed);

/**
 * \brief �ڶ�������
 * \param[in] cmd: ��ָͣ��
 */
void servo_wiggler_ctrl(enum servo_cmd);
 

#endif /* __MOTOR_H */
