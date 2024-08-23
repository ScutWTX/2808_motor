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

/** \brief 电机运动指令 */
enum servo_motor_motion {
    SVO_MOTOR_STOP = 0,      /**< \brief stop                              */
    SVO_MOTOR_MOVE_TO_LUF,   /**< \brief move to left, up, forward side    */
    SVO_MOTOR_MOVE_TO_RDB    /**< \brief move to right, down, backard side */
};

/** \brief 启动、停止指令 */
enum servo_cmd {
    SVO_CMD_POWERON = 0, /**< \brief 伺服、摆动器启动 */
	SOV_CMD_POWEROFF     /**< \brief 伺服、摆动器停止 */
};

/** \brief 伺服初始化 */
void servo_init(void);

/**
 * \brief 伺服控制
 * \param[in] cmd: 启停指令 
 */
void servo_welding_motor_ctrl(enum serov_cmd cmd);

/** 
 * \brief 伺服电机控制 
 * \param[in] idx: PRJCONF_ROUGH_FB, PRJCONF_ROUGH_UD, PRJCONF_FINE_LR, PRJCONF_FINE_UD, etc. define in prjconfig.h
 * \param[in] motion: stop, move to luf or move to rdb \ref servo_motor_dir
 *
 * \return true:  success
 * \return false: invalid param(s)
 */
bool servo_motor_ctrl(enum motor_axis_idx idx, enum servo_motor_motion motion);

/**
 * \brief 设置手动速度
 * \param[in] idx: PRJCONF_ROUGH_FB, PRJCONF_ROUGH_UD, PRJCONF_FINE_LR, PRJCONF_FINE_UD, etc. define in prjconfig.h
 * \param[in] speed: 速度,单位:mm/min
 * \return true:  success
 * \return false: invalid param(s) 
 */
bool servo_motor_set_manual_speed(enum motor_axis_idx idx, uint16_t speed);

/**
 * \brief 设置焊接时的速度
 * \param[in] idx: PRJCONF_ROUGH_FB, PRJCONF_ROUGH_UD, PRJCONF_FINE_LR, PRJCONF_FINE_UD, etc. define in prjconfig.h
 * \param[in] speed: 速度,单位:mm/min
 * \return true:  success
 * \return false: invalid param(s) 
 */
bool servo_motor_set_auto_speed(enum motor_axis_idx idx, uint16_t speed);

/**
 * \brief 摆动器控制
 * \param[in] cmd: 启停指令
 */
void servo_wiggler_ctrl(enum servo_cmd);
 

#endif /* __MOTOR_H */
