#ifndef __WIREFEEDER_H 
#define __WIREFEEDER_H

#include "stdint.h"
#include "stdbool.h"

/** \brief 送丝机状态 */
enum wfr_status {
    WFR_STAT_OK = 0,         /**< \brief 正常状态 */
    WFR_STAT_COMM_EXCEPTION, /**< \brief 通信异常 */
};

/** \brief 送丝or退丝 */
enum wfr_dir {
    WFR_DIR_PUSH = 0,        /**< \brief 送丝     */
    WFR_DIR_PULL,            /**< \brief 退丝     */
};

/** \brief 送丝机初始化 */
void wfr_init(void);

/**
 * \brief 停止抽丝
 * \param[in] is_active: true-有效,false-无效
 */
void wfr_set_rollback_when_stop(bool is_active);

/**
 * \brief 配置送丝机方向
 * \param[in] dir: WFR_DIR_PUSH-进丝,WFR_DIR_PULL-退丝
 */
void wfr_set_dir(enum wfr_dir dir);

/**
 * \brief 送丝机启动或停止
 * \param[in] is_startup: true:启动,false:停止
 */
void wfr_startup(bool is_startup);

/**
 * \brief 送丝机送丝速度配置 
 * \param[in] speed: 速度
 */
void wfr_set_push_speed(uint16_t speed);

/** 
 * \brief 抽丝速度设置 
 * \param[in] speed: 抽丝速度
 */
void wfr_set_pull_speed(uint16_t speed);

/**
 * \brief 抽丝时间设置
 * \param[in] time：抽丝时间,单位0.1s
 */
void wfr_set_pull_time(uint16_t time);


/** \brief 获取当前状态 */
enum wfr_status wfr_get_status(void);

/** \brief 分段参数 */
typedef struct {
//    uint16_t id;         /**< \brief 编号     */
//    uint16_t variation;  /**< \brief 变化量   */
//    uint16_t slope_time; /**< \brief 渐变时间 */
//    uint16_t duration;   /**< \brief 持续时间 */
    uint16_t id;
    uint16_t isperiod;
    uint16_t feeding_speed;
    uint16_t drawing_speed;
    uint16_t feeding_time;
    uint16_t drawing_time;
} segs;

#define TIMING_OF_FEEDING 1
#define TIMING_OF_DRAWING 2

/** \brief 段管理 */
typedef struct {
    uint8_t  max_seg_id; /**< \brief 段数                */
    uint8_t  cur_seg;    /**< \brief 当前段(从0开始计数) */
    int16_t  intval;     /**< \brief 渐变阶段插值        */
    int16_t  cur_val;    /**< \brief 当前值              */
    segs data[100];       /**< \brief 段数据              */
} seg_manager;

/** \brief 设置最大段 */
void set_seg_max_count(uint16_t id);

/** \brief 设置段数据 */
void set_seg_data(uint16_t id, uint16_t offs, uint16_t data);

/** \brief 复位段数据 */
void reset_seg(void);

/** \brief 段切换 */
void seg_switch(void);

/** \brief 分段控制 */
void seg_ctrl(bool startup);

#endif /* __WIREFEEDER_H */
