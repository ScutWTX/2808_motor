#ifndef __WIREFEEDER_H 
#define __WIREFEEDER_H

#include "stdint.h"
#include "stdbool.h"

/** \brief ��˿��״̬ */
enum wfr_status {
    WFR_STAT_OK = 0,         /**< \brief ����״̬ */
    WFR_STAT_COMM_EXCEPTION, /**< \brief ͨ���쳣 */
};

/** \brief ��˿or��˿ */
enum wfr_dir {
    WFR_DIR_PUSH = 0,        /**< \brief ��˿     */
    WFR_DIR_PULL,            /**< \brief ��˿     */
};

/** \brief ��˿����ʼ�� */
void wfr_init(void);

/**
 * \brief ֹͣ��˿
 * \param[in] is_active: true-��Ч,false-��Ч
 */
void wfr_set_rollback_when_stop(bool is_active);

/**
 * \brief ������˿������
 * \param[in] dir: WFR_DIR_PUSH-��˿,WFR_DIR_PULL-��˿
 */
void wfr_set_dir(enum wfr_dir dir);

/**
 * \brief ��˿��������ֹͣ
 * \param[in] is_startup: true:����,false:ֹͣ
 */
void wfr_startup(bool is_startup);

/**
 * \brief ��˿����˿�ٶ����� 
 * \param[in] speed: �ٶ�
 */
void wfr_set_push_speed(uint16_t speed);

/** 
 * \brief ��˿�ٶ����� 
 * \param[in] speed: ��˿�ٶ�
 */
void wfr_set_pull_speed(uint16_t speed);

/**
 * \brief ��˿ʱ������
 * \param[in] time����˿ʱ��,��λ0.1s
 */
void wfr_set_pull_time(uint16_t time);


/** \brief ��ȡ��ǰ״̬ */
enum wfr_status wfr_get_status(void);

/** \brief �ֶβ��� */
typedef struct {
//    uint16_t id;         /**< \brief ���     */
//    uint16_t variation;  /**< \brief �仯��   */
//    uint16_t slope_time; /**< \brief ����ʱ�� */
//    uint16_t duration;   /**< \brief ����ʱ�� */
    uint16_t id;
    uint16_t isperiod;
    uint16_t feeding_speed;
    uint16_t drawing_speed;
    uint16_t feeding_time;
    uint16_t drawing_time;
} segs;

#define TIMING_OF_FEEDING 1
#define TIMING_OF_DRAWING 2

/** \brief �ι��� */
typedef struct {
    uint8_t  max_seg_id; /**< \brief ����                */
    uint8_t  cur_seg;    /**< \brief ��ǰ��(��0��ʼ����) */
    int16_t  intval;     /**< \brief ����׶β�ֵ        */
    int16_t  cur_val;    /**< \brief ��ǰֵ              */
    segs data[100];       /**< \brief ������              */
} seg_manager;

/** \brief �������� */
void set_seg_max_count(uint16_t id);

/** \brief ���ö����� */
void set_seg_data(uint16_t id, uint16_t offs, uint16_t data);

/** \brief ��λ������ */
void reset_seg(void);

/** \brief ���л� */
void seg_switch(void);

/** \brief �ֶο��� */
void seg_ctrl(bool startup);

#endif /* __WIREFEEDER_H */
