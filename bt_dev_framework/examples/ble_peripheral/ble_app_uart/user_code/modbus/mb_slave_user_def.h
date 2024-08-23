#ifndef __MB_SLAVE_USER_DEF_H
#define __MB_SLAVE_USER_DEF_H
 
/** \brief �ӻ���ַ */
#define MB_SLAVE_USER_ADDR  0x08

/********************* ��Ȧ��ַ *****************************************/

#define MB_SLAVE_USER_DIR              0x012C /**< \brief ��Ȧ: 0:��˿,1:��˿ */
#define MB_SLAVE_USER_STARTUP          0x012D /**< \brief ��Ȧ: 0:ֹͣ,1:���� */
#define MB_SLAVE_USER_STOP_ROLLBACK    0x012E /**< \brief ��Ȧ: ֹͣ��˿,0:��Ч,1:��Ч */
#define MB_SLAVE_USER_RESET            0x012F /**< \brief ��Ȧ��1:��λ,0:��Ч,��1ִ�к��Զ���λ0 */

#define MB_SLAVE_USER_SEG_SWITCH       0x0130 /**< \brief ��Ȧ�����л� */
#define MB_SLAVE_USER_SEG_STARTUP      0x0131 /**< \brief ��Ȧ���ֶ� 0:ֹͣ, 1:��ʼ */

#define MB_SLAVE_USER_COIL_MIN_ADDR MB_SLAVE_USER_DIR   /**< \brief ��Ȧ��С��ַ */
#define MB_SLAVE_USER_COIL_MAX_ADDR MB_SLAVE_USER_SEG_STARTUP /**< \brief ��Ȧ����ַ */

/********************* �Ĵ�����ַ *****************************************/

#define MB_SLAVE_USER_SPEED            0x1388 /**< \brief �Ĵ���: ��˿�ٶ�, ADDR_WIREFEEDER_COIL_DIRʹ�ô��ٶ����� */
#define MB_SLAVE_USER_PULL_SPEED       0x1389 /**< \brief �Ĵ�������˿�ٶ�, ADDR_WIREFEEDER_COIL_WIRE_PULL_ACTIVE��Чʱʹ�ô��ٶ�����*/
#define MB_SLAVE_USER_PULL_TIME        0x138A /**< \brief �Ĵ�������˿ʱ�� */
#define MB_SLAVE_USER_STATUS           0x138B /**< \brief �Ĵ�����״̬ */

#define MB_SLAVE_USER_SEG_ID           0x138C
#define MB_SLAVE_USER_ISPERIOD         0x138D
#define MB_SLAVE_USER_FEEDINGSPEED     0x138E
#define MB_SLAVE_USER_DRAWINGSPEED     0x138F
#define MB_SLAVE_USER_FEEDINGTIME      0x1390
#define MB_SLAVE_USER_DRAWINGTIME      0x1391

//#define MB_SLAVE_USER_SEG_ID           0x138C /**< \brief ��id       */
//#define MB_SLAVE_USER_SEG_DELTA        0x138D /**< \brief �α仯��   */
//#define MB_SLAVE_USER_SEG_SLOPE_TIME   0x138E /**< \brief �ν���ʱ�� */
//#define MB_SLAVE_USER_SEG_DURATION     0x138F /**< \brief �γ���ʱ�� */

#define MB_SLAVE_USER_REG_MIN_ADDR MB_SLAVE_USER_SPEED        /**< \brief ��С�Ĵ�����ַ */
#define MB_SLAVE_USER_REG_MAX_ADDR MB_SLAVE_USER_DRAWINGTIME  /**< \brief ���Ĵ�����ַ */

#endif /* __MB_SLAVE_USER_DEF_H */
