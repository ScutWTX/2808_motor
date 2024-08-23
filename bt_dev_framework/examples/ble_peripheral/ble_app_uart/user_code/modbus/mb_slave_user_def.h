#ifndef __MB_SLAVE_USER_DEF_H
#define __MB_SLAVE_USER_DEF_H
 
/** \brief 从机地址 */
#define MB_SLAVE_USER_ADDR  0x08

/********************* 线圈地址 *****************************************/

#define MB_SLAVE_USER_DIR              0x012C /**< \brief 线圈: 0:进丝,1:退丝 */
#define MB_SLAVE_USER_STARTUP          0x012D /**< \brief 线圈: 0:停止,1:启动 */
#define MB_SLAVE_USER_STOP_ROLLBACK    0x012E /**< \brief 线圈: 停止抽丝,0:无效,1:有效 */
#define MB_SLAVE_USER_RESET            0x012F /**< \brief 线圈：1:复位,0:无效,置1执行后自动复位0 */

#define MB_SLAVE_USER_SEG_SWITCH       0x0130 /**< \brief 线圈：段切换 */
#define MB_SLAVE_USER_SEG_STARTUP      0x0131 /**< \brief 线圈：分段 0:停止, 1:开始 */

#define MB_SLAVE_USER_COIL_MIN_ADDR MB_SLAVE_USER_DIR   /**< \brief 线圈最小地址 */
#define MB_SLAVE_USER_COIL_MAX_ADDR MB_SLAVE_USER_SEG_STARTUP /**< \brief 线圈最大地址 */

/********************* 寄存器地址 *****************************************/

#define MB_SLAVE_USER_SPEED            0x1388 /**< \brief 寄存器: 送丝速度, ADDR_WIREFEEDER_COIL_DIR使用此速度配置 */
#define MB_SLAVE_USER_PULL_SPEED       0x1389 /**< \brief 寄存器：抽丝速度, ADDR_WIREFEEDER_COIL_WIRE_PULL_ACTIVE有效时使用此速度配置*/
#define MB_SLAVE_USER_PULL_TIME        0x138A /**< \brief 寄存器：抽丝时间 */
#define MB_SLAVE_USER_STATUS           0x138B /**< \brief 寄存器：状态 */

#define MB_SLAVE_USER_SEG_ID           0x138C
#define MB_SLAVE_USER_ISPERIOD         0x138D
#define MB_SLAVE_USER_FEEDINGSPEED     0x138E
#define MB_SLAVE_USER_DRAWINGSPEED     0x138F
#define MB_SLAVE_USER_FEEDINGTIME      0x1390
#define MB_SLAVE_USER_DRAWINGTIME      0x1391

//#define MB_SLAVE_USER_SEG_ID           0x138C /**< \brief 段id       */
//#define MB_SLAVE_USER_SEG_DELTA        0x138D /**< \brief 段变化量   */
//#define MB_SLAVE_USER_SEG_SLOPE_TIME   0x138E /**< \brief 段渐变时间 */
//#define MB_SLAVE_USER_SEG_DURATION     0x138F /**< \brief 段持续时间 */

#define MB_SLAVE_USER_REG_MIN_ADDR MB_SLAVE_USER_SPEED        /**< \brief 最小寄存器地址 */
#define MB_SLAVE_USER_REG_MAX_ADDR MB_SLAVE_USER_DRAWINGTIME  /**< \brief 最大寄存器地址 */

#endif /* __MB_SLAVE_USER_DEF_H */
