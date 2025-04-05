#ifndef USER_USART_H
#define USER_USART_H
#include "user_c.h"

extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#define BUFFER_SIZE  100  
extern  volatile uint8_t rx_len ;  //接收一帧数据的长度
extern volatile uint8_t recv_end_flag; //一帧数据接收完成标志
extern uint8_t rx_buffer[100];  //接收数据缓存数组

extern uint8_t Rxbuf[10];
extern uint8_t RxFlag;

extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
/**
 * @brief UART+DMA 发送数据
 *
 * @param data
 * @param len
 */
void usart6_tx_dma_enable(uint8_t *data, uint16_t len);
#endif /*USER_USART_H*/
