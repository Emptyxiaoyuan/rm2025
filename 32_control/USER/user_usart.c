/**
 * @file user_usart.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 串口处理
 * @version 0.1
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "user_usart.h"
#include "user_c.h"
#include "usart.h"

uint8_t Rxbuf[10];
uint8_t RxFlag = 0;

//接受处理变量
volatile uint8_t rx_len = 0;  //接收一帧数据的长度
volatile uint8_t recv_end_flag = 0; //一帧数据接收完成标志
uint8_t rx_buffer[100]={0};  //接收数据缓存数组


volatile uint8_t  usart_dma_tx_over = 1;


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

void usart1_tx_dma_init(void)
{
  // enable the DMA transfer for the receiver and tramsmit request
  // 使能DMA串口接收和发送
  SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
  SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

  // disable DMA
  // 失效DMA
  __HAL_DMA_DISABLE(&hdma_usart1_tx);

  while (hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_usart1_tx);
  }

  hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
  hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
  hdma_usart1_tx.Instance->NDTR = 0;
	
	HAL_UART_Receive_IT(&huart1,Rxbuf,10);
	
	HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);
	
	
}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
  // disable DMA
  // 失效DMA
  __HAL_DMA_DISABLE(&hdma_usart1_tx);

  while (hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_usart1_tx);
  }

  __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

  hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
  __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

  __HAL_DMA_ENABLE(&hdma_usart1_tx);
}

void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	// 使能DMA串口接收和发送
  SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
  SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

  // 使能空闲中断
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

  // 失效DMA
  __HAL_DMA_DISABLE(&hdma_usart6_rx);
  while (hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
  }

  // DMA配置
  hdma_usart6_rx.Instance->PAR = (uint32_t) &(USART6->DR);
  hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
  hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
  __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

  // 使能双缓冲区模式
  SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

  // 使能DMA
  __HAL_DMA_ENABLE(&hdma_usart6_rx);

  // 失效DMA
  __HAL_DMA_DISABLE(&hdma_usart6_tx);
  while (hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_usart6_tx);
  }

  hdma_usart6_tx.Instance->PAR = (uint32_t) &(USART6->DR);
	
	
	
//  // 使能DMA串口接收和发送
//  SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
//  SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

//  // 使能空闲中断
//  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

//  // 失效DMA
//  __HAL_DMA_DISABLE(&hdma_usart6_rx);

//  while (hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
//  {
//    __HAL_DMA_DISABLE(&hdma_usart6_rx);
//  }

//  __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

//  hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
//  // 内存缓冲区1
//  hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
//  // 内存缓冲区2
//  hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
//  // data length
//  // 数据长度
//  __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

//  // 使能双缓冲区
//  SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

//  // 使能DMA
//  __HAL_DMA_ENABLE(&hdma_usart6_rx);

//  // 失效DMA
//  __HAL_DMA_DISABLE(&hdma_usart6_tx);

//  while (hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
//  {
//    __HAL_DMA_DISABLE(&hdma_usart6_tx);
//  }

//  hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);
}

/**
 * @brief UART+DMA 发送数据
 *
 * @param data 发送数据
 * @param len 数据长度
 */
void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
  // 失效DMA
  __HAL_DMA_DISABLE(&hdma_usart6_tx);

  while (hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_usart6_tx);
  }

  __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);

  hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
  __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);

  __HAL_DMA_ENABLE(&hdma_usart6_tx);
}


int Printf_Usart1_DMA(const char *format,...)
{
  va_list arg;
  static char SendBuff[200] = {0};
  int rv;
  while(!usart_dma_tx_over);//等待前一次DMA发送完成
 
  va_start(arg,format);
  rv = vsnprintf((char*)SendBuff,sizeof(SendBuff)+1,(char*)format,arg);
  va_end(arg);
 
  HAL_UART_Transmit_DMA(&huart1,(uint8_t *)SendBuff,rv);
  usart_dma_tx_over = 0;//清0全局标志，发送完成后重新置1
 
  return rv;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        RxFlag = 1;
        HAL_UART_Receive_IT(&huart1, (uint8_t *)Rxbuf, 10);    // 使能串口接收中断
    }
}
 

