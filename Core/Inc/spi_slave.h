#pragma once
#include "spi.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 两种帧长：A5 d0 d1 d2 d3 5A（6字节）或 A5 lo0 hi0 lo1 hi1 lo2 hi2 lo3 hi3 5A（10字节） */
typedef enum {
    SPI_Slave_FrameLen6  = 6,
    SPI_Slave_FrameLen10 = 10,
} SPI_Slave_FrameLen;

/* 每个 SPI 从机实例的上下文 */
typedef struct {
    SPI_HandleTypeDef *hspi;         /* 绑定的 SPI 外设（hspi1/hspi2/...） */
    volatile SPI_Slave_FrameLen len; /* 当前帧长 */
    uint8_t rxbuf[10];               /* 接收缓冲（最大 10B） */

    /* 解析后的数据与“新数据标志” */
    volatile uint8_t new_u8, new_i16;
    uint8_t d0, d1, d2, d3; /* 4×u8 */
    int16_t a, b, c, d;     /* 4×i16 */
} SPI_SlaveCtx;

/* 绑定并启动一次中断接收（会在回调里自动续订） */
void SPI_Slave_InitAndStart_IT(SPI_SlaveCtx *ctx, SPI_HandleTypeDef *hspi, SPI_Slave_FrameLen len);

/* 非阻塞读取：有新数据则拷贝出来并清标志，返回1；否则返回0 */
uint8_t SPI_Slave_TryGet4U8(SPI_SlaveCtx *ctx, uint8_t *d0, uint8_t *d1, uint8_t *d2, uint8_t *d3);
uint8_t SPI_Slave_TryGet4I16(SPI_SlaveCtx *ctx, int16_t *a, int16_t *b, int16_t *c, int16_t *d);

/* 交给 HAL 的通用回调 —— 你仍然写在这个 .c 里就行 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

// #ifndef SPI_SLAVE_H
// #define SPI_SLAVE_H

// #include "spi.h"
// #include <stdint.h>

// #ifdef __cplusplus
// extern "C" {
// #endif

// /* ======= 可按需改：SPI 实例 ======= */
// #ifndef SPI_SLAVE_HANDLE
// #define SPI_SLAVE_HANDLE hspi1
// #endif
// /* ================================== */

// /* 两种帧长度 */
// typedef enum {
//     SPI_Slave_FrameLen6  = 6,  /* A5 d0 d1 d2 d3 5A */
//     SPI_Slave_FrameLen10 = 10, /* A5 lo0 hi0 ... lo3 hi3 5A */
// } SPI_Slave_FrameLen;

// /* 初始化并启动一次接收（默认 6 字节帧） */
// void SPI_Slave_Start_IT(SPI_Slave_FrameLen len);

// /* 查询是否收到新帧（查询式）。返回 1=有新帧，0=没有 */
// uint8_t SPI_Slave_TryGet4U8(uint8_t *d0, uint8_t *d1, uint8_t *d2, uint8_t *d3);

// /* 16 位版本：返回 1=有新帧 */
// uint8_t SPI_Slave_TryGet4I16(int16_t *a, int16_t *b, int16_t *c, int16_t *d);

// #ifdef __cplusplus
// }
// #endif
// #endif /* SPI_SLAVE_H */
