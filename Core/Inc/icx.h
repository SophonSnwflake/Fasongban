
#ifndef __ICX_H__
#define __ICX_H__

#include "tim.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TIM_HandleTypeDef *htim; // 对应的定时器句柄（如 &htim3）
    uint32_t base_ch;        // 作为 PWMI 启动的“基准通道”（建议用 TIM_CHANNEL_1）
    uint32_t tick_hz;        // 计数频率（Hz），例如 1,000,000
} ICX_Handle;

/* 绑定一个定时器实例（不启动） */
void ICX_Attach(ICX_Handle *icx, TIM_HandleTypeDef *htim, uint32_t base_ch, uint32_t tick_hz);
/* 启动 PWMI（会启动 base_ch 以及配对通道） */
HAL_StatusTypeDef ICX_Start(ICX_Handle *icx);

/* 读频率（Hz）与占空比（百分比 0..100） */
uint32_t ICX_GetFreq(const ICX_Handle *icx);
uint32_t ICX_GetDuty(const ICX_Handle *icx);

#ifdef __cplusplus
}
#endif
#endif


