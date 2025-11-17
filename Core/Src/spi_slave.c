#include "spi_slave.h"
#include <string.h>

/* 最多挂几个实例（这里放2个就够你用 SPI1+SPI2 了；要更多可以加大） */
#define MAX_SPI_SLAVE_CTX 4
static SPI_SlaveCtx *s_ctxs[MAX_SPI_SLAVE_CTX] = {0};

/* 在表里登记/查找对应的 ctx */
static void _register_ctx(SPI_SlaveCtx *ctx)
{
    for (int i = 0; i < MAX_SPI_SLAVE_CTX; ++i) {
        if (!s_ctxs[i]) {
            s_ctxs[i] = ctx;
            return;
        }
    }
}
static SPI_SlaveCtx *_find_ctx(SPI_HandleTypeDef *hspi)
{
    for (int i = 0; i < MAX_SPI_SLAVE_CTX; ++i) {
        if (s_ctxs[i] && s_ctxs[i]->hspi == hspi) return s_ctxs[i];
    }
    return NULL;
}

/* 内部：帧解析 */
static void _parse_frame(SPI_SlaveCtx *ctx)
{
    if (ctx->len == SPI_Slave_FrameLen6) {
        if (ctx->rxbuf[0] == 0xA5 && ctx->rxbuf[5] == 0x5A) {
            ctx->d0     = ctx->rxbuf[1];
            ctx->d1     = ctx->rxbuf[2];
            ctx->d2     = ctx->rxbuf[3];
            ctx->d3     = ctx->rxbuf[4];
            ctx->new_u8 = 1;
        }
    } else { /* 10B */
        if (ctx->rxbuf[0] == 0xA5 && ctx->rxbuf[9] == 0x5A) {
            ctx->a       = (int16_t)((ctx->rxbuf[2] << 8) | ctx->rxbuf[1]);
            ctx->b       = (int16_t)((ctx->rxbuf[4] << 8) | ctx->rxbuf[3]);
            ctx->c       = (int16_t)((ctx->rxbuf[6] << 8) | ctx->rxbuf[5]);
            ctx->d       = (int16_t)((ctx->rxbuf[8] << 8) | ctx->rxbuf[7]);
            ctx->new_i16 = 1;
        }
    }
}

uint8_t SPI_Slave_TryGet4I8(SPI_SlaveCtx *ctx, int8_t *a, int8_t *b, int8_t *c, int8_t *d)
{
    if (!ctx->new_u8) return 0;
    if (a) *a = (int8_t)ctx->d0;
    if (b) *b = (int8_t)ctx->d1;
    if (c) *c = (int8_t)ctx->d2;
    if (d) *d = (int8_t)ctx->d3;
    ctx->new_u8 = 0;
    return 1;
}

/* Public: 绑定并启动 IT 接收（自动续订） */
void SPI_Slave_InitAndStart_IT(SPI_SlaveCtx *ctx, SPI_HandleTypeDef *hspi, SPI_Slave_FrameLen len)
{
    ctx->hspi   = hspi;
    ctx->len    = len;
    ctx->new_u8 = ctx->new_i16 = 0;
    _register_ctx(ctx);

    /* 预挂接收：主机拉低 NSS 并发时钟后硬件会移入 rxbuf，收满 len 字节触发中断 */
    HAL_SPI_Receive_IT(ctx->hspi, ctx->rxbuf, (uint16_t)ctx->len);
}

/* Try-get APIs（查询式，非阻塞） */
uint8_t SPI_Slave_TryGet4U8(SPI_SlaveCtx *ctx, uint8_t *d0, uint8_t *d1, uint8_t *d2, uint8_t *d3)
{
    if (!ctx->new_u8) return 0;
    if (d0) *d0 = ctx->d0;
    if (d1) *d1 = ctx->d1;
    if (d2) *d2 = ctx->d2;
    if (d3) *d3 = ctx->d3;
    ctx->new_u8 = 0;
    return 1;
}
uint8_t SPI_Slave_TryGet4I16(SPI_SlaveCtx *ctx, int16_t *a, int16_t *b, int16_t *c, int16_t *d)
{
    if (!ctx->new_i16) return 0;
    if (a) *a = ctx->a;
    if (b) *b = ctx->b;
    if (c) *c = ctx->c;
    if (d) *d = ctx->d;
    ctx->new_i16 = 0;
    return 1;
}

/* ===== HAL 回调：对号入座到对应 ctx ===== */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_SlaveCtx *ctx = _find_ctx(hspi);
    if (!ctx) return;

    _parse_frame(ctx);
    /* 立刻续订下一帧 */
    HAL_SPI_Receive_IT(ctx->hspi, ctx->rxbuf, (uint16_t)ctx->len);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    SPI_SlaveCtx *ctx = _find_ctx(hspi);
    if (!ctx) return;

    /* 最简单的恢复：重新挂接收 */
    HAL_SPI_Receive_IT(ctx->hspi, ctx->rxbuf, (uint16_t)ctx->len);
}

