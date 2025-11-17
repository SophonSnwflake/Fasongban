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

// #include "spi_slave.h" // 引入本模块对应的头文件，提供外部可见的API与类型
// #include <string.h>    // 用于内存操作，如 memcpy 等（本文件里暂未使用）

// /* 外部 SPI 句柄（由 CubeMX 提供） */
// extern SPI_HandleTypeDef SPI_SLAVE_HANDLE; // 声明一个在别处定义的 SPI 句柄（通常在 spi.c 里定义的 hspiX）；本模块通过它访问硬件

// /* 当前帧长度与接收缓冲 */
// static volatile SPI_Slave_FrameLen s_len = SPI_Slave_FrameLen6; // 当前期望接收的帧长度（6 或 10），volatile 防止编译器优化导致的读取不一致
// static uint8_t s_rxbuf[10];                                     // 接收缓冲区，最大容纳 10 字节（可同时支持 6/10 字节两种帧）

// /* 帧到达标志与解析后的数据缓存 */
// static volatile uint8_t s_new_u8 = 0, s_new_i16 = 0; // 两个“新数据到达”标志：u8 帧/ i16 帧分别置位
// static uint8_t s_d0, s_d1, s_d2, s_d3;               // 保存解析后的 4 个 uint8_t 数据
// static int16_t s_a, s_b, s_c, s_d;                   // 保存解析后的 4 个 int16_t 数据

// /* 内部：校验并解析一帧 */
// static void _parse_frame(void) // 仅在本模块内部使用：对 s_rxbuf 中的数据做帧头/帧尾校验并提取字段
// {
//     if (s_len == SPI_Slave_FrameLen6) {                 // 如果当前配置的是 6 字节帧格式：A5 d0 d1 d2 d3 5A
//         if (s_rxbuf[0] == 0xA5 && s_rxbuf[5] == 0x5A) { // 校验帧头 0xA5 和帧尾 0x5A
//             s_d0     = s_rxbuf[1];                      // 提取 d0
//             s_d1     = s_rxbuf[2];                      // 提取 d1
//             s_d2     = s_rxbuf[3];                      // 提取 d2
//             s_d3     = s_rxbuf[4];                      // 提取 d3
//             s_new_u8 = 1;                               // 置位“有一帧 4×u8 新数据到达”的标志
//         }
//     } else { /* 10 字节 */                                         // 否则是 10 字节帧：A5 lo0 hi0 lo1 hi1 lo2 hi2 lo3 hi3 5A（小端：低字节在前）
//         if (s_rxbuf[0] == 0xA5 && s_rxbuf[9] == 0x5A) {            // 同样校验帧头帧尾
//             s_a       = (int16_t)((s_rxbuf[2] << 8) | s_rxbuf[1]); // 合成 16 位 a：高字节在前 s_rxbuf[2]，低字节 s_rxbuf[1]
//             s_b       = (int16_t)((s_rxbuf[4] << 8) | s_rxbuf[3]); // 合成 16 位 b
//             s_c       = (int16_t)((s_rxbuf[6] << 8) | s_rxbuf[5]); // 合成 16 位 c
//             s_d       = (int16_t)((s_rxbuf[8] << 8) | s_rxbuf[7]); // 合成 16 位 d
//             s_new_i16 = 1;                                         // 置位“有一帧 4×i16 新数据到达”的标志
//         }
//     }
// }

// /* 对外：启动一次中断接收（会在回调里自动续订） */
// void SPI_Slave_Start_IT(SPI_Slave_FrameLen len) // 对外 API：以“中断方式”开始接收一帧，回调里会自动重新挂接收
// {
//     s_len = len; // 记录当前期望的帧长度（6 或 10）
//     /* 先清标志 */
//     s_new_u8  = 0; // 清除 u8 帧的新数据标志
//     s_new_i16 = 0; // 清除 i16 帧的新数据标志
//     /* 预备接收 N 字节——主机拉低 NSS 并打时钟后，SPI 硬件将移入 s_rxbuf */
//     HAL_SPI_Receive_IT(&SPI_SLAVE_HANDLE, // 使用 HAL 的“中断接收”API
//                        s_rxbuf,           // 指向接收缓冲区
//                        (uint16_t)s_len);  // 本次期望接收的字节数（与帧长一致）
// }

// /* 查询式：4×uint8_t */
// uint8_t SPI_Slave_TryGet4U8(uint8_t *d0, uint8_t *d1, uint8_t *d2, uint8_t *d3) // 非阻塞读取：若有新 4×u8 数据就拷贝出来并清标志
// {
//     if (!s_new_u8) return 0; // 若没有新数据，返回 0
//     if (d0) *d0 = s_d0;      // 调用者给了指针就写回 d0
//     if (d1) *d1 = s_d1;      // 写回 d1
//     if (d2) *d2 = s_d2;      // 写回 d2
//     if (d3) *d3 = s_d3;      // 写回 d3
//     s_new_u8 = 0;            // 清除新数据标志（数据已被读取）
//     return 1;                // 返回 1 表示本次确实取到了新数据
// }

// /* 查询式：4×int16_t */
// uint8_t SPI_Slave_TryGet4I16(int16_t *a, int16_t *b, int16_t *c, int16_t *d) // 非阻塞读取：若有新 4×i16 数据就拷贝出来并清标志
// {
//     if (!s_new_i16) return 0; // 没有新数据则返回 0
//     if (a) *a = s_a;          // 写回 a
//     if (b) *b = s_b;          // 写回 b
//     if (c) *c = s_c;          // 写回 c
//     if (d) *d = s_d;          // 写回 d
//     s_new_i16 = 0;            // 清标志
//     return 1;                 // 返回 1 表示读取成功
// }

// /* ======== HAL 回调：接收满一帧后被调用 ======== */
// void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) // HAL 中断回调：当本次 HAL_SPI_Receive_IT 规定的字节数全部收到时触发
// {
//     if (hspi->Instance == SPI_SLAVE_HANDLE.Instance) { // 确认是本 SPI 实例触发的回调（防止多个 SPI 共用回调时串台）
//         _parse_frame();                                // 对收到的 s_rxbuf 做帧校验与解析，并置相应“新数据”标志
//         /* 立刻继续准备下一帧（持续在线） */
//         HAL_SPI_Receive_IT(&SPI_SLAVE_HANDLE, // 再次挂接收，使能持续接收下一帧数据
//                            s_rxbuf,
//                            (uint16_t)s_len);
//     }
// }

// /* （可选）错误恢复 */
// void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) // HAL 错误回调：如溢出、模式故障等
// {
//     if (hspi->Instance == SPI_SLAVE_HANDLE.Instance) { // 同样确认是本 SPI 实例
//         /* 简单恢复：重新挂接收 */
//         HAL_SPI_Receive_IT(&SPI_SLAVE_HANDLE, // 直接重新启动中断接收，做最简单的恢复
//                            s_rxbuf,
//                            (uint16_t)s_len);
//     }
// }
