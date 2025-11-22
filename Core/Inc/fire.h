#ifndef __FIRE_H
#define __FIRE_H

#include "stdint.h"

// 初始化发射机构 GPIO（PA2）
void Fire_Init(void);

// 每帧调用一次（比如 200Hz）
void Fire_ControlLoop(void);

#endif
