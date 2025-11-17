//控制电机1 - 4转动 
Motor_SetPWM(1, 10);
Motor_SetPWM(2, 10);
Motor_SetPWM(3, 10);
Motor_SetPWM(4, 10);
//第二个变量范围0-100，为占空比


//读取轮速
uint8_t k0 , k1, k2, k3; // SPI2收到的数据
SPI_Slave_TryGet4U8(&spi2_ctx, &k0, &k1, &k2, &k3);
caculateWheelSpeed(1, k0, 1);
caculateWheelSpeed(1, k0, 1);
caculateWheelSpeed(1, k0, 1);
caculateWheelSpeed(1, k0, 1);
// K1,K2,K3,K4代表原始数据,经过caculate计算后，变成有符号的数据，正号向前转，负号向后转

//读取六通道遥控器的值
uint8_t d0, d1, d2, d3,d4,d5;
SPI_Slave_TryGet4U8(&spi1_ctx, &d0, &d1, &d2, &d3);
ICX_GetDuty(&ic3);
ICX_GetDuty(&ic4);
// d1,d2,d3,d4代表原始数据,注意，这里函数和上面一样，但是用的是spi1_ctx，这个是分辨SPI1和SPI2的标识符
// d0:要控制云台左右的摇杆 范围从左到右77-178
// d1:要控制底盘前进后退的摇杆 范围从下到上85-169
// d2:要控制云台上下的摇杆 范围从上到下85-169
// d3:要控制底盘左右的摇杆 范围从左到右85-169
// d4:二档钮子开关，接收值85 169，建议用范围判断而不是直接用==来判断
// d5:二挡钮子开关，接收值01 85，建议用范围判断而不是直接用==来判断

//控制舵机方向
PWM6_SetDutyPercent(5, 5.8);//控制舵机1
PWM6_SetDutyPercent(6, 5.8);//控制舵机2
//后面的第二个变量“5.8”为示例，作用是调节角度的变量，实际使用中，5代表0度，10代表180度，可以用浮点数来表示中间的度数


//OLED是直接CV的，注释比较全，如果要用的话，记得加Update




//获取遥控器的值


