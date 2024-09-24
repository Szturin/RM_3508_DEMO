#include "bsp_system.h"

uint8_t key_val = 0;  // 当前按键状态
uint8_t key_old = 0;  // 前一按键状态
uint8_t key_down = 0; // 按下的按键
uint8_t key_up = 0;   // 释放的按键

/**
 * @brief 读取按键状态
 *
 * 该函数读取连接在 GPIO 引脚上的按键状态，并返回相应的按键编号。
 *
 * @return 返回按键编号。0 表示没有按键按下，1-4 表示对应的按键被按下。
 */
uint8_t key_read(void)
{
  // 用于存储按键状态的临时变量
  uint8_t temp = 0;

  // 检查 GPIOB 引脚 0 的状态
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET)
    temp = 1; // 如果引脚状态为 RESET，则按键 1 被按下

  // 检查 GPIOB 引脚 1 的状态
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET)
    temp = 2; // 如果引脚状态为 RESET，则按键 2 被按下

  // 检查 GPIOB 引脚 2 的状态
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET)
    temp = 3; // 如果引脚状态为 RESET，则按键 3 被按下

  // 检查 GPIOA 引脚 0 的状态
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
    temp = 4; // 如果引脚状态为 RESET，则按键 4 被按下

  // 返回检测到的按键编号
  return temp;
}

/**
 * @brief 按键处理函数
 *
 * 该函数用于扫描按键的状态，并更新按键按下和释放的标志
 */
void key_proc(void)
{
  // 读取当前按键状态
  key_val = key_read();
  // 计算按下的按键（当前按下状态与前一状态异或，并与当前状态相与）
  key_down = key_val & (key_old ^ key_val);
  // 计算释放的按键（当前未按下状态与前一状态异或，并与前一状态相与）
  key_up = ~key_val & (key_old ^ key_val);
  // 更新前一按键状态
  key_old = key_val;
  
	if(key_down == 3){
		set_speed += 1000;
		if(set_speed > 5000){
			set_speed = 0;
		}
	}
}

typedef struct
{
  GPIO_TypeDef *gpiox;     // 指向 GPIO 端口的指针，例如 GPIOA、GPIOB 等。
  uint16_t pin;            // 指定 GPIO 引脚，例如 GPIO_PIN_0、GPIO_PIN_1 等。
  uint16_t ticks;          // 用于计时的变量，通常用于去抖动处理。
  uint8_t level;           // 当前按键的电平状态，高电平或低电平。
  uint8_t id;              // 按键的唯一标识符，可以用于区分不同的按键。
  uint8_t state;           // 按键的当前状态，可能用于表示按键是否被按下或释放。
  uint8_t debouce_cnt;     // 按键去抖动计数器，用于防止按键抖动引起的误触发。
  uint8_t repeat;          // 按键重复按下的次数。
} button;

button btns[4]; // 按键数组

// 按键初始化函数
void key_init(void)
{
  // 初始化第一个按键
  btns[0].gpiox = GPIOB;    // 指定GPIO端口
  btns[0].pin = GPIO_PIN_0; // 指定引脚
  btns[0].level = 1;        // 设置初始电平
  btns[0].id = 0;           // 设置按键ID

  // 初始化第二个按键
  btns[1].gpiox = GPIOB;
  btns[1].pin = GPIO_PIN_1;
  btns[1].level = 1;
  btns[1].id = 1;

  // 初始化第三个按键
  btns[2].gpiox = GPIOB;
  btns[2].pin = GPIO_PIN_2;
  btns[2].level = 1;
  btns[2].id = 2;

  // 初始化第四个按键
  btns[3].gpiox = GPIOA;
  btns[3].pin = GPIO_PIN_0;
  btns[3].level = 1;
  btns[3].id = 3;
}


// 按键状态处理函数
void key_state(void)
{
  for (uint8_t i = 0; i < 4; i++) // 遍历所有按键
  {
    //key_task(&btns[i]); // 处理每个按键的状态
  }
}
