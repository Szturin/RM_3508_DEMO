#include "bsp_system.h"

uint8_t key_val = 0;  // ��ǰ����״̬
uint8_t key_old = 0;  // ǰһ����״̬
uint8_t key_down = 0; // ���µİ���
uint8_t key_up = 0;   // �ͷŵİ���

/**
 * @brief ��ȡ����״̬
 *
 * �ú�����ȡ������ GPIO �����ϵİ���״̬����������Ӧ�İ�����š�
 *
 * @return ���ذ�����š�0 ��ʾû�а������£�1-4 ��ʾ��Ӧ�İ��������¡�
 */
uint8_t key_read(void)
{
  // ���ڴ洢����״̬����ʱ����
  uint8_t temp = 0;

  // ��� GPIOB ���� 0 ��״̬
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET)
    temp = 1; // �������״̬Ϊ RESET���򰴼� 1 ������

  // ��� GPIOB ���� 1 ��״̬
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET)
    temp = 2; // �������״̬Ϊ RESET���򰴼� 2 ������

  // ��� GPIOB ���� 2 ��״̬
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET)
    temp = 3; // �������״̬Ϊ RESET���򰴼� 3 ������

  // ��� GPIOA ���� 0 ��״̬
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
    temp = 4; // �������״̬Ϊ RESET���򰴼� 4 ������

  // ���ؼ�⵽�İ������
  return temp;
}

/**
 * @brief ����������
 *
 * �ú�������ɨ�谴����״̬�������°������º��ͷŵı�־
 */
void key_proc(void)
{
  // ��ȡ��ǰ����״̬
  key_val = key_read();
  // ���㰴�µİ�������ǰ����״̬��ǰһ״̬��򣬲��뵱ǰ״̬���룩
  key_down = key_val & (key_old ^ key_val);
  // �����ͷŵİ�������ǰδ����״̬��ǰһ״̬��򣬲���ǰһ״̬���룩
  key_up = ~key_val & (key_old ^ key_val);
  // ����ǰһ����״̬
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
  GPIO_TypeDef *gpiox;     // ָ�� GPIO �˿ڵ�ָ�룬���� GPIOA��GPIOB �ȡ�
  uint16_t pin;            // ָ�� GPIO ���ţ����� GPIO_PIN_0��GPIO_PIN_1 �ȡ�
  uint16_t ticks;          // ���ڼ�ʱ�ı�����ͨ������ȥ��������
  uint8_t level;           // ��ǰ�����ĵ�ƽ״̬���ߵ�ƽ��͵�ƽ��
  uint8_t id;              // ������Ψһ��ʶ���������������ֲ�ͬ�İ�����
  uint8_t state;           // �����ĵ�ǰ״̬���������ڱ�ʾ�����Ƿ񱻰��»��ͷš�
  uint8_t debouce_cnt;     // ����ȥ���������������ڷ�ֹ��������������󴥷���
  uint8_t repeat;          // �����ظ����µĴ�����
} button;

button btns[4]; // ��������

// ������ʼ������
void key_init(void)
{
  // ��ʼ����һ������
  btns[0].gpiox = GPIOB;    // ָ��GPIO�˿�
  btns[0].pin = GPIO_PIN_0; // ָ������
  btns[0].level = 1;        // ���ó�ʼ��ƽ
  btns[0].id = 0;           // ���ð���ID

  // ��ʼ���ڶ�������
  btns[1].gpiox = GPIOB;
  btns[1].pin = GPIO_PIN_1;
  btns[1].level = 1;
  btns[1].id = 1;

  // ��ʼ������������
  btns[2].gpiox = GPIOB;
  btns[2].pin = GPIO_PIN_2;
  btns[2].level = 1;
  btns[2].id = 2;

  // ��ʼ�����ĸ�����
  btns[3].gpiox = GPIOA;
  btns[3].pin = GPIO_PIN_0;
  btns[3].level = 1;
  btns[3].id = 3;
}


// ����״̬������
void key_state(void)
{
  for (uint8_t i = 0; i < 4; i++) // �������а���
  {
    //key_task(&btns[i]); // ����ÿ��������״̬
  }
}
