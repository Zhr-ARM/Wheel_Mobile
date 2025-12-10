#include "usart.h"
#include "string.h"
#include "pid.h" 
#include "Uart.h"
#include "math.h"

extern UART_HandleTypeDef huart3;
extern PID_Incremental_t pid_motor_a; 

extern PID_Incremental_TypeDef pid_m_a;
extern PID_Incremental_TypeDef pid_m_b;
extern PID_Incremental_TypeDef pid_m_c;
extern PID_Incremental_TypeDef pid_m_d;

extern float Target_Speed_A;
extern float Target_Speed_B;
extern float Target_Speed_C;
extern float Target_Speed_D;

uint8_t Recv3;
uint8_t Recv_buf3[32];
uint8_t Recv_index3 = 0; 
uint8_t Recv_flag3 = 0;

uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_flag = 0; 

uint8_t tx_buffer[TX_BUF_SIZE]; 

void USART_Init_Logic(void)
{
    
    // 开启空闲中断
    // __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    
    // 开启 DMA 接收
    HAL_UART_Receive_IT(&huart3, rx_buffer, RX_BUFFER_SIZE);
}

void Uart3_Receive()
{
  if(Recv_flag3)
  return;
  if(Recv_index3==0&&Recv3!='(')
  return;
  Recv_buf3[Recv_index3++] = Recv3; 
  if(Recv_index3>=2&&Recv3==')')
  Recv_flag3 = 1; 
}


void Uart3_Parse()
{
  if(Recv_flag3)
  {
    int x,y,w;    
    float val = 0.0f;
    float p, i, d;
    int t;
    Recv_buf3[Recv_index3-1] = '\0'; // 将结束符替换为字符串结束符
    if (sscanf((char*)Recv_buf3, "p=%f", &val) == 1)
    {
        pid_motor_a.Kp = val;
    }
    else if (sscanf((char*)Recv_buf3, "i=%f", &val) == 1)
    {
        pid_motor_a.Ki = val;
    }
    else if (sscanf((char*)Recv_buf3, "#T=%d,P=%f,I=%f,D=%f$", &t, &p, &i, &d) == 4)
    {
        Target_Speed_B = (float)t;
        pid_m_b.kp = p;
        pid_m_b.ki = i;
        pid_m_b.kd = d;
    }
    Recv_flag3=0;
    Recv_index3=0;
  }
}

void Send_Speed(float ch1, float ch2, float ch3, float ch4)
{
    int len = sprintf((char*)tx_buffer, "%.6f, %.6f, %.6f, %.6f\r\n", ch1, ch2, ch3, ch4);
    HAL_UART_Transmit_DMA(&huart3, tx_buffer, len);
}

void USER_UART_IRQHandler_User(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3) 
    {
        Uart3_Receive();
        HAL_UART_Receive_IT(&huart3, &Recv3, 1);
    }
}

void Process_Command(void)
{
    if (rx_flag)
    {
        float val = 0.0f;
        float p, i, d;
        int t;
        // 解析指令: p=1.5, i=0.2, d=0.05
        if (sscanf((char*)rx_buffer, "p=%f", &val) == 1)
        {
            pid_motor_a.Kp = val;
        }
        else if (sscanf((char*)rx_buffer, "i=%f", &val) == 1)
        {
            pid_motor_a.Ki = val;
        }
        else if (sscanf((char*)rx_buffer, "#T=%d,P=%f,I=%f,D=%f$", &t, &p, &i, &d) == 4)
        {
            Target_Speed_B = (float)t;
            pid_m_b.kp = p;
            pid_m_b.ki = i;
            pid_m_b.kd = d;
        }

        // 清除标志，准备下一次接收
        rx_flag = 0;
    }
}

