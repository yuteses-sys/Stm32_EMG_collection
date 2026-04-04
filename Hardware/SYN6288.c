#include "syn6288.h" // 引用自己的头文件


extern UART_HandleTypeDef huart3; 

// ==========================================================
// SYN6288 语音合成播报函数 
// ==========================================================
void SYN6288_Say(char *text)
{
    uint8_t frame[200];              
    uint16_t text_len = strlen(text); 
    
    uint16_t data_len = text_len + 3; 
    
    uint8_t xor_check = 0;           
    
    frame[0] = 0xFD;                 
    frame[1] = data_len >> 8;        
    frame[2] = data_len & 0xFF;      
    frame[3] = 0x01;                 
    frame[4] = 0x00;                 // 0x01 是 GBK 编码
    
    memcpy(&frame[5], text, text_len);
    
    for (int i = 0; i < data_len + 2; i++) 
    {
        xor_check ^= frame[i]; 
    }
    
    frame[5 + text_len] = xor_check;
    
    HAL_UART_Transmit(&huart3, frame, 6 + text_len, 1000);
    HAL_Delay(50); 
}