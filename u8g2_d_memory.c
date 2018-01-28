/* u8g2_d_memory.c */
/* generated code, codebuild, u8g2 project */

#include "u8g2.h"


uint8_t *u8g2_m_16_8_1(uint8_t *page_cnt)
{
  static uint8_t buf[128];
  *page_cnt = 1;
  return buf;
}
uint8_t *u8g2_m_16_8_2(uint8_t *page_cnt)
{
  static uint8_t buf[256];
  *page_cnt = 2;
  return buf;
}
uint8_t *u8g2_m_16_8_f(uint8_t *page_cnt)
{
  static uint8_t buf[1024];
  *page_cnt = 8;
  return buf;
}

/* end of generated code */
