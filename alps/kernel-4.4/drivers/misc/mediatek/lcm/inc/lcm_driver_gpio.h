#ifndef __LCM_DRIVER_GPIO_H__
#define __LCM_DRIVER_GPIO_H__
extern unsigned int GPIO_LCD_PWR_;
extern unsigned int GPIO_LCD_RST_;
extern unsigned int GPIO_LCD_ID1_;
int lcm_vgp_supply_enable(void);
int lcm_vgp_supply_disable(void);
#endif


