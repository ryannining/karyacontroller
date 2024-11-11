#pragma once
#ifndef temp_H
#define temp_H
extern int ltemp_pin,water_pin,temp_limit,BUZZER_ERR;
extern void init_temp();
extern void temp_loop(uint32_t cm);
#endif