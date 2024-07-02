#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <Arduino.h>
#include <stdio.h>
#include <stdint.h>
#include <esp_task_wdt.h>
#include <string.h>

#define UP_LEG 5    // CM
#define LED    10

void vofaPrint(char *str, int num);

#endif