/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ILITEK Linux Daemon Tool
 *
 * Copyright (c) 2021 Luca Hsu <luca_hsu@ilitek.com>
 * Copyright (c) 2021 Joe Hung <joe_hung@ilitek.com>
 *
 * The code could be used by anyone for any purpose,
 * and could perform firmware update for ILITEK's touch IC.
 */

#ifndef INC_ILITEK_MAIN_H_
#define INC_ILITEK_MAIN_H_
#include <stdint.h>
#include <stdbool.h>

extern int ChangeToAPMode();
extern int ChangeToBootloader();

int PanelInfor();
uint32_t RawDataInfor(void);
extern uint32_t get_file_size(char *filename);
int DealWithFunctions(int argc, char *argv[]);
int viGetPanelInfor();
int viSetTestMode(bool setTest, int delay_ms);
int viSwitchMode(int mode);
int GetFlashData_V6(uint32_t start, uint32_t len, char *path);
int viScript(char *argv[]);
int viConsoleData(char *argv[]);
#endif
