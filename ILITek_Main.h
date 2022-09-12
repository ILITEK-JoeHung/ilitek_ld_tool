/*
 * Copyright (c) 2019 ILI Technology Corp.
 *
 * This file is part of ILITEK Linux Daemon Tool
 *
 * Copyright (c) 2021 Luca Hsu <luca_hsu@ilitek.com>
 * Copyright (c) 2021 Joe Hung <joe_hung@ilitek.com>
 */
#ifndef INC_ILITEK_MAIN_H_
#define INC_ILITEK_MAIN_H_
#include <stdint.h>
#include <stdbool.h>

#ifdef USE_ANDROID
#include <jni.h>
extern JNIEnv* g_env;
#endif

extern int ChangeToAPMode(unsigned int start, unsigned int end);
extern int ChangeToBootloader(unsigned int start, unsigned int end);

//M3 + M2V
extern int ChangeToAPMode_M3_M2V();
extern int ChangeToBootloader_M3_M2V();
//

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
