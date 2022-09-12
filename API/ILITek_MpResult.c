/*
 * Copyright (c) 2019 ILI Technology Corp.
 *
 * This file is part of ILITEK Linux Daemon Tool
 *
 * Copyright (c) 2021 Luca Hsu <luca_hsu@ilitek.com>
 * Copyright (c) 2021 Joe Hung <joe_hung@ilitek.com>
 */
#ifndef _ILITEK_MP_RESULT_C_
#define _ILITEK_MP_RESULT_C_

#include <stdio.h>
#include <stdlib.h>
#include "ILITek_MpResult.h"
#include "../ILITek_Main.h"
#include "../ILITek_Protocol.h"
#include "../ILITek_CMDDefine.h"
#include "../ILITek_Device.h"

int get_fwid_by_mpresult(uint16_t *fwid)
{
	int error;
	uint16_t customer_id;

	if ((error = viGetPanelInfor()) < 0)
		return error;

	if ((error = GetFWID(&customer_id, fwid)) < 0)
		return error;

	LD_DBG("Customer ID: %#hx, FWID: %#hx from MpResult\n",
		customer_id, *fwid);

	return 0;
}

#endif
