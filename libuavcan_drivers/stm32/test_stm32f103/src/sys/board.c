/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <ch.h>
#include <hal.h>

const PALConfig pal_default_config = {
    { VAL_GPIOAODR, VAL_GPIOACRL, VAL_GPIOACRH },
    { VAL_GPIOBODR, VAL_GPIOBCRL, VAL_GPIOBCRH },
    { VAL_GPIOCODR, VAL_GPIOCCRL, VAL_GPIOCCRH },
    { VAL_GPIODODR, VAL_GPIODCRL, VAL_GPIODCRH },
    { VAL_GPIOEODR, VAL_GPIOECRL, VAL_GPIOECRH }
};

void __early_init(void)
{
    stm32_clock_init();
}

#ifndef AFIO_MAPR_CAN_REMAP_REMAP2
#define AFIO_MAPR_CAN_REMAP_REMAP2 ((uint32_t)0x00004000)
#endif

void boardInit(void)
{
    AFIO->MAPR |=
        AFIO_MAPR_CAN_REMAP_REMAP2 |
        AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
}
