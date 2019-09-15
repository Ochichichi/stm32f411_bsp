#ifndef __MAGNETO_H
#define __MAGNETO_H

#include <stdint.h>

/**
 * Magnetometer Driver structure
 */
typedef struct
{
    /* data */
    void            (*Init)(uint16_t, uint8_t);
    void            (*DeInit)(void);
    uint8_t         (*ReadID)(void);
    void            (*Reset)(void);
    void            (*GetXYZ)(float*);
    void            (*GetTemperature)(uint8_t);
} MAGNETO_DrvTypeDef;

/**
 * Magnetometer Init structure
 */
typedef struct
{
    /* data */
    uint8_t         Power_Mode;
    uint8_t         MagOutput_DataRate;
    uint8_t         MagFull_Scale;
    uint8_t         MagTemperature;
} MAGNETO_InitTypeDef;

#endif // __MAGNETO_H