#ifndef __GYRO_H
#define __GYRO_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/**
 * Gyroscope Driver structure
 */
typedef struct
{
    /* data */
    void        (*Init)(uint16_t);
    void        (*DeInit)(void);
    uint8_t     (*ReadId)(void);
    void        (*Reset)(void);
    void        (*LowPower)(uint16_t);
    void        (*ConfigIT)(uint16_t);
    void        (*EnableIT)(uint8_t);
    void        (*DisableIT)(uint8_t);
    void        (*ITStatus)(uint16_t, uint16_t);
    void        (*ClearIT)(uint16_t, uint16_t);
    void        (*FilterConfig)(uint8_t);
    void        (*FilterCmd)(uint8_t);
    void        (*GetXYZ)(float *);
} GYRO_DrvTypeDef;

/**
 * Gyroscope Configuration structure
 */
typedef struct
{
    /* data */
    uint8_t Power_Mode;                         /* Power-down/Sleep/Normal Mode */
    uint8_t Output_DataRate;                    /* OUT data rate */
    uint8_t Axes_Enable;                        /* Axes enable */
    uint8_t Band_Width;                         /* Bandwidth selection */
    uint8_t BlockData_Update;                   /* Block Data Update */
    uint8_t Endianness;                         /* Endian Data selection */
    uint8_t Full_Scale;                         /* Full Scale selection */
} GYRO_InitTypeDef;

/**
 * GYRO High Pass Filter struct
typedef struct
{
    /* data */
    uint8_t HighPassFilter_Mode_Selection;      /* Internal filter mode */
    uint8_t HighPassFilter_CutOff_Frequency;    /* High pass filter cut-off frequency */
} GYRO_FilterConfigTypeDef;

typedef struct
{
    /* data */
    uint8_t Latch_Request;                      /* Latch interrupt request into CLICK_SRC register */
    uint8_t Interrupt_Axes;                     /* X, Y, Z Axes Interrupts */ 
    uint8_t Interrupt_ActiveEdge;               /* Interrupt Active edge */
} GYRO_InterruptConfigTypeDef;

#endif // __GYRO_H