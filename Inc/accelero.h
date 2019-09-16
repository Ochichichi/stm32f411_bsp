#ifndef __ACCELERO_H
#define __ACCELERO_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/**
 * Accelerometer Driver structure
 */
typedef struct
{
    /* data */
    void      (*Init)(uint16_t);
    void      (*DeInit)(void); 
    uint8_t   (*ReadID)(void);
    void      (*Reset)(void);
    void      (*LowPower)(void);
    void      (*ConfigIT)(void);
    void      (*EnableIT)(uint8_t);
    void      (*DisableIT)(uint8_t);
    uint8_t   (*ITStatus)(uint16_t);
    void      (*ClearIT)(void);
    void      (*FilterConfig)(uint8_t);
    void      (*FilterCmd)(uint8_t);
    void      (*GetXYZ)(float *);
} ACCELERO_DrvTypeDef;

/**
 * Accelerometer Configuration structure
 */
typedef struct
{
    /* data */
    uint8_t Power_Mode;                 // Power-down/Normal Mode
    uint8_t AccOutput_DataRate;         // OUT data rate
    uint8_t Axes_Enable;
    uint8_t High_Resolution;            // High resolution enabling/disabling
    uint8_t BlockData_Update;           // Block data update
    uint8_t Endianness;                  // Endian Data selection
    uint8_t AccFull_Scale;              // Full Scale selection
    uint8_t Communication_Mode;
} ACCELERO_InitTypeDef;

/**
 * Accelerometer High Pass Filter struct.
 */
typedef struct
{
    /* data */
    uint8_t HighPassFilter_Mode_Selection;      /* Internal filter mode */
    uint8_t HighPassFilter_CutOff_Frequency;    /* High pass filter cut-off frequency */
    uint8_t HighPassFilter_AOI1;                /* HPF_enabling/disabling for AOI function on interrupt 1 */
    uint8_t HighPassFilter_AOI2;                /* HPF_enabling/disabling for AOI function on interrupt 2 */
    uint8_t HighPassFilter_Data_Sel;
    uint8_t HighPassFilter_Stat;
} ACCELERO_FilterConfigTypeDef;


#endif // __ACCELERO_H
