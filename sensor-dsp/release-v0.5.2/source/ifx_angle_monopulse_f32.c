/***************************************************************************//**
* \file ifx_angle_monopulse_f32.c
*
* \brief
* This file contains the implementation for the
* ifx_angle_monopulse_f32 function
*
*******************************************************************************
* \copyright
* Copyright 2022 Infineon Technologies AG
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include "ifx_sensor_dsp.h"

#define ATANHALFF32 0.463648f
#define PIHALFF32 1.5707963267948966192313f

#define ATAN2_NB_COEFS_F32 10

static const float32_t atan2_coefs_f32[ATAN2_NB_COEFS_F32]={0.0f
,1.0000001638308195518f
,-0.0000228941363602264f
,-0.3328086544578890873f
,-0.004404814619311061f
,0.2162217461808173258f
,-0.0207504842057097504f
,-0.1745263362250363339f
,0.1340557235283553386f
,-0.0323664125927477625f
};

static inline float32_t arm_atan_limited_f32(float32_t x)
{
    float32_t res=atan2_coefs_f32[ATAN2_NB_COEFS_F32-1];
    int i=1;
    for(i=1;i<ATAN2_NB_COEFS_F32;i++)
    {
        res = x*res + atan2_coefs_f32[ATAN2_NB_COEFS_F32-1-i];
    }


    return(res);
}

static inline float32_t arm_atan_f32(float32_t x)
{
   int sign=0;
   float32_t res=0.0f;

   if (x < 0.0f)
   {
      sign=1;
      x=-x;
   }

   if (x > 1.0f)
   {
      x = 1.0f / x;
      res = PIHALFF32 - arm_atan_limited_f32(x);
   }
   else
   {
     res += arm_atan_limited_f32(x);
   }


   if (sign)
   {
     res = -res;
   }

   return(res);
}

static arm_status arm_atan2_f32(float32_t y,float32_t x,float32_t *result)
{
    if (x > 0.0f)
    {
        *result=arm_atan_f32(y/x);
        return(ARM_MATH_SUCCESS);
    }
    if (x < 0.0f)
    {
        if (y > 0.0f)
        {
           *result=arm_atan_f32(y/x) + PI;
        }
        else if (y < 0.0f)
        {
           *result=arm_atan_f32(y/x) - PI;
        }
        else
        {
            if (signbit(y))
            {
               *result= -PI;
            }
            else
            {
               *result= PI;
            }
        }
        return(ARM_MATH_SUCCESS);
    }
    if (x == 0.0f)
    {
        if (y > 0.0f)
        {
            *result=PIHALFF32;
            return(ARM_MATH_SUCCESS);
        }
        if (y < 0.0f)
        {
            *result=-PIHALFF32;
            return(ARM_MATH_SUCCESS);
        }
    }


    return(ARM_MATH_NANINF);

}

/*******************************************************************************
* Function Name: ifx_angle_monopulse_f32
****************************************************************************//**
* Description:
* Calculates and gives angle of the direction of arrival of a monopulse of given
* wavelength and with given spacing between two antennas and phase shifts.
* Operates on vectors rx1 and rx2 of given length and updates values in output
* vector of angles.
*******************************************************************************/

arm_status ifx_angle_monopulse_f32(const cfloat32_t* rx1,
                                   const cfloat32_t* rx2,
                                   uint32_t size,
                                   float32_t wavelength,
                                   float32_t antenna_spacing,
                                   float32_t* angle)
{
    assert(rx1 != NULL);
    assert(rx2 != NULL);
    assert(angle != NULL);
    assert(size > 0);
    assert(wavelength > 0);
    assert(antenna_spacing > 0);

    const float32_t TWO_PI = (2.0F * PI);
    const float32_t ratio = wavelength / antenna_spacing / TWO_PI;

    float32_t rx1_ang = 0.0F;
    float32_t rx2_ang = 0.0F;

    uint32_t status = 0U;

    for (uint32_t i = 0; i < size; ++i)
    {
        status |= (uint32_t)arm_atan2_f32(cimagf(rx1[i]), crealf(rx1[i]), &rx1_ang);
        status |= (uint32_t)arm_atan2_f32(cimagf(rx2[i]), crealf(rx2[i]), &rx2_ang);

        float32_t delta_phi = (rx1_ang - rx2_ang);
        if (delta_phi <= -PI)
        {
            delta_phi += TWO_PI;
        }
        else if (delta_phi > PI)
        {
            delta_phi -= TWO_PI;
        }
        else
        {
            //added empty else because of MISRA C-2012 15.7
        }

        status |= (uint32_t)ifx_arcsin_f32(delta_phi * ratio, &(angle[i]));  // in radians
    }

    return (status == 0U) ? ARM_MATH_SUCCESS : ARM_MATH_ARGUMENT_ERROR;
}