/*
 * encoder.h
 *
 *  Created on: Mar 13, 2026
 *      Author: Vedant
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

void Encoder_Init(void);

int32_t Encoder_GetLeft(void);
int32_t Encoder_GetRight(void);

void Encoder_ResetLeft(void);
void Encoder_ResetRight(void);
void Encoder_ResetBoth(void);
void Encoder_Update(void);

float Encoder_GetDistanceLeft(void);
float Encoder_GetDistanceRight(void);

#endif
