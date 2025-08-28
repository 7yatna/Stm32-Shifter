/*
 * This file is part of the ZombieVeter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "mag_snsor.h"
uint8_t received_data[8];
uint8_t GET1_msg[8];
uint8_t NOP_msg[8];
float f32_angle_degrees = 0.0;
uint16_t u16_angle_lsb = 0;
const float f32_lsb_to_dec_degrees = 0.02197;


void MagAngle::ReadMag1Angle() 
	{	
		GET1_msg[0] = 0x00;
		GET1_msg[1] = 0x00;
		GET1_msg[2] = 0xFF;
		GET1_msg[3] = 0xFF;
		GET1_msg[4] = 0x00;
		GET1_msg[5] = 0x00;
		GET1_msg[6] = 0x13;
		GET1_msg[7] = 0xEA;
		
		DigIo::CS1.Clear();
		
		for (int i = 0; i< 8; i++)
			{
				received_data[i] = spi_xfer(SPI1, GET1_msg[i]);
			}
			
		DigIo::CS1.Set();
		 for (int i = 0; i< 8000; i++)
		 {
		 }
		 
		NOP_msg[0] = 0x00;
		NOP_msg[1] = 0x00;
		NOP_msg[2] = 0xAA;
		NOP_msg[3] = 0xAA;
		NOP_msg[4] = 0x00;
		NOP_msg[5] = 0x00;
		NOP_msg[6] = 0xD0;
		NOP_msg[7] = 0xAB;
		
		DigIo::CS1.Clear();
		
		for (int i = 0; i< 8; i++)
			{
				received_data[i] = spi_xfer(SPI1, NOP_msg[i]);
			}
		DigIo::CS1.Set();

		u16_angle_lsb = ((received_data[1] & 0x3F) << 8);
		u16_angle_lsb = (u16_angle_lsb + received_data[0]);
		f32_angle_degrees = (u16_angle_lsb * f32_lsb_to_dec_degrees);
		Param::SetInt(Param::Angle, f32_angle_degrees);
		
	}


	