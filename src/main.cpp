/*
 * This file is part of the stm32-template project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
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
#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/desig.h>
#include "stm32_can.h"
#include "canmap.h"
#include "cansdo.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "hwinit.h"
#include "anain.h"
#include "param_save.h"
#include "my_math.h"
#include "errormessage.h"
#include "printf.h"
#include "stm32scheduler.h"
#include "terminalcommands.h"
#include "my_string.h"
#include "mag_snsor.h"
#define PRINT_JSON 0

extern "C" void __cxa_pure_virtual()
{
    while (1);
}

static Stm32Scheduler* scheduler;
static CanHardware* can;
static CanMap* canMap;
static CanSdo* canSdo;

int uauxGain = 223; //!! hard coded AUX gain
int Lock1 = 0;
int Lock2 = 0;
int CanLock = 1;
int lock_counter = 0;
int PON_counter = 0;
int LED_RUN = 0;

uint8_t Gcount = 0x00;
uint8_t shiftPos = 0xE1;  // P=0xE1, R=0xD2, N=0xB4, D=0x78, ERR = 0xFF.

static void Ms10Task(void)
{
    //Set timestamp of error message
    ErrorMessage::SetTime(rtc_get_counter_val());
	Param::SetInt(Param::MODE, (Param::GetInt(Param::Mode)));
	Param::SetInt(Param::Brake, (Param::GetInt(Param::Brake_IN)));
	Lock1 = ((float)AnaIn::Lock1.Get());
	Lock2 = ((float)AnaIn::Lock2.Get());
	Param::SetInt(Param::Lock1, Lock1);
	Param::SetInt(Param::Lock2, Lock2);
	M1_Locking();
}


//sample 100ms task
static void Ms100Task(void)
{
    DigIo::led_out.Toggle();
    iwdg_reset();
    Param::SetFloat(Param::VOLTAGE, ((float)AnaIn::Vsense.Get()) / uauxGain);
    float cpuLoad = scheduler->GetCpuLoad();
    Param::SetFloat(Param::CPU_LOAD, cpuLoad / 10);
	canMap->SendAll();
	Can_Tasks();
    Set_Gear();
}

static void Ms200Task(void)
{
	Power_ON();
	Set_LED();
	if ((Param::GetInt(Param::CanCtrl) == 0)  && (CanLock))
	{
		Param::SetInt(Param::Mode, 0);
		Param::SetInt(Param::Brake_IN, 0);
		CanLock = 0;
	}
}

void Set_Gear()
{
	MagAngle::ReadMag1Angle();
	switch (Param::GetInt(Param::Angle))
	{
		case 340 ... 360:
			Param::SetInt(Param::GEAR, 4);
			shiftPos = 0xFF; //Error
		break;
		case 298 ... 339:
			Param::SetInt(Param::GEAR, 4);
			if (Param::GetInt(Param::MODE)) Param::SetInt(Param::GEAR, 3);
			shiftPos = 0x78; //Drive
		break;
		case 253 ... 297:
			Param::SetInt(Param::GEAR, 4);
			if (Param::GetInt(Param::MODE)) Param::SetInt(Param::GEAR, 2);
			shiftPos = 0xB4; //Neutral
		break;
		case 205 ... 252:
			Param::SetInt(Param::GEAR, 4);
			if (Param::GetInt(Param::MODE)) Param::SetInt(Param::GEAR, 1);
			shiftPos = 0xD2; //Reverse
		break;
		case 175 ... 204:
			Param::SetInt(Param::GEAR, 0);
			Param::SetInt(Param::M1_LOCK, 1);
			if ((Param::GetInt(Param::Brake)) && (Param::GetInt(Param::MODE))) Param::SetInt(Param::M1_LOCK, 0);
			shiftPos = 0xE1; //Park
		break;
		case 0 ... 174:
			Param::SetInt(Param::GEAR, 4);
			shiftPos = 0xFF; //Error
		break;
	}
}

void Can_Tasks()
{
	
	uint8_t bytes[8];
    bytes[0]= shiftPos;
    bytes[1]= 0x00;
	if (Param::GetInt(Param::KNOB_LOCK)) bytes[1] = 0xFF;
    bytes[2]= Gcount;
    bytes[3]= shiftPos + 0x0A;
    
    can->Send(0x1D2, bytes, 4); //Send on CAN1	
	
	Gcount = Gcount + 0x01;
   
   if (Gcount==0xFF)
   {
      Gcount=0x00;
   }
}
	
void DecodeCAN(int id, uint32_t* data)
{
	uint8_t* bytes = (uint8_t*)data;
	switch (id)
	{
		case 0x1AE:
			Param::SetInt(Param::Mode, bytes[0]);
			Param::SetInt(Param::Brake_IN, bytes[1]);
			break;
	}
			
}
		
void PinIntialization()
{
	DigIo::P_LED.Clear();
	DigIo::R_LED.Clear();
	DigIo::N_LED.Clear();
	DigIo::D_LED.Clear();
	DigIo::BkLT_LED.Clear();
	DigIo::M1_CW.Clear();
	DigIo::M1_CCW.Clear();
	DigIo::M2_CW.Clear();
	DigIo::M2_CCW.Clear();
	DigIo::CS1.Set();
	DigIo::CS2.Set();
	DigIo::BkLT_LED.Set();
	Param::SetInt(Param::GEAR, 4);
}

void Power_ON()
{
	switch(PON_counter)
	{
		case 0 ... 1:
			DigIo::P_LED.Set();
			DigIo::R_LED.Clear();
			DigIo::N_LED.Clear();
			DigIo::D_LED.Clear();
			PON_counter++;
			break;
		case 2 ... 3:
			DigIo::P_LED.Clear();
			DigIo::R_LED.Set();
			DigIo::N_LED.Clear();
			DigIo::D_LED.Clear();
			PON_counter++;
			break;
		case 4 ... 5:
			DigIo::P_LED.Clear();
			DigIo::R_LED.Clear();
			DigIo::N_LED.Set();
			DigIo::D_LED.Clear();
			PON_counter++;
			break;
		case 6 ... 7:
			DigIo::P_LED.Clear();
			DigIo::R_LED.Clear();
			DigIo::N_LED.Clear();
			DigIo::D_LED.Set();
			PON_counter++;
			break;
		case 8 ... 9:
			DigIo::P_LED.Clear();
			DigIo::R_LED.Clear();
			DigIo::N_LED.Clear();
			DigIo::D_LED.Clear();
			DigIo::BkLT_LED.Set();
			PON_counter++;
			break;
		case 10:
		LED_RUN = 1;
			break;
	}
}

void Set_LED()
{
	if (LED_RUN)
	{
		switch(Param::GetInt(Param::GEAR))
		{
			case 0:
				DigIo::P_LED.Set();
				DigIo::R_LED.Clear();
				DigIo::N_LED.Clear();
				DigIo::D_LED.Clear();
				break;
			case 1:
				DigIo::P_LED.Clear();
				DigIo::R_LED.Set();
				DigIo::N_LED.Clear();
				DigIo::D_LED.Clear();
				break;
			case 2:
				DigIo::P_LED.Clear();
				DigIo::R_LED.Clear();
				DigIo::N_LED.Set();
				DigIo::D_LED.Clear();
				break;
			case 3:
				DigIo::P_LED.Clear();
				DigIo::R_LED.Clear();
				DigIo::N_LED.Clear();
				DigIo::D_LED.Set();
				break;		
			case 4:
				DigIo::P_LED.Toggle();
				DigIo::R_LED.Toggle();
				DigIo::N_LED.Toggle();
				DigIo::D_LED.Toggle();
				break;
		}
	}
}

void M1_Locking()
{
	if (Lock1 > 2600)
	{
	switch(Param::GetInt(Param::M1_LOCK))
		{
			case 0:
					DigIo::M1_CW.Clear();
					DigIo::M1_CCW.Clear();
					Param::SetInt(Param::KNOB_LOCK, 0);
					break;
			case 1:
				switch (lock_counter)
					{
						case 0:
						DigIo::M1_CW.Set();
						DigIo::M1_CCW.Clear();
						lock_counter++;
							break;
						case 1:
						lock_counter++;
							break;
						case 2:
						lock_counter++;
							break;
						case 3:
						lock_counter++;
							break;
						case 4:
						DigIo::M1_CW.Clear();
						DigIo::M1_CCW.Clear();
						lock_counter = 0;
						Param::SetInt(Param::KNOB_LOCK, 1);
							break;
					}	
				break;
		}
		
	}
	if (Lock1 < 2600)
	{	
	switch(Param::GetInt(Param::M1_LOCK))
		{
			case 0:
				switch (lock_counter)
					{
						case 0:
						DigIo::M1_CW.Clear();
						DigIo::M1_CCW.Set();
						lock_counter++;
							break;
						case 1:
						lock_counter++;
							break;
						case 2:
						lock_counter++;
							break;
						case 3:
						lock_counter++;
							break;
						case 4:
						DigIo::M1_CW.Clear();
						DigIo::M1_CCW.Clear();
						lock_counter = 0;
						Param::SetInt(Param::KNOB_LOCK, 0);
							break;
					}
						
					break;
			case 1:
					DigIo::M1_CW.Clear();
					DigIo::M1_CCW.Clear();
					Param::SetInt(Param::KNOB_LOCK, 1);
				break;
		}
	}
}


static void SetCanFilters()
{
	can->RegisterUserMessage(0x601); //Can SDO
	can->RegisterUserMessage(0x1AE); //OI Control Message
}
	
static bool CanCallback(uint32_t id, uint32_t data[2], uint8_t dlc) //This is where we go when a defined CAN message is received.
{
    dlc = dlc;
	if (Param::GetInt(Param::CanCtrl)) DecodeCAN(id,data);
	else
	{
		Param::SetInt(Param::MODE, (Param::GetInt(Param::Mode)));
		Param::SetInt(Param::Brake, (Param::GetInt(Param::Brake_IN)));
	}		
		
	return false;
}

/** This function is called when the user changes a parameter */


void Param::Change(Param::PARAM_NUM paramNum)
{
    switch (paramNum)
    {
		case Param::NodeId:
			canSdo->SetNodeId(Param::GetInt(Param::NodeId));
			break; 
		case Param::M1_LOCK:
			break;
		case Param::CanCtrl:
			SetCanFilters();
			break;
	default:
        //Handle general parameter changes here. Add paramNum labels for handling specific parameters
		
        break;
    }
}

//Whichever timer(s) you use for the scheduler, you have to
//implement their ISRs here and call into the respective scheduler
extern "C" void tim2_isr(void)
{
    scheduler->Run();
}

extern "C" int main(void)
{
    extern const TERM_CMD termCmds[];

    clock_setup(); //Must always come first
    rtc_setup();
    ANA_IN_CONFIGURE(ANA_IN_LIST);
    DIG_IO_CONFIGURE(DIG_IO_LIST);
	AnaIn::Start(); //Starts background ADC conversion via DMA
    write_bootloader_pininit(); //Instructs boot loader to initialize certain pins
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_CAN1_REMAP_PORTB);//Remap CAN pins to Portb alt funcs.
    nvic_setup(); //Set up some interrupts
    parm_load(); //Load stored parameters
	spi1_setup();											
    Stm32Scheduler s(TIM2); //We never exit main so it's ok to put it on stack
    scheduler = &s;
    //Initialize CAN1, including interrupts. Clock must be enabled in clock_setup()
    Stm32Can c(CAN1, CanHardware::Baud500,true);
	FunctionPointerCallback cb(CanCallback, SetCanFilters);
	CanMap cm(&c);
	CanSdo sdo(&c, &cm);
	sdo.SetNodeId(Param::GetInt(Param::NodeId));
	//store a pointer for easier access
	can = &c;
	canMap = &cm;
    canSdo = &sdo;
	c.AddCallback(&cb);
    Terminal t(USART3, termCmds);
    TerminalCommands::SetCanMap(canMap);
  
    s.AddTask(Ms10Task, 10);
    s.AddTask(Ms100Task, 100);
	s.AddTask(Ms200Task, 200);

    Param::SetInt(Param::version, 4);
    Param::Change(Param::PARAM_LAST); //Call callback one for general parameter propagation
	PinIntialization();
	SetCanFilters();
	if (Param::GetInt(Param::CanCtrl) == 0)
	{
		Param::SetInt(Param::Mode, 0);
		Param::SetInt(Param::Brake_IN, 0);
	}
	

    while(1)
    {
        char c = 0;
        t.Run();
        if (sdo.GetPrintRequest() == PRINT_JSON)
        {
            TerminalCommands::PrintParamsJson(&sdo, &c);
        }
    }

    return 0;
}

