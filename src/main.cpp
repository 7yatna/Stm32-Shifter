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
#include "stm32_can.h"
#include "canmap.h"
#include "cansdo.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "mag_snsor.h"
#include "hwinit.h"
#include "anain.h"
#include "param_save.h"
#include "my_math.h"
#include "errormessage.h"
#include "printf.h"
#include "stm32scheduler.h"
#include "terminalcommands.h"
#include "CAN_Common.h"
#define PRINT_JSON 0

extern "C" void __cxa_pure_virtual()
{
    while (1);
}

static Stm32Scheduler* scheduler;
static CanHardware* can;
static CanMap* canMap;


int uauxGain = 223; //!! hard coded AUX gain

int Lock1 = 0;
int Lock2 = 0;
int lock_counter = 0;
int PON_counter = 0;

static void Ms10Task(void)
{
    //Set timestamp of error message
    ErrorMessage::SetTime(rtc_get_counter_val());
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
   // CAN_Common::Task100Ms();
    Param::SetInt(Param::GEAR, (Param::GetInt(Param::gear)));
	MagAngle::ReadMag1Angle();
	/*
	
	if (Param::GetInt(Param::P_LED)) DigIo::P_LED.Set();
	else DigIo::P_LED.Clear();
	
	if (Param::GetInt(Param::R_LED)) DigIo::R_LED.Set();
	else DigIo::R_LED.Clear();
	
	if (Param::GetInt(Param::N_LED)) DigIo::N_LED.Set();
	else DigIo::N_LED.Clear();
	
	if (Param::GetInt(Param::D_LED)) DigIo::D_LED.Set();
	else DigIo::D_LED.Clear();
	
	if (Param::GetInt(Param::BkLT_LED)) DigIo::BkLT_LED.Set();
	else DigIo::BkLT_LED.Clear();
	
	if (Param::GetInt(Param::M1_CW)) DigIo::M1_CW.Set();
	else DigIo::M1_CW.Clear();
	
	if (Param::GetInt(Param::M1_CCW)) DigIo::M1_CCW.Set();
	else DigIo::M1_CCW.Clear();
	
	if (Param::GetInt(Param::M2_CW)) DigIo::M2_CW.Set();
	else DigIo::M2_CW.Clear();
	
	if (Param::GetInt(Param::M2_CCW)) DigIo::M2_CCW.Set();
	else DigIo::M2_CCW.Clear();
	
	if (Param::GetInt(Param::CS1)) DigIo::CS1.Set();
	else DigIo::CS1.Clear();
	
	if (Param::GetInt(Param::CS2)) DigIo::CS2.Set();
	else DigIo::CS2.Clear();

	
	
	
    uint8_t bytes[8];
    bytes[0]=0x05;
    bytes[1]=0x00;
    bytes[2]=0x01;
    bytes[3]=0x10;
    bytes[4]=0x00;
    bytes[5]=0x00;
    bytes[6]=0x00;
    bytes[7]=0x69;
    can->Send(0x380, bytes, 8); //Send on CAN1
		*/
}

static void Ms200Task(void)
{
	Power_ON();
	Set_LED();
	//just for dev
    //MagAngle::ReadMag2Angle(); //just for dev
	//DigIo::CS1.Toggle();
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
		case 0:
			DigIo::P_LED.Set();
			DigIo::R_LED.Clear();
			DigIo::N_LED.Clear();
			DigIo::D_LED.Clear();
			PON_counter++;
			break;
		case 1:
			DigIo::P_LED.Clear();
			DigIo::R_LED.Set();
			DigIo::N_LED.Clear();
			DigIo::D_LED.Clear();
			PON_counter++;
			break;
		case 2:
			DigIo::P_LED.Clear();
			DigIo::R_LED.Clear();
			DigIo::N_LED.Set();
			DigIo::D_LED.Clear();
			PON_counter++;
			break;
		case 3:
			DigIo::P_LED.Clear();
			DigIo::R_LED.Clear();
			DigIo::N_LED.Clear();
			DigIo::D_LED.Set();
			PON_counter++;
			break;
		case 4:
			DigIo::P_LED.Clear();
			DigIo::R_LED.Clear();
			DigIo::N_LED.Clear();
			DigIo::D_LED.Clear();
			DigIo::BkLT_LED.Set();
			PON_counter++;
			break;
		case 5:
			break;
	}
}

void Set_LED()
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


/** This function is called when the user changes a parameter */


void Param::Change(Param::PARAM_NUM paramNum)
{
    switch (paramNum)
    {
		case Param::nodeid:
         //CanSdo->SetNodeId(Param::GetInt(Param::nodeid));
			break; 
		case Param::M1_LOCK:
			//M1_Locking();
	default:
        //Handle general parameter changes here. Add paramNum labels for handling specific parameters
		
        break;
    }
}

static void HandleClear()//Must add the ids to be received here as this set the filters.
{
    can->RegisterUserMessage(0x100);

}

static bool CanCallback(uint32_t id, uint32_t data[2], uint8_t dlc)//Here we decide what to to with the received ids. e.g. call a function in another class etc.
{
    dlc=dlc;
    switch (id)
    {
    case 0x100:
        CAN_Common::HandleCan(data);//can also pass the id and dlc if required to do further work downstream.
        break;
    default:

        break;
    }
    return false;

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
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_CAN1_REMAP_PORTB);//Remap CAN pins to Portb alt funcs.
    AnaIn::Start(); //Starts background ADC conversion via DMA
    write_bootloader_pininit(); //Instructs boot loader to initialize certain pins
    nvic_setup(); //Set up some interrupts
    parm_load(); //Load stored parameters
    Stm32Scheduler s(TIM2); //We never exit main so it's ok to put it on stack
    scheduler = &s;
	
    //Initialize CAN1, including interrupts. Clock must be enabled in clock_setup()
    Stm32Can c(CAN1, CanHardware::Baud500,true);
    FunctionPointerCallback cb(CanCallback, HandleClear);

//store a pointer for easier access
    can = &c;
    //c.SetNodeId(2);
    c.AddCallback(&cb);
    CanMap cm(&c);
    CanSdo sdo(&c, &cm);
    TerminalCommands::SetCanMap(&cm);
    HandleClear();
    sdo.SetNodeId(2);

    canMap = &cm;

    CAN_Common::SetCan(&c);

    Terminal t(USART3, termCmds);
    TerminalCommands::SetCanMap(canMap);


    s.AddTask(Ms10Task, 10);
    s.AddTask(Ms100Task, 100);
	s.AddTask(Ms200Task, 200);

    Param::SetInt(Param::VERSION, 4);
    Param::Change(Param::PARAM_LAST); //Call callback one for general parameter propagation
	PinIntialization();
	

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

