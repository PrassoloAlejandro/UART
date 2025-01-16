/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    UART.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "board.h"
#include "fsl_usart.h"
#include "fsl_swm.h"
#include "pin_mux.h"
#include <stdbool.h>
#include "fsl_i2c.h"
/* TODO: insert other definitions and declarations here. */

#define SHT30_ADDRESS	 			0x45	// Address SHT30


//---------------------------------------------------------------//
// Variables
//---------------------------------------------------------------//
uint8_t flagReceived = 0, dataUsart;

char buffer[64]; //Buffer donde cargo la informacion a enviar por USART.
char newline []="\r\n";

volatile uint16_t flagTick = 0;
uint32_t baudRate = 50000;
uint32_t frequency = 1000000;

//---------------------------------------------------------------//
// Prototipos
//---------------------------------------------------------------//

void delay_ms(int msec);
void initI2C(void);

/*
 * Inicializo UART en los pines 25 y 24, los cuales pueden ser modificados
 * La configuracion del UART es por defecto
 */
void Init_UART(void);

/*Transmito por USART
 *
 *  *base: Base de la USART que uso, en este caso USART1
 *  data: informacion a enviar
 *  texto: indico formato y texto que quiero enviar. Debe estar como "Temp: %d"
 *
 */

void Send_USART(USART_Type *base, uint8_t data, char texto);


/*
 * Si recibo informacion por USART desde la computadora, la leo y
 * almaceno en dataUsart.
 */
void USART1_IRQHandler(void);



int32_t r;
uint8_t x[6];
float temp=0; float humidity=0;



/*
 * @brief   Application entry point.
 */
int main(void) {

	Init_UART();
	initI2C();


	if (kStatus_Success == I2C_MasterStart(I2C1_BASE, SHT30_ADDRESS, kI2C_Write)){
		/*
		 * Asi envia en arduino
		 *
		 * Send measurement command
			Wire.write(0x2C); clock stretching enable
			Wire.write(0x06); High repeatability

			Por datasheet sería: 0x2C06: high repeatability measurement with clock
			stretching enabled

		*/

		x[0] = 0x24; //high repeatability measurement whithout clock streatching
		x[1] = 0x00;
		r = I2C_MasterWriteBlocking(I2C1_BASE, x, 2, 0);
		r = I2C_MasterStop(I2C1_BASE);

		if (kStatus_Success == I2C_MasterRepeatedStart(I2C1_BASE, SHT30_ADDRESS, kI2C_Read))
		{

			r = I2C_MasterReadBlocking(I2C1_BASE, x, 6, 0);
			r = I2C_MasterStop(I2C1_BASE);

			uint16_t dividendo = 65535;
			uint16_t divisortemp =(x[0]*256)+x[1];
			uint16_t divisorhum = (x[3]*256)+x[4];

			temp= -45 + 175*(1.0*divisortemp/dividendo); //((((x[0] * 256.0) + x[1]) * 175) / 65535.0) - 45
			humidity= 100*(1.0*divisorhum/dividendo); //((((x[3] * 256.0) + x[4]) * 100) / 65535.0)

			PRINTF("Temperatura : %i \r\n",temp);
			PRINTF("Humedad : %i \r\n", humidity);
		}
	}
	sprintf(buffer, "Temperatura: %i",(int)temp);
	//Send_USART( USART1, &buffer, strlen(buffer));
	USART_WriteBlocking(USART1, &buffer, strlen(buffer));
	USART_WriteBlocking(USART1, &newline, strlen(newline));

	sprintf(buffer, "Humedad: %i",(int)humidity);
	USART_WriteBlocking(USART1, &buffer, strlen(buffer));



	return 0;
}




/*
 * Inicializo UART en los pines 25 y 24, los cuales pueden ser modificados
 * La configuracion del UART es por defecto
 */
void Init_UART(void){
	// Habilito clock de matriz de conmutacion
	CLOCK_EnableClock(kCLOCK_Swm);
	// USART1_TXD es una funcion movible y la asigno al pin P0_25, puedo cambiar de pin si es necesario.
	SWM_SetMovablePinSelect(SWM0, kSWM_USART1_TXD, kSWM_PortPin_P0_25);	    // USART1_TXD connect to P0_25
	// USART1_RXD es una funcion movible y la asigno al pin P0_24, puedo cambiar de pin si es necesario.
	SWM_SetMovablePinSelect(SWM0, kSWM_USART1_RXD, kSWM_PortPin_P0_24);		// USART1_RXD connect to P0_24
	// Deshabilito clock de matriz de conmutacion
	CLOCK_DisableClock(kCLOCK_Swm);
	CLOCK_Select(kUART1_Clk_From_MainClk);
	/* Default config by using USART_GetDefaultConfig():
	 * config->baudRate_Bps = 9600U;
	 * config->parityMode = kUSART_ParityDisabled;
	 * config->stopBitCount = kUSART_OneStopBit;
	 * config->bitCountPerChar = kUSART_8BitsPerChar;
	 * config->loopback = false;
	 * config->enableRx = false;
	 * config->enableTx = false;
	 * config->syncMode = kUSART_SyncModeDisabled;
	 */
	usart_config_t config;
	USART_GetDefaultConfig(&config);
	/*En caso de querer cambiar el baudrate:
	 * config.baudRate_Bps = VALOR DECEADO
	 */

	//Habilito pines de Tx y Rx
	config.enableRx     = true;
	config.enableTx     = true;

	 /* Initialize the USART with configuration. */
	USART_Init(USART1, &config, CLOCK_GetFreq(kCLOCK_MainClk));
	//Habilito la interrupcion para USART
	USART_EnableInterrupts(USART1, kUSART_RxReadyInterruptEnable);
	NVIC_EnableIRQ(USART1_IRQn);

	//Escribo para verificar la conexión
	USART_WriteByte(USART1, 0x31);
	return;
}




/*Transmito por USART
 *
 *  *base: Base de la USART que uso, en este caso USART1
 *  data: informacion a enviar
 *  texto: indico formato y texto que quiero enviar. Debe estar como "Temp: %d"
 *
 */

void Send_USART(USART_Type *base, uint8_t data, char texto){
	//Cargo los datos al buffer
	sprintf(buffer, texto,data);
	USART_WriteBlocking(base,&buffer,strlen(buffer));

	flagReceived = 0;
	return;
}


/*--------------------------------------
 * Delay Function
 *--------------------------------------*/
void delay_ms(int msec) {
    flagTick = 0;
    while (flagTick <= msec);
}



/*--------------------------------------
 * I2C Initialization Function
 *--------------------------------------*/
void initI2C(void) {
    BOARD_InitDebugConsole();

    CLOCK_Select(kI2C1_Clk_From_MainClk);
    CLOCK_EnableClock(kCLOCK_Swm);
    SWM_SetMovablePinSelect(SWM0, kSWM_I2C1_SDA, kSWM_PortPin_P0_27); // Pin 13
    SWM_SetMovablePinSelect(SWM0, kSWM_I2C1_SCL, kSWM_PortPin_P0_26); // Pin 12
    CLOCK_DisableClock(kCLOCK_Swm);

    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = baudRate;
    I2C_MasterInit(I2C1_BASE, &masterConfig, frequency);
}



//---------------------------------------------------------------//
//Interrupción
//---------------------------------------------------------------//
/*
 * Si recibo informacion por USART desde la computadora, la leo y
 * almaceno en dataUsart.
 */
//---------------------------------------------------------------//
void USART1_IRQHandler(void){
	dataUsart = USART_ReadByte(USART1);
	flagReceived = 1;
	return;
}
//---------------------------------------------------------------//


/*--------------------------------------
 * Interrupt Handler
 *--------------------------------------*/
void SysTick_Handler(void) {
    flagTick++;
}
