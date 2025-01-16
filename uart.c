/*
 * UART.c
 *
 *  Created on: 28 oct. 2024
 *      Author: aguil
 */

#include "UART.h"

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
	//USART_WriteByte(USART1, 0x31);
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
