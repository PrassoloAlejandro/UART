/*
 * UART.h
 *
 *  Created on: 28 oct. 2024
 *      Author: aguil
 */

#ifndef UART_H_
#define UART_H_

#include "board.h"
#include "fsl_usart.h"
#include "fsl_swm.h"
#include "pin_mux.h"
#include <stdbool.h>

//---------------------------------------------------------------//

//---------------------------------------------------------------//
// Defines
//---------------------------------------------------------------//


//---------------------------------------------------------------//
// Variables
//---------------------------------------------------------------//
uint8_t flagReceived = 0, dataUsart;

char buffer[64]; //Buffer donde cargo la informacion a enviar por USART.
char newline []="\r\n";

//---------------------------------------------------------------//
// Prototipos
//---------------------------------------------------------------//

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




#endif /* UART_H_ */
