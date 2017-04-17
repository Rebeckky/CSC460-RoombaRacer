/* Authors: Chris Cook and Kevin Gill 
 * CSC460 2016 Roomba Tank
 *
 *
 * Modified for CSC460 Spring 2017
 * Modified by: Becky Croteau
 * Modifications: Roomba_QueryList(), Roomba_Song()
 *	Port and Pins in Roomba_Init()
 */
#define F_CPU 16000000

#include "roomba.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 
#include "../uart/uart.h" 

#define ROOMBA_PORT PORTC
#define ROOMBA_PIN PC7 // digital pin 30

void Roomba_Init(){
   
    // Wake Roomba from sleep
    // Pin 30

    DDRC |= (1<<ROOMBA_PIN);
    ROOMBA_PORT |= (1<<ROOMBA_PIN);
    
    ROOMBA_PORT &= ~(1<<ROOMBA_PIN);
    
    ROOMBA_PORT |= (1<<ROOMBA_PIN);
    
    // start the OI
    Roomba_Send_Byte(START);

    _delay_ms(2100);

    ROOMBA_PORT |= (1<<ROOMBA_PIN);
    ROOMBA_PORT &= ~(1<<ROOMBA_PIN);
    _delay_ms(100);

    ROOMBA_PORT |= (1<<ROOMBA_PIN);
    ROOMBA_PORT &= ~(1<<ROOMBA_PIN);
    _delay_ms(100);

    ROOMBA_PORT |= (1<<ROOMBA_PIN);
    ROOMBA_PORT &= ~(1<<ROOMBA_PIN);
    _delay_ms(100);

    // set Roomba to safe mode
    Roomba_Send_Byte(SAFE_MODE); 

    Roomba_Song(0); // Initialize song 0 
}

void Roomba_Drive(int16_t velocity, int16_t radius) {    
    Roomba_Send_Byte(DRIVE);
    Roomba_Send_Byte(velocity>>8);
    Roomba_Send_Byte(velocity);
    Roomba_Send_Byte(radius>>8);
    Roomba_Send_Byte(radius);   
}

void Roomba_Play(uint8_t song) {
	Roomba_Send_Byte(PLAY);
	Roomba_Send_Byte(song);
}

void Roomba_Sensors(uint8_t packet_id) {
	Roomba_Send_Byte(SENSORS);
	Roomba_Send_Byte(packet_id);
}

void RoombaQueryList(uint8_t packet1,uint8_t packet2, uint8_t packet3)
{
	Roomba_Send_Byte(QUERYLIST);
	Roomba_Send_Byte(3);
	Roomba_Send_Byte(packet1);
	Roomba_Send_Byte(packet2);
	Roomba_Send_Byte(packet3);
}

void Roomba_Song(uint8_t n) {
	Roomba_Send_Byte(SONG);
	Roomba_Send_Byte(n);  // Song 0
	Roomba_Send_Byte(9);  // 9 Notes
	//Imperial March
	for(int i = 0; i < 3; i++)
	{		
		Roomba_Send_Byte(64); // E
		Roomba_Send_Byte(32);
	}
	for(int j = 0; j < 2; j++)
	{
		Roomba_Send_Byte(60); // C
		Roomba_Send_Byte(16);
		Roomba_Send_Byte(67); // G
		Roomba_Send_Byte(22);
		Roomba_Send_Byte(64); // E
		Roomba_Send_Byte(24);
	}
}