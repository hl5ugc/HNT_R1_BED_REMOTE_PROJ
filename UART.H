//
// =======================================================================
// [ PROJECT   ] R1 BED REMOTE CONTROL 
// -----------------------------------------------------------------------
// [ License   ] SAMJIN ELECTRONICS
// [ Author    ] Copyright 2021-08 By HAG-SEONG KANG
// [ E-MAIL    ] hl5ugc@nate.com (82)10- 3841-9706
// [ C  P  U   ]  
// [ Compller  ] CodeWizardAVR V3.12 Professional
// [ Filename  ] UART.H
// [ Version   ] 1.0
// [ Created   ] 2021-08-10
// ----------------------------------------------------------------------------
// Revision History :
// ----------------------------------------------------------------------------
// Author         Date          Comments on this revision    
// ----------------------------------------------------------------------------
// HAG-SEONG KANG 2022-09-08    First release of header file   
// 
// 
// ============================================================================
//
//
//
//
/* Mutiple includes protection */
#ifndef _UART_header
#define _UART_header

#pragma used+
/* Includes */
#include "SJTYPE.H"  
 

// ---------------------------------------------------------------------------
// Basic Define  
// --------------------------------------------------------------------------- 
// 
#define USART_BPS       19200 
#define PLC_DATA_MAX    10 
// ---------------------------------------------------------------------------
// Define typedef.
// --------------------------------------------------------------------------- 
//
typedef struct
{
    uint8_t bit0          : 1 ;
    uint8_t bit1          : 1 ;
    uint8_t bit2          : 1 ;
    uint8_t bit3          : 1 ;
    uint8_t bit4          : 1 ;
    uint8_t bit5          : 1 ;
    uint8_t bit6          : 1 ;
    uint8_t bit7          : 1 ;    
}   REMOTE_DATA_BITS ; 
//
typedef union _REMOTE_Flag  
{
    REMOTE_DATA_BITS bits ;
    uint8_t valus   ;
    
}  REMOTE_FLAG_t ; 
// ---------------------------------------------------------------------------
//  Define union Varibales.
// ---------------------------------------------------------------------------
//
extern REMOTE_FLAG_t        REMOTE_cmd1_flag ;
//
// ---------------------------------------------------------------------------
//  Define union Macro.
// ---------------------------------------------------------------------------
//
//
#define REMOTE_UART_OK_Flag_On              REMOTE_cmd1_flag.bits.bit0 = 1 
#define REMOTE_UART_OK_Flag_Off             REMOTE_cmd1_flag.bits.bit0 = 0 
#define Is_REMOTE_UART_OK_Flag              REMOTE_cmd1_flag.bits.bit0 == 1 
#define Is_Not_REMOTE_UART_OK_Flag          REMOTE_cmd1_flag.bits.bit0 == 0
// 
// --------------------------------------------------------------------------- 
// Global Variables Define
// ----------------------------------------------------------------------------
//
extern vuint8_t PLC_DATA[PLC_DATA_MAX] ;
#define KEY_DATA_BUFF1  PLC_DATA[0]
#define KEY_DATA_BUFF2  PLC_DATA[1]
// PLC_DATA[5].1  = Error
// PLC_DATA[5].2  = OK
// PLC_DATA[5].3  = P1
// PLC_DATA[5].4  = P2
// PLC_DATA[5].5  = P3

#define LED_DATA_BUFF   PLC_DATA[5]
// ---------------------------------------------------------------------------
// Define  Function prototypes.
// --------------------------------------------------------------------------- 
//
void Uart_Init(void) ;
void send_STX(void) ;
void Uart_Check(void) ;
void send_1Byte(uint8_t u8data);
void send_Write_Data_Bytes(uint8_t u8Address,uint8_t u8Count); 

// ---------------------------------------------------------------------------
// Define  Callback protoetypes.
// --------------------------------------------------------------------------- 
//
 


 
#pragma used-
#endif /* SAMJIN_header */ 