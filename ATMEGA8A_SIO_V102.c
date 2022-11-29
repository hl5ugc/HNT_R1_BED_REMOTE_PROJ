// =======================================================================
// [ PROJECT   ]
// -----------------------------------------------------------------------
// [ License   ] SAMJIN ELECTRONICS
// [ Author    ] Copyright 2021.8 By HAG-SEONG KANG (hl5ugc@nate.com)
// [ HomePage  ]
// [ C  P  U   ]
// [ Compller  ] CodeWizardAVR V13.12 Professional
// [ Filename  ] ATMEGA8A_SIO.C
// [ Version   ] 0.5
// [ Created   ] 2022/09/08
// [ Modified  ]
// ------------------------------------------------------------------
// [Updata  List] :
// HAG-SEONG KANG 2022-10-13    OSC 16MHz Modify
//
//
//
//
// ==================================================================
//
//   
//  
//
#pragma used+
#include <mega8.h>
#include <delay.h>
#include "uart.h" 
 
// ---------------------------------------------------------------------------
// typedef define
// --------------------------------------------------------------------------- 
 
// ---------------------------------------------------------------------------
// Structure and Union Definitions
// ---------------------------------------------------------------------------        
// ---------------------------------------------------------------------------
// UART defines
// ---------------------------------------------------------------------------
// 
#define STX         '@'
#define ETX         '$'
//
#define PC_STX      '@' 
#define PC_ETX      '$' 
#define PC_ACK      '#'
#define ALL_ADD     '?' 
//

#define OVR 3

#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<OVR)
#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE (1<<RXC)
//
#define UART_TX_BUF_MAX     30
#define UART_RX_BUF_MAX     30 
//
// --------------------------------------------------------------------------- 
// UART Variables Define
// ----------------------------------------------------------------------------
// 
vuint8_t Uart_Tx_Buff[UART_TX_BUF_MAX] ;
uint8_t Uart_Tx_Wr_Ptr  ;
uint8_t Uart_Tx_Rd_Ptr  ;
//uint8_t Uart_Tx_Event   ;
//uint8_t Uart_Tx_Count   ;
//
vuint8_t Uart_Rx_Buff[UART_RX_BUF_MAX] ;
uint8_t Uart_Rx_Wr_Ptr  ;
//uint8_t Uart_Rx_Rd_Ptr  ;
uint8_t Uart_Rx_Event   ;
uint8_t Uart_Rx_Wait    ;
//
//
// ---------------------------------------------------------------------------
// Variables Define
// --------------------------------------------------------------------------- 
//
REMOTE_FLAG_t        REMOTE_cmd1_flag ;
vuint8_t PLC_DATA[PLC_DATA_MAX] ;

// ---------------------------------------------------------------------------
// Define Function prototypes.
// --------------------------------------------------------------------------- 
//
static uint8_t  hex2byte_To_Bin(uint8_t u8Address) ;
static void     send_Bin_2AsciiHex(uint8_t u8Data);
static void     send_AsciiByte(uint8_t u8Data) ;
// ---------------------------------------------------------------------------
// GLOBAL Function Routine Start
// --------------------------------------------------------------------------- 
//
//   
/** -------------------------------------------------------------------------------------
 * @brief UART Input Data Parsing Routine
 * 
 * data Frame  STX ,CMD ,Mem,Count, HEX_ASCII (2Byte), ETX
------------------------------------------------------------------------------------- */
void Uart_Check(void)  
{
    static uint8_t  i_temp  = 0;
    static uint8_t  address = 0;
    static uint8_t  counter = 0; 
    //static uint16_t check = 0;
    // 
    if(Uart_Rx_Event == 1)          // Check Data Frame END
    {
        Uart_Rx_Event = 0 ;         //
        if(Uart_Rx_Buff[0] == STX)  // Check Data Frame Start
        { 
            REMOTE_UART_OK_Flag_On ;
            switch(Uart_Rx_Buff[1])
            {
                case 'W' :  // Memory Write Command
                case 'w' :
                            address =  Uart_Rx_Buff[2] & 0x0F ;        
                            counter =  Uart_Rx_Buff[3] & 0x0F ;
                            i_temp = address + counter ;
                            if(i_temp <= PLC_DATA_MAX)
                            {
                                //
                                for(i_temp=0; i_temp<counter; i_temp++)
                                {
                                    PLC_DATA[address+i_temp] = hex2byte_To_Bin(i_temp*2 + 4)  ; 
                                }
                            }                             
                            break ;
                case 'R' :  // Memory Read Command
                case 'r' :
                            address =  Uart_Rx_Buff[2] & 0x0F ;        
                            counter =  Uart_Rx_Buff[3] & 0x0F ;
                            i_temp = address + counter ;
                            if(i_temp <= PLC_DATA_MAX)
                            {
                                send_1Byte('W');
                                send_1Byte(Uart_Rx_Buff[2]);    // Address
                                send_1Byte(Uart_Rx_Buff[3]);    // Counter
                                
                                for(i_temp=0; i_temp<counter; i_temp++)
                                {
                                    send_Bin_2AsciiHex(PLC_DATA[address+i_temp]);     
                                }
                            }      
                            //
                            send_1Byte(ETX); 
                            send_STX() ;
                            break ;            
            
                default :
                            break ; 
            }
        }
    }
}

void send_Write_Data_Bytes(uint8_t u8Address,uint8_t u8Count) 
{
    static uint8_t u8Temp  = 0;
    while(Uart_Tx_Wr_Ptr != Uart_Tx_Rd_Ptr ) 
    {
        delay_ms(1) ;
    } 
    //
    send_1Byte('W');
    send_AsciiByte(u8Address);
    send_AsciiByte(u8Count);   
    //
    for(u8Temp = 0; u8Temp < u8Count; u8Temp++)
    {
        send_Bin_2AsciiHex(PLC_DATA[u8Address+u8Temp]);
    } 
    send_1Byte(ETX);
    // 
    send_STX() ;
  
} 

/** -------------------------------------------------------------------------------------
 * @brief   Send 1byte Data To Uart Buffer
 * 
 * @param   u8data // send bayte
 ------------------------------------------------------------------------------------- */
void send_1Byte(uint8_t u8data)
{   
    if(Uart_Tx_Wr_Ptr < UART_TX_BUF_MAX)
    {
        Uart_Tx_Buff[Uart_Tx_Wr_Ptr] =  u8data  ;
        if(++Uart_Tx_Wr_Ptr == UART_TX_BUF_MAX) Uart_Tx_Wr_Ptr = 0;  
    }
}

/** -------------------------------------------------------------------------------------
 * @brief   Send Uart Start Text Code
 * 
------------------------------------------------------------------------------------- */
void send_STX(void)
{                   
    //Txd_output_enable ;
    //delay_us(5);
    UDR = STX ;
} 
// ---------------------------------------------------------------------------
// Local Function Routine Start
// --------------------------------------------------------------------------- 
//
/** -------------------------------------------------------------------------------------
 * @brief   HEXASCII 2Byte Data to Byte Bin Data Convertion 
 * 
 * @param   u8Address Input Buffer HEXASCII 2Byte Data Start Address
 * @return  uint8_t   1byte Convertion Data
------------------------------------------------------------------------------------- */
static uint8_t hex2byte_To_Bin(uint8_t u8Address)  
{
    static uint8_t  temp     = 0 ;
    static uint8_t  cal_temp = 0 ;
    // 
    if(u8Address < PLC_DATA_MAX)
    {
        temp = Uart_Rx_Buff[u8Address] ;
        if(temp < 0x3A) temp = temp - 0x30 ;
        else temp = temp - 0x37 ;
        cal_temp = temp << 4 ; 
        cal_temp = cal_temp & 0xF0 ;
        //
        u8Address++ ;
        temp = Uart_Rx_Buff[u8Address] ; 
        if(temp < 0x3A) temp = temp - 0x30 ;
        else temp = temp - 0x37 ;
        cal_temp = cal_temp + temp   ; 
    }
    // 
    return  cal_temp ;
}
/** -------------------------------------------------------------------------------------
 * @brief   Bin  1byte Dta to 2Byte HexAscii Convert and Uart Buffer save for Send UART
 * 
 * @param   u8Data send Bin 1byte data 
------------------------------------------------------------------------------------- */
static void send_Bin_2AsciiHex(uint8_t u8Data) 
{
    static uint8_t i_Temp = 0 ; 
    //
    i_Temp = (u8Data >> 4) & 0x0F ;
    if(i_Temp <= 9) send_1Byte(i_Temp + '0') ;
    else            send_1Byte((i_Temp - 10) + 'A') ;
    //
    i_Temp =  u8Data & 0x0F ;
    if(i_Temp <= 9) send_1Byte(i_Temp + '0') ;
    else            send_1Byte((i_Temp - 10) + 'A') ;
}
/** ------------------------------------------------------------------------------------- 
 * @brief   Send 1byte(0x00 ~ 0x0F) Bin Data to ASscii Data Convert and Send UART
 * 
 * @param   u8data Bin Data(0x00 ~ 0x0F) For Send
-------------------------------------------------------------------------------------  */
static void send_AsciiByte(uint8_t u8Data)
{   
    static uint8_t i_Temp = 0 ; 
    //
    i_Temp =  u8Data & 0x0F ;
    if(i_Temp <= 9) send_1Byte(i_Temp + '0') ;
    else            send_1Byte((i_Temp - 10) + 'A') ;
}

// ---------------------------------------------------------------------------
// USART Receiver interrupt service routine
// --------------------------------------------------------------------------- 
//
interrupt [USART_RXC] void uart_rx_isr(void)
{

    static uint8_t status = 0 ; 
    static uint8_t data = 0 ;
    //
    status= UCSRA  ; 
    data  = UDR  ; 
    //
    if(data == STX)
    {
        Uart_Rx_Wr_Ptr  = 0 ;
        Uart_Rx_Event   = 0 ;
    }
    else if(data == ETX) 
    {
        Uart_Rx_Event   = 1 ;
    }    
    else ;
    //-------------------------------------------------------------
    if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
    {   
        if(Uart_Rx_Wr_Ptr < UART_RX_BUF_MAX)
        {
            Uart_Rx_Buff[Uart_Rx_Wr_Ptr] = data ;
            if (++Uart_Rx_Wr_Ptr >= UART_RX_BUF_MAX) Uart_Rx_Wr_Ptr = 0;         
        }
    } 
    // 
    Uart_Rx_Wait = 3 ; //
} 
// ---------------------------------------------------------------------------
// USART Transmitter interrupt service routine
// ---------------------------------------------------------------------------
//
interrupt [USART_TXC] void uart_tx_isr(void)
{
    if(Uart_Tx_Wr_Ptr != Uart_Tx_Rd_Ptr)
    {             
        if(Uart_Tx_Rd_Ptr < UART_TX_BUF_MAX) 
        {
            UDR = Uart_Tx_Buff[Uart_Tx_Rd_Ptr]; 
            if(++Uart_Tx_Rd_Ptr >= UART_TX_BUF_MAX) Uart_Tx_Rd_Ptr = 0; 
        }  
    }  

}
// ---------------------------------------------------------------------------
// Uart Port Init
// --------------------------------------------------------------------------- 
//
void Uart_Init(void)
{
#if (USART_BPS == 9600)
    // USART initialization
    // Communication Parameters: 8 Data, 1 Stop, No Parity
    // USART Receiver: On
    // USART Transmitter: On
    // USART Mode: Asynchronous
    // USART Baud Rate: 9600
    UCSRA=(0<<RXC) | (0<<TXC) | (0<<UDRE) | (0<<FE) | (0<<DOR) | (0<<UPE) | (0<<U2X) | (0<<MPCM);
    UCSRB=(1<<RXCIE) | (1<<TXCIE) | (0<<UDRIE) | (1<<RXEN) | (1<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);
    UCSRC=(1<<URSEL) | (0<<UMSEL) | (0<<UPM1) | (0<<UPM0) | (0<<USBS) | (1<<UCSZ1) | (1<<UCSZ0) | (0<<UCPOL);
    UBRRH=0x00;
    UBRRL=0x67;     
    // USART initialization   internal OSC 8Mhz
    // Communication Parameters: 8 Data, 1 Stop, No Parity
    // USART Receiver: On
    // USART Transmitter: On
    // USART Mode: Asynchronous
    // USART Baud Rate: 9600
//    UCSRA=(0<<RXC) | (0<<TXC) | (0<<UDRE) | (0<<FE) | (0<<DOR) | (0<<UPE) | (0<<U2X) | (0<<MPCM);
//    UCSRB=(1<<RXCIE) | (1<<TXCIE) | (0<<UDRIE) | (1<<RXEN) | (1<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);
//    UCSRC=(1<<URSEL) | (0<<UMSEL) | (0<<UPM1) | (0<<UPM0) | (0<<USBS) | (1<<UCSZ1) | (1<<UCSZ0) | (0<<UCPOL);
//    UBRRH=0x00;
//    UBRRL=0x1A ;   // imsi
#elif (USART_BPS == 19200)
    // USART initialization
    // Communication Parameters: 8 Data, 1 Stop, No Parity
    // USART Receiver: On
    // USART Transmitter: On
    // USART Mode: Asynchronous
    // USART Baud Rate: 19200
    UCSRA=(0<<RXC) | (0<<TXC) | (0<<UDRE) | (0<<FE) | (0<<DOR) | (0<<UPE) | (0<<U2X) | (0<<MPCM);
    UCSRB=(1<<RXCIE) | (1<<TXCIE) | (0<<UDRIE) | (1<<RXEN) | (1<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);
    UCSRC=(1<<URSEL) | (0<<UMSEL) | (0<<UPM1) | (0<<UPM0) | (0<<USBS) | (1<<UCSZ1) | (1<<UCSZ0) | (0<<UCPOL);
    UBRRH=0x00;
    UBRRL=0x33;
#elif (USART_BPS == 38400)
    // USART initialization
    // Communication Parameters: 8 Data, 1 Stop, No Parity
    // USART Receiver: On
    // USART Transmitter: On
    // USART Mode: Asynchronous
    // USART Baud Rate: 19200
    UCSRA=(0<<RXC) | (0<<TXC) | (0<<UDRE) | (0<<FE) | (0<<DOR) | (0<<UPE) | (0<<U2X) | (0<<MPCM);
    UCSRB=(1<<RXCIE) | (1<<TXCIE) | (0<<UDRIE) | (1<<RXEN) | (1<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);
    UCSRC=(1<<URSEL) | (0<<UMSEL) | (0<<UPM1) | (0<<UPM0) | (0<<USBS) | (1<<UCSZ1) | (1<<UCSZ0) | (0<<UCPOL);
    UBRRH=0x00;
    UBRRL=0x19;
#endif    

}
#pragma used-


