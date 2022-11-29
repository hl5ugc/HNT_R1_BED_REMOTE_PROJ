//
// =======================================================================
// [ PROJECT   ] R1 BED REMOTE CONTROL 
// -----------------------------------------------------------------------
// [ License   ] SAMJIN ELECTRONICS
// [ Author    ] Copyright 2022-09 By HAG-SEONG KANG
// [ E-MAIL    ] hl5ugc@nate.com (82)10- 3841-9706
// [ C  P  U   ] ATmega8A
// [ Compller  ] CodeWizardAVR V3.12 Professional
// [ Filename  ] R1_BED_REMORE.C
// [ Version   ] 1.0
// [ Created   ] 2022-09-10
// ----------------------------------------------------------------------------
// Revision History :
// ----------------------------------------------------------------------------
// Author         Date          Comments on this revision    
// ----------------------------------------------------------------------------
// HAG-SEONG KANG 2022-09-08    First release of Source file   
// HAG-SEONG KANG 2022-10-13    OSC 16MHz Modify
// 
// ============================================================================
//
//
//
//
#pragma used+
#include <mega8.h>
#include <delay.h>
// Standard Input/Output functions
#include <stdio.h>
// Declare your global variables here
#include "SJTYPE.H"  
#include "R1_BED_REMORE_V102.H"
#include "UART.H" 
//
static uint8_t Check_Rom(void); 
// --------------------------------------------------
// Timer 0 overflow interrupt service routine
// --------------------------------------------------
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
    // Reinitialize Timer 0 value
    TCNT0=0x06; //  1mSec 
    // Place your code here
    Uart_Check() ;
    //
    // Buzzer Control
    if(Is_Buzzer_Start_Flag)
    {
        Buzzer_On ; 
        if(--Buzzer_On_time == 0x00)
        {
            Buzzer_Off ; 
            Buzzer_Start_Flag_Off ;
            Buzzer_End_Flag_On ;
        }
    }
    else 
    {   // Check Eror Mask
        Buzzer_Off ;
    } 
    // 
}
// --------------------------------------------------
// Timer 2 output compare interrupt service routine
// --------------------------------------------------
// 4mSec
interrupt [TIM2_COMP] void timer2_comp_isr(void)
{
    // Place your code here
    static uint8_t u8Data = 0 ;  
    static uint16_t u16Data = 0 ;
    //
    // Key InPut    
    u8Data =  ((~PINC) & 0b00000111) ;      // Joy s/w, sw10, sw9  
    u16Data = ((uint16_t)u8Data << 8) & 0x0700 ;
    //
    // LEFT X > 128 , RIGHT X < 56
    if(adc_data[0]      <= AD_SWITCH_MIN)        { u16Data |=  0x2000 ; }  // RIGHT X
    else if(adc_data[0] >= AD_SWITCH_MAX)   { u16Data |=  0x1000 ; }  // LEFT X
    else                                    { u16Data &=  0xCFFF ; }
    // UP Y < 56 , DOWN Y > 200
    if(adc_data[1]      <= AD_SWITCH_MIN)        { u16Data |=  0x4000 ; }    // UP Y
    else if(adc_data[1] >= AD_SWITCH_MAX)   { u16Data |=  0x8000 ; }    // DOWN Y
    else                                    { u16Data &=  0x3FFF ; } 
    //
    u8Data  = ((~PINB) & 0b00000111) ;
    u8Data |= ((~PIND) & 0b11111000) ; 
    u16Data =  u16Data + u8Data  ;  
    // Check New Data
    if(key_in_data_old != u16Data) 
    {
        key_in_data_old = u16Data ;
    } 
    // Re Check New Data 
    else if(key_in_data != key_in_data_old)
    {
        key_in_data = key_in_data_old ; 
        //   
        Key_Change_Flag_On ;
        // Check Key Pressed
        if(key_in_data == 0x00)
        {   // UnPress
            Buzzer_End_Flag_Off ;
        } 
        else
        {   // Check Key Pressed or New Key Pressed 
            if(Is_Not_Buzzer_End_Flag)
            {
                Buzzer_On ; 
                Buzzer_Start_Flag_On ;  
                Buzzer_End_Flag_Off ;  
                Buzzer_On_time = BUZZER_ON_TIME ;   
            }
        }
    } 
    else  // Key Data Update
    {
        KEY_DATA_BUFF2 = (key_in_data >> 8);  
        KEY_DATA_BUFF1 =  key_in_data ;  
        //
        if(Is_Key_Change_Flag) {
            Key_Change_Flag_Off ;
            Key_Update_Flag_On ;
        }
    }
    // Led Output Control   
    hc595_Output(LED_DATA_BUFF) ;
}
// --------------------------------------------------
// ADC interrupt service routine
// with auto input scanning
// --------------------------------------------------
interrupt [ADC_INT] void adc_isr(void)
{
    static unsigned char input_index=0;
    // Read the 8 most significant bits
    // of the AD conversion result
    adc_data[input_index]=ADCH;
    // Select next ADC input
    if (++input_index > (LAST_ADC_INPUT-FIRST_ADC_INPUT))
       input_index=0;
    ADMUX=(FIRST_ADC_INPUT | ADC_VREF_TYPE)+input_index;
    // Delay needed for the stabilization of the ADC input voltage
    delay_us(10);
    // Start the AD conversion
    ADCSRA|=(1<<ADSC);
}

/**
 * @brief   Main Program Start
 * 
 */
void main(void)
{
    static uint8_t u8Temp = 0 ;
    // Declare your local variables here
    System_Init();
    Uart_Init() ;
    // Global enable interrupts
    #asm("sei")
    //
    if( Check_Rom() == u8Temp) u8Temp = 0x00 ;
    //
    while (1)
    {
        // key Data Update By Key Press or unPress 
        if(Is_Key_Update_Flag) {
            Key_Update_Flag_Off ; 
            // Key Data Send To ACTUATOR Main Board
            send_Write_Data_Bytes(0x00,0x02) ;
        }  
    }
}
//
// -----------------------------------------------
/**
 * @brief   74HC595 For Led On/Off Control
 * 
 * @param   u8Data LED On/Off Data
 */
//#define hc595_Clk_Hi    PORTB.5 = 1
//#define hc595_Clk_Lo    PORTB.5 = 0
//#define hc595_Data_Hi   PORTB.3 = 1
//#define hc595_Data_Lo   PORTB.3 = 0
//#define hc595_Cs_Hi     PORTC.5 = 1
//#define hc595_Cs_Lo     PORTC.5 = 0
//
static void hc595_Output(uint8_t u8Data)
{
    static uint8_t u8Temp = 0 ;
    //
    hc595_Clk_Hi ;
    hc595_Data_Hi ; 
    hc595_Cs_Lo ;
    //
    for(u8Temp = 8 ; u8Temp != 0 ; u8Temp--)
    {
        hc595_Clk_Lo ; 
        if((u8Data & 0x80) == 0x00) { hc595_Data_Lo ;}
        else { hc595_Data_Hi ;}
        //
        delay_us(2) ;
        hc595_Clk_Hi ; 
        delay_us(4) ;
        //
        u8Data = u8Data << 1 ;
    } 
    //  
    hc595_Cs_Hi ;
}

static uint8_t Check_Rom(void)
{
    static uint8_t u8Temp = 0 ;   
    
    u8Temp  = *mfr1 ;
    u8Temp += *mfr2 ;  
    u8Temp += *mfr3 ;
    u8Temp += *mfr4 ;   
    //
    u8Temp += epdata1[0] ; 
    u8Temp += epdata2[0] ;
    u8Temp += epdata3[0] ;
    //
    if(u8Temp != 0x00) u8Temp = 0 ;
    //
    return u8Temp ;
}

#pragma used-