//
// =======================================================================
// [ PROJECT   ] R1 BED REMOTE CONTROL 
// -----------------------------------------------------------------------
// [ License   ] SAMJIN ELECTRONICS
// [ Author    ] Copyright 2021-08 By HAG-SEONG KANG
// [ E-MAIL    ] hl5ugc@nate.com (82)10- 3841-9706
// [ C  P  U   ]  
// [ Compller  ] CodeWizardAVR V3.12 Professional
// [ Filename  ] R1_BED_REMORE.H
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
#ifndef _R1_BED_REMORE_header
#define _R1_BED_REMORE_header

#pragma used+
/* Includes */
 

// ---------------------------------------------------------------------------
// Basic Define  
// ---------------------------------------------------------------------------
//
#define BUZZER_ON_TIME  50 
#define AD_SWITCH_MAX   200
#define AD_SWITCH_MIN   56
// 
// 
#define FIRST_ADC_INPUT 3
#define LAST_ADC_INPUT 4
uint8_t adc_data[LAST_ADC_INPUT-FIRST_ADC_INPUT+1];
// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (1<<ADLAR))

//
// ---------------------------------------------------------------------------
// Bit control PortA - PortG
// --------------------------------------------------------------------------- 
//
#define Buzzer_On       PORTD.2 = 1 
#define Buzzer_Off      PORTD.2 = 0 
//
#define hc595_Clk_Hi    PORTB.5 = 1
#define hc595_Clk_Lo    PORTB.5 = 0
#define hc595_Data_Hi   PORTB.3 = 1
#define hc595_Data_Lo   PORTB.3 = 0
#define hc595_Cs_Hi     PORTC.5 = 1
#define hc595_Cs_Lo     PORTC.5 = 0
// --------------------------------------------------------------------------- 
// Global Variables Define
// ----------------------------------------------------------------------------
//
vuint16_t   key_in_data  ; 
vuint16_t   key_in_data_old   ; 
vuint8_t    Buzzer_On_time  ;

eeprom unsigned char epdata1[20] = {"SAMJIN ELECTRONICS "} ;
eeprom unsigned char epdata2[20] = {" - Kang Hag Seong -"} ;
eeprom unsigned char epdata3[20] = {"- hl5ugc@nate.com -"} ;
//

// ---------------------------------------------------------------------------
// Define Variables 
// --------------------------------------------------------------------------- 
//
System_Flag_t Buzzer_Flag ;


// ---------------------------------------------------------------------------
// Define Macro
// --------------------------------------------------------------------------- 
//
#define Buzzer_Flag_value           Buzzer_Flag.valus
//
#define Buzzer_Start_Flag_On        Buzzer_Flag.bits.bit0 = 1
#define Buzzer_Start_Flag_Off       Buzzer_Flag.bits.bit0 = 0   
#define Is_Buzzer_Start_Flag        Buzzer_Flag.bits.bit0 == 1 
#define Is_Not_Buzzer_Start_Flag    Buzzer_Flag.bits.bit0 == 0
//
#define Buzzer_End_Flag_On          Buzzer_Flag.bits.bit1 = 1
#define Buzzer_End_Flag_Off         Buzzer_Flag.bits.bit1 = 0   
#define Is_Buzzer_End_Flag          Buzzer_Flag.bits.bit1 == 1 
#define Is_Not_Buzzer_End_Flag      Buzzer_Flag.bits.bit1 == 0
//
//
#define Key_Change_Flag_On          Buzzer_Flag.bits.bit6 = 1
#define Key_Change_Flag_Off         Buzzer_Flag.bits.bit6 = 0   
#define Is_Key_Change_Flag          Buzzer_Flag.bits.bit6 == 1 
#define Is_Not_Key_Change_Flag      Buzzer_Flag.bits.bit6 == 0
//
#define Key_Update_Flag_On          Buzzer_Flag.bits.bit7 = 1
#define Key_Update_Flag_Off         Buzzer_Flag.bits.bit7 = 0   
#define Is_Key_Update_Flag          Buzzer_Flag.bits.bit7 == 1 
#define Is_Not_Key_Update_Flag      Buzzer_Flag.bits.bit7 == 0
//
 
// ---------------------------------------------------------------------------
// Define  Function prototypes.
// --------------------------------------------------------------------------- 
//
static void System_Init(void) ;
static void hc595_Output(uint8_t u8Data);
 

// ---------------------------------------------------------------------------
// Define  Callback protoetypes.
// --------------------------------------------------------------------------- 
//
 

static void System_Init(void)
{
    // Input/Output Ports initialization
    // Port B initialization
    // Function: Bit7=In Bit6=In Bit5=Out Bit4=In Bit3=Out Bit2=In Bit1=In Bit0=In 
    DDRB=(0<<DDB7) | (0<<DDB6) | (1<<DDB5) | (0<<DDB4) | (1<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
    // State: Bit7=T Bit6=T Bit5=1 Bit4=P Bit3=1 Bit2=P Bit1=P Bit0=P 
    PORTB=(0<<PORTB7) | (0<<PORTB6) | (1<<PORTB5) | (1<<PORTB4) | (1<<PORTB3) | (1<<PORTB2) | (1<<PORTB1) | (1<<PORTB0);
    // Port C initialization
    // Function: Bit6=In Bit5=Out Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
    DDRC=(0<<DDC6) | (1<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
    // State: Bit6=T Bit5=0 Bit4=T Bit3=T Bit2=P Bit1=P Bit0=P 
    PORTC=(0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (1<<PORTC2) | (1<<PORTC1) | (1<<PORTC0);
    // Port D initialization
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=Out Bit1=Out Bit0=In 
    DDRD=(0<<DDD7) | (0<<DDD6) | (0<<DDD5) | (0<<DDD4) | (0<<DDD3) | (1<<DDD2) | (1<<DDD1) | (0<<DDD0);
    // State: Bit7=P Bit6=P Bit5=P Bit4=P Bit3=P Bit2=0 Bit1=0 Bit0=T 
    PORTD=(1<<PORTD7) | (1<<PORTD6) | (1<<PORTD5) | (1<<PORTD4) | (1<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);
    // Timer/Counter 0 initialization
    // Clock source: System Clock
    // Clock value: 250.000 kHz
    TCCR0=(0<<CS02) | (1<<CS01) | (1<<CS00);
    TCNT0=0x06;   
    // Timer/Counter 1 initialization
    // Clock source: System Clock
    // Clock value: Timer1 Stopped
    // Mode: Normal top=0xFFFF
    // OC1A output: Disconnected
    // OC1B output: Disconnected
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // Timer1 Overflow Interrupt: Off
    // Input Capture Interrupt: Off
    // Compare A Match Interrupt: Off
    // Compare B Match Interrupt: Off
    TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
    TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
    TCNT1H=0x00;
    TCNT1L=0x00;
    ICR1H=0x00;
    ICR1L=0x00;
    OCR1AH=0x00;
    OCR1AL=0x00;
    OCR1BH=0x00;
    OCR1BL=0x00;
    // Timer/Counter 2 initialization
    // Clock source: System Clock
    // Clock value: 125.000 kHz
    // Mode: CTC top=OCR2A
    // OC2 output: Disconnected
    // Timer Period: 2 ms
//    ASSR=0<<AS2;
//    TCCR2=(0<<PWM2) | (0<<COM21) | (0<<COM20) | (1<<CTC2) | (1<<CS22) | (0<<CS21) | (1<<CS20);
//    TCNT2=0x00;
//    OCR2=0xF9;  
    // Timer/Counter 2 initialization
    // Clock source: System Clock
    // Clock value: 62.500 kHz
    // Mode: CTC top=OCR2A
    // OC2 output: Disconnected
    // Timer Period: 4 ms
    ASSR=0<<AS2;
    TCCR2=(0<<PWM2) | (0<<COM21) | (0<<COM20) | (1<<CTC2) | (1<<CS22) | (1<<CS21) | (0<<CS20);
    TCNT2=0x00;
    OCR2=0xF9;
    // Timer(s)/Counter(s) Interrupt(s) initialization
    TIMSK=((1<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (1<<TOIE0));
    // ADC initialization
    // ADC Clock frequency: 125.000 kHz
    // ADC Voltage Reference: AVCC pin
    // Only the 8 most significant bits of
    // the AD conversion result are used
    ADMUX=FIRST_ADC_INPUT | ADC_VREF_TYPE;
    ADCSRA=(1<<ADEN) | (1<<ADSC) | (0<<ADFR) | (0<<ADIF) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
    SFIOR=(0<<ACME);
 
}

flash uint8_t *mfr1  = "SAMJIN ELECTRONICS" ;  //  
flash uint8_t *mfr2  = "Kang Hag Seong  " ;  //  
flash uint8_t *mfr3  = "-hl5ugc@nate.com" ;  //  
flash uint8_t *mfr4  = "tel+821038419706" ;  //  
#pragma used-
#endif /* SAMJIN_header */ 