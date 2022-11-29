//
// =======================================================================
// [ PROJECT   ] type Define
// -----------------------------------------------------------------------
// [ License   ] SAMJIN ELECTRONICS
// [ Author    ] Copyright 2021-08 By HAG-SEONG KANG
// [ E-MAIL    ] hl5ugc@nate.com (82)10- 3841-9706
// [ C  P  U   ]  
// [ Compller  ] CodeWizardAVR V3.12 Professional
// [ Filename  ] SJTYPE.H
// [ Version   ] 1.0
// [ Created   ] 2022-09-08
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
#ifndef _SJTYPE_header
#define _SJTYPE_header

#pragma used+      
// ---------------------------------------------------------------------------
// Define Basic Define
// --------------------------------------------------------------------------- 
 
#define TRUE						1
#define FALSE						0
#define HI    					    1
#define LO						    0
#define ON						    1
#define OFF						    0
#define SET						    1
#define RESET						0
#define BIT_OK                      '0'
#define BIT_NG                      '1'

#define BIT0						0x0001
#define BIT1						0x0002
#define BIT2						0x0004
#define BIT3						0x0008
#define BIT4						0x0010
#define BIT5						0x0020
#define BIT6						0x0040
#define BIT7						0x0080 
#define BIT8						0x0100
#define BIT9						0x0200
#define BIT10						0x0400
#define BIT11						0x0800
#define BIT12						0x1000
#define BIT13						0x2000
#define BIT14						0x4000
#define BIT15						0x8000
// ---------------------------------------------------------------------
// Basic Typedefs
// ---------------------------------------------------------------------
typedef char                    int8_t    ;
typedef unsigned char           uint8_t   ;
typedef signed int              int16_t   ;
typedef unsigned int            uint16_t   ;
typedef unsigned long int       uint32_t  ;  
typedef volatile char           vint8_t   ; 
typedef volatile unsigned char  vuint8_t  ;
typedef volatile int            vint16_t  ;
typedef volatile unsigned int   vuint16_t ;
//
typedef unsigned char           BOOL ;  
typedef unsigned char           byte ;
typedef unsigned int            word;

//
// ---------------------------------------------------------------------
// Structure and Union Definitions
// ---------------------------------------------------------------------	
typedef struct  
{
    uint8_t bit0  		: 1 ;
    uint8_t bit1		: 1 ;
    uint8_t bit2		: 1 ;
    uint8_t bit3		: 1 ;
    uint8_t bit4		: 1 ;
    uint8_t bit5		: 1 ;
    uint8_t bit6		: 1 ;
    uint8_t bit7		: 1 ;	
}	DATA_BITS_t ;
//
typedef union    
{
    DATA_BITS_t     bits ;
    uint8_t         valus   ;
    
}  System_Flag_t  ;	 


//flash uint8_t *mfr1  = "SAMJIN KANG.H.S " ;  //  
//flash uint8_t *mfr2  = "SAMJIN ELECTRONI" ;  //  
//flash uint8_t *mfr3  = "Kang Hag Seong  " ;  //  
//flash uint8_t *mfr4  = "-hl5ugc@nate.com" ;  //  
//flash uint8_t *mfr5  = "tel+821038419706" ;  // 
 
#pragma used-
#endif  