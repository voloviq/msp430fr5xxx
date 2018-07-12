/**
*    @author Michal Wolowik
*    @date Warsaw 02/VII/2018
*
*    @brief
*        Bootloader intended to MSP430FR5xx devices
*
*    @file
*        main.c
*
*    @short
*       Main destiny of current code is to change user application
*       code located at address 0x4400
*
*    @version V1.0a
*
*    @preconditions
*        None
*
*    @remark
*        Compiler: TI v18.1.2.LTS
*
*    @remark
*        GUI: Code Composer Studio Version: 8.0.0.00016
*
*    @note
*       None
*
*    @copyright
*        This library is free software; you can redistribute it and/or
*        modify it under the terms of the GNU Lesser General Public
*        License as published by the Free Software Foundation; either
*        version 2.1 of the License, or (at your option) any later version.
*    @copyright
*        This library is distributed in the hope that it will be useful,
*        but WITHOUT ANY WARRANTY; without even the implied warranty of
*        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
*        Lesser General Public License for more details.
*
*    @note
*        None currently
*/


/*
 *  Standard ANSI C Library:
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <intrinsics.h>


/*
 *  Specific Device Library:
 */
#include <msp430.h>


/*
 *  External variables from linker file.
 *  Note that this file gets constants from linker file in order to be able
 *  to check where the Application resides, avoid corrupting boot area, etc
 */

extern uint16_t _Appl_End;                  /*! Application End Address */
extern uint32_t _Flex_Start;                /*! Flex Area Start Address */
extern uint32_t _Flex_End;                  /*! Flex Area End Address */
extern uint16_t _Appl_Checksum;             /*! Application Checksum Address */
extern uint16_t _Appl_Vector_Start;         /*! Application Vector Table Start */
extern uint16_t _Appl_Reset_Vector;         /*! Application Reset vector */
extern uint16_t __Boot_VectorTable;         /*! Bootloader Vector Table Start */
extern uint16_t __Boot_Start;               /*! Bootloader Start */

/*
 *  Macros and definitions
 */

/*! Application start address (from linker file) */
#define APP_START_ADDR          ((uint32_t )&_Appl_Checksum)
/*! Application end address (from linker file) */
#define APP_END_ADDR            ((uint32_t )&_Appl_End)
/*! Flex area start address (from linker file) */
#define FLEX_START_ADDR         ((uint32_t )&_Flex_Start)
/*! Flex area end address (from linker file) */
#define FLEX_END_ADDR           ((uint32_t )&_Flex_End)
/*! Application Vector Table */
#define APP_VECTOR_TABLE        ((uint32_t) &_Appl_Vector_Start)
/*! Application Reset Vector */
#define APP_RESET_VECTOR_ADDR   ((uint32_t) &_Appl_Reset_Vector)
/*! Application Interrupt Table (from linker file) */
#define BOOT_VECTOR_TABLE       ((uint32_t) &__Boot_VectorTable)
/*! Boot start address (from linker file) */
#define BOOT_START_ADDR         ((uint32_t) &__Boot_Start)


/* Bootloader version definition */
#define VERSION                 "Boot_1.0.a"
/* One separately page size */
#define BUF_SIZE                0x0020      /* 32 bytes page size */
/* In no character incoming after TIMEOUT go to user application*/
#define TIMEOUT                 0x0A


/*
 * Universal Bootloader request command
 */

#define GET_BOOT_VER            0x10        /* Get bootloader version */
#define HEARTBEAT_INDICATOR     0x11        /* LED diode heart beat indicator */
#define PAGE_SIZE_REQUEST       0x12        /* Page size / n-DATA quantity information */
#define MASS_FLASH_ERASE        0x13        /* Erase all flash pages */
#define DATA_FLASH_WRITE        0x14        /* Data and Flash Write */
#define ADDRESS_ALIGMENT        0x15        /* Address alignment factor */
#define JUMP_TO_APP             0x16        /* Jump to application */


/*
 * Universal Bootloader answer command
 */

#define BOOT_VER_STRING         VERSION     /* Bootloader version string */
#define PAGE_SIZE_VALUE         0           /* Specific controller page size - NA for MSP430FR5xxx */
#define ADDRESS_ALIGMENT_VALUE  0x0000      /* NA for MSP430FR5xxx device */
#define ACKNOWLEDGE             0x12        /* When received proper request send acknowledge */
#define FRAM_WRITE_ERR          0xF9        /* Write to FRAM memory error */
#define DATA_CRC_ERR            0xF1        /* Detected data frame crc error */


/*
 * Function return values
 */

#define RET_OK                  0           /*! Function returned OK */
#define RET_PARAM_ERROR         1           /*! Parameters are incorrect */
#define RET_JUMP_TO_APP         2           /*! Function exits and jump to application */


/*! Private global variables of current module declaration */

/** @brief LED_ON/OFF control definition  */
#define LED_ON                  P2OUT |= BIT5
#define LED_OFF                 P2OUT &= ~BIT5



/* Buffer which is intended to contain bytes to be written in flash */
uint8_t gbuffer[BUF_SIZE];


/* Timeout value - when reach TIMEOUT try to go to user application */
uint32_t timeout = 0x00000000;


/*!
@brief      Initializes the Memory Protection Unit of FR5989
            This allows for HW protection of Bootloader area
@return     None
@note       Testing result  :   Testing NA
@note       Conditions      :   NA
@note       Macro valid     :   NA
@note       Optimization    :   Level 4 - whole program optimization
@version    V1_0a
@author     M.Wolowik
@date       09.VII.2018
@bug        None
*/
static void MPU_init(void);


/*!
@brief      Function initialize minimal hardware configuration, like
            system clock, GPIO, watchdog etc.
@return     None
@note       Testing result  :   Testing NA
@note       Conditions      :   NA
@note       Macro valid     :   NA
@note       Optimization    :   Level 4 - whole program optimization
@version    V1_0a
@author     M.Wolowik
@date       02.VII.2018
@bug        None
*/
static void min_hw_init(void);


/*!
@brief      Function initialize UART 1 (9600, 8bit, 1stp, no parity)
@return     None
@note       Testing result  :   Testing NA
@note       Conditions      :   NA
@note       Macro valid     :   NA
@note       Optimization    :   Level 4 - whole program optimization
@version    V1_0a
@author     M.Wolowik
@date       02.VII.2018
@bug        None
*/
static void UART_init(void);


/*!
@brief      Function send character via UART
@param      outchar         :   is an uint8_t type, provide byte to send
@return     None
@note       Testing result  :   Testing NA
@note       Conditions      :   NA
@note       Macro valid     :   NA
@note       Optimization    :   Level 4 - whole program optimization
@version    V1_0a
@author     M.Wolowik
@date       02.VII.2018
@bug        None
*/
static void UART_putchar(uint8_t outchar);


/*!
@brief      Function receive character acquired from UART if available
@return     uint8t          :   Return received byte from UART
@note       Testing result  :   Testing NA
@note       Conditions      :   NA
@note       Macro valid     :   NA
@note       Optimization    :   Level 4 - whole program optimization
@version    V1_0a
@author     M.Wolowik
@date       02.VII.2018
@bug        None
*/
static char UART_getchar(void);


/*!
@brief      Function receive character acquired from UART if available
@param      tout_val        :   is an uint32_t type, provide byte receive timeout
@return     uint8t          :   Return received byte from UART
@note       Testing result  :   Testing NA
@note       Conditions      :   NA
@note       Macro valid     :   NA
@note       Optimization    :   Level 4 - whole program optimization
@version    V1_0a
@author     M.Wolowik
@date       02.VII.2018
@bug        None
*/
static char UART_getchar_with_timeout(uint16_t tout_val);


/*!
@brief      Function send via UART string
@param      buf             :   is an char pointer type,
                                provide buffer with string content
@return     None
@note       Testing result  :   Testing NA
@note       Conditions      :   NA
@note       Macro valid     :   NA
@note       Optimization    :   Level 4 - whole program optimization
@version    V1_0a
@author     M.Wolowik
@date       02.VII.2018
@bug        None
*/
static void send_string_uart(char *buf);


/*!
@brief      Main state machine of bootloader - refer to algorithm
@param      state           :   is an uint8_t type, provide current state
@return     None
@note       Testing result  :   Testing NA
@note       Conditions      :   NA
@note       Macro valid     :   NA
@note       Optimization    :   Level 4 - whole program optimization
@version    V1_0a
@author     M.Wolowik
@date       02.VII.2018
@bug        None
*/
static void bootloader_state_machine(uint8_t state);


/*!
@brief      Jumps to application using its reset vector address
@return     None
@note       Testing result  :   Testing NA
@note       Conditions      :   NA
@note       Macro valid     :   NA
@note       Optimization    :   Level 4 - whole program optimization
@version    V1_0a
@author     M.Wolowik
@date       09.VII.2018
@bug        None
*/
#define TI_MSPBoot_APPMGR_JUMPTOAPP()   {((void (*)()) _Appl_Reset_Vector) ();}


/*!
@brief      Function write data to FLASH(legacy to FRAM)
@param      addr            :   is an uint32_t type, provide address of block to store data
@param      data            :   is an uint8_t type, provide data to be stored
@return     None
@note       Testing result  :   Testing NA
@note       Conditions      :   NA
@note       Macro valid     :   NA
@note       Optimization    :   Level 4 - whole program optimization
@version    V1_0a
@author     M.Wolowik
@date       09.VII.2018
@bug        None
*/
static uint8_t writebytedirect(uint32_t addr, uint8_t data);


/*!
@brief      Function read from FLASH(legacy to FRAM)
@param      addr            :   is an uint32_t type, provide address of block to store data
@return     uint8_t         :   is read byte from specified addr of FLASH or FRAM
@note       Testing result  :   Testing NA
@note       Conditions      :   NA
@note       Macro valid     :   NA
@note       Optimization    :   Level 4 - whole program optimization
@version    V1_0a
@author     M.Wolowik
@date       09.VII.2018
@bug        None
*/
static uint8_t readbytedirect(uint32_t addr);


/*!
@brief      Erase the application area (address obtained from linker file)
            It can erase application in Download or App Area
            FRAM doesn't have an "erased" state but this function is added
            for compatibility with Flash
@param      downarea        :   is an uint8_t type, provide section to be erased
                                0 - application space
                                1 - download space
@return     None
@note       Testing result  :   Testing NA
@note       Conditions      :   NA
@note       Macro valid     :   NA
@note       Optimization    :   Level 4 - whole program optimization
@version    V1_0a
@author     M.Wolowik
@date       09.VII.2018
@bug        None
*/
static void erase_app_direct(uint8_t downarea);


/*!
@brief      Convert a virtual address of application to a physical address in
            download area
@param      addr            :   is an uint32_t type, provide address
                                in application memory
@return     uint32_t        :   Physical address in download area
@note       Testing result  :   Testing NA
@note       Conditions      :   NA
@note       Macro valid     :   NA
@note       Optimization    :   Level 4 - whole program optimization
@version    V1_0a
@author     M.Wolowik
@date       09.VII.2018
@bug        None
*/
uint32_t get_physical_address_from_virtual(uint32_t addr);


int main(void)
{
    uint16_t hbeat = 0;

    min_hw_init();

    UART_init();

    /* Clear timeout value */
    timeout = 0x00000000;

    while(1){
        if(hbeat == 0x01){
            LED_ON;
            hbeat++;
        }
        else if( hbeat == 0x02 ){
            LED_OFF;
            hbeat++;
        }
        else if (hbeat == 0x05){
            hbeat = 0x00;
        }
        else{
            hbeat++;
        }
        bootloader_state_machine(UART_getchar_with_timeout(0x7FFF));
    }
}


static void MPU_init(void)
{
    /* These calculations work for FR5989 (check user guide for MPUSEG values) */
    /*  Border 1  = Start of bootloader */
    /*  Border 2  = 0x10000 */
    /*  Segment 1 = 0x4400 - Bootloader */
    /*  Segment 2 = Bootloader - 0xFFFF */
    /*  Segment 3 = 0x10000 - 0x23FFF */
    /*  Segment 2 is write protected and generates a PUC */

    MPUCTL0   = MPUPW;                   /* Write PWD to access MPU registers    */
    MPUSEGB1  = (BOOT_START_ADDR) >> 4;  /* B1 = Start of Boot; B2 = 0x10000     */
    MPUSEGB2  = (0x10000) >> 4;
    MPUSAM   &= ~MPUSEG2WE;              /* Segment 2 is protected from write    */
    MPUSAM   |= MPUSEG2VS;               /* Violation select on write access     */
    MPUCTL0   = MPUPW | MPUENA;          /* Enable MPU protection                */
    MPUCTL0_H = 0x00;                    /* Disable access to MPU registers      */

}


static void min_hw_init(void)
{
    WDTCTL = WDTPW | WDTHOLD;   /* Stop watchdog timer */

    MPU_init();

    /*  Use external 32KHz crystal
        MClock and SMClock from DCO 4MHz */

    /* Using LFXTCLK */
    PJSEL0 |=  BIT4 + BIT5;

    /* Write in the password for system clock setting */
    CSCTL0 = 0XA500;

    /* 4MHz for DCO */
    CSCTL1 = 0x0006;

    /*  ACLK from LFXTCLK(SELA), SMCLK(SELS) and MCLK from DCO(SELM)
        For more details refer to SLAU367O chapter 3.3.3
        CSCTL2 Register */
    CSCTL2 = 0x0033;

    /* ACLK div by 1, SMCLK div by 1, MCLK div by 1 */
    CSCTL3 = 0x0000;

    /* HFXTOFF, LFXTDRIVE = 1, VLO off, SMCLK on */
    CSCTL4 = 0x0148;

    do{
        /* Clear XT1 fault flag */
        CSCTL5 &= ~LFXTOFFG;
        SFRIFG1 &= ~OFIFG;
    }while (SFRIFG1&OFIFG); /* Test oscillator fault flag */


    /*  This is to set the I/O port in the initialization.
        The power consumption will be 0.8uA in LPM4 after setting   */
    P1DIR=0xFF;
    P1OUT=0;
    P2DIR=0xFF;
    P2OUT=0;
    P3DIR=0xFF;
    P3OUT=0;
    P4DIR=0xFF;
    P4OUT=0;
    P5DIR=0xFF;
    P5OUT=0;
    P6DIR=0xFF;
    P6OUT=0;
    P7DIR=0xFF;
    P7OUT=0;
    P8DIR=0xFF;
    P8OUT=0;
    P9DIR=0x00;
    P9OUT=0;
    P10DIR=0xFF;
    P10OUT=0;
    PJDIR=0xFF;
    PJOUT=0;

    /*  Configure GPIO P2.5 as output
        for LED diode for testing purposes
        HW_DD_CFG_1 - refers to HW-DD document */
    /*  Configure port as output */
    P2DIR |= BIT5;
    /*  Set default state after reset on low level logic */
    P2OUT &= ~BIT5;


    CECTL3 = 0xFF00;

    /* Disable the GPIO power-on default high-impedance mode to activate
       previously configured port settings */
    PM5CTL0 &= ~LOCKLPM5;

    PMMCTL0_H = 0xA5;
    PMMCTL0_L &= ~SVSHE;
    PMMCTL0_H = 0xEE;
}


static void UART_init(void)
{
    /* Step 1. First configure GPIO to switch them
     * to USCI_A0 UART purpose
     */
    P4SEL0 |= BIT2 | BIT3;
    P4SEL1 &= ~(BIT2 | BIT3);


    /* Step 2. Disable the GPIO power-on default
     * high-impedance mode to activate
     * previously configured port settings
     */
    PM5CTL0 &= ~LOCKLPM5;

    /* Step 3. Configure USCI_A0 for UART mode */
    UCA0CTLW0 = UCSWRST;    /* Put eUSCI in reset */
    UCA0CTLW0 |= UCSSEL__SMCLK; /* CLK = SMCLK */

    /* Baud Rate calculation
     * 4000000/(16*9600) = 26.041
     * Fractional portion = 0.083
     * User's Guide Table 21-4: UCBRSx = 0x04
     * UCBRFx = int ( (26.041-26)*16) = 1
     */
    /* 9600 Baudrate */
//    UCA0BR0 = 26;   /* 4000000/16/9600 */
//    UCA0BR1 = 0x00;
//    UCA0MCTLW |= UCOS16 | UCBRF_1 | 0xB600;
    /* 115200 Baudrate */
    UCA0BR0 = 0x02;   /* 4000000/16/115200 */
    UCA0BR1 = 0x00;
    UCA0MCTLW |= UCOS16 | UCBRF_2 | 0xBB00;

    UCA0CTLW0 &= ~UCSWRST;  /* Initialize eUSCI */
}


static void UART_putchar(uint8_t outchar)
{
    while(!(UCA0IFG&UCTXIFG));
    UCA0TXBUF = outchar;
    __no_operation();
}


static char UART_getchar(void)
{
    int8_t dt;

    while(!(UCA0IFG&UCRXIFG));
    dt = UCA0RXBUF;

    return(dt);
}


static char UART_getchar_with_timeout(uint16_t tout_val)
{
    int8_t dt=-1;
    uint16_t tmp = tout_val;

    while(tmp)
    {
        if(UCA0IFG&UCRXIFG){
            dt = UCA0RXBUF;
            return(dt);
        }
        else{
            tmp--;
            __delay_cycles(10);
        }
    }
    return(dt);
}


static void send_string_uart(char *buf)
{
    uint8_t i = 0x00;
    do{
        if(buf[i] != '\0'){
            UART_putchar(buf[i]);
        }
    }while(buf[i++] != '\0');
}


static void bootloader_state_machine(uint8_t state)
{
    uint8_t rec_size = 0;
    uint32_t address = 0;
    uint32_t adr;
    uint8_t crc = 0;
    uint8_t crc_frame;

    switch (state)
    {
        case GET_BOOT_VER:{
            /* First send command - Boot Loader Version */
            UART_putchar(GET_BOOT_VER);
            send_string_uart((char*)BOOT_VER_STRING);
        break;
        }
        case HEARTBEAT_INDICATOR:{
            timeout = 0;
            UART_putchar(ACKNOWLEDGE);
        break;
        }
        case MASS_FLASH_ERASE:{
            /* Only user code section */
            erase_app_direct(0);
            UART_putchar(ACKNOWLEDGE);
        break;
        }
        case DATA_FLASH_WRITE:{
            rec_size = (uint8_t)UART_getchar();
            address = ((uint32_t)(UART_getchar()))<<24;
            address |= ((uint32_t)(UART_getchar()))<<16;
            address |= ((uint16_t)(UART_getchar()))<<8;
            address |= (uint16_t)UART_getchar();
            if(address > 0xFFFF)
            {
                crc = 0;
            }
            crc = 0;
            for (adr = 0; adr < rec_size; adr++){
                gbuffer[adr] = UART_getchar();
            }
            crc_frame = UART_getchar();
            for (adr = 0; adr < rec_size; adr++){
                crc ^= gbuffer[adr];
            }
            if(crc == crc_frame){
                for (adr = 0; adr < rec_size; adr++){
                    writebytedirect(address+adr, gbuffer[adr]);
                }
                for (adr = 0; adr < rec_size; adr++){
                    if(readbytedirect(address+adr)!=gbuffer[adr])
                        UART_putchar(DATA_CRC_ERR);
                    else
                        break;
                }
                UART_putchar(ACKNOWLEDGE);
            }
            else
                UART_putchar(DATA_CRC_ERR);
        break;
        }
        case PAGE_SIZE_REQUEST:{
            crc = PAGE_SIZE_REQUEST;
            UART_putchar(PAGE_SIZE_REQUEST);
            crc += (uint8_t)((PAGE_SIZE_VALUE&0xFF00)>>8);
            UART_putchar((uint8_t)((PAGE_SIZE_VALUE&0xFF00)>>8));
            crc += (uint8_t)(PAGE_SIZE_VALUE&0x00FF);
            UART_putchar((uint8_t)(PAGE_SIZE_VALUE&0x00FF));
            UART_putchar(crc);
        break;
        }
        case ADDRESS_ALIGMENT:{
            crc = ADDRESS_ALIGMENT;
            UART_putchar(ADDRESS_ALIGMENT);
            crc += (uint8_t)((ADDRESS_ALIGMENT_VALUE&0xFF00)>>8);
            UART_putchar((uint8_t)((ADDRESS_ALIGMENT_VALUE&0xFF00)>>8));
            crc += (uint8_t)(ADDRESS_ALIGMENT_VALUE&0x00FF);
            UART_putchar((uint8_t)(ADDRESS_ALIGMENT_VALUE&0x00FF));
            UART_putchar(crc);
        break;
        }
        case JUMP_TO_APP:{
            UART_putchar(ACKNOWLEDGE);
            __delay_cycles(70000);
            /* Disable UART before jump to user code */
            UCA0CTLW0 |= UCSWRST;

            /* Jump to user application code */
            TI_MSPBoot_APPMGR_JUMPTOAPP();
            __no_operation();
        break;
        }
        default:{
            timeout++;
            if(timeout == TIMEOUT){
                /* Disable UART before jump to user code */
                UCA0CTLW0 |= UCSWRST;

                /* Jump to user application code */
                TI_MSPBoot_APPMGR_JUMPTOAPP();

                __no_operation();
            }
            else
                __delay_cycles(10);
        break;
        }
    }
}


void erase_app_direct(uint8_t downarea)
{
    uint32_t addr;

    for(addr = APP_START_ADDR; addr <= APP_END_ADDR; addr+=2){
        if(downarea==0){
            /* Erase all the application area */
            __data20_write_short(addr, 0xFFFF);
        }
        else{
            /* Erase the download area */
            __data20_write_short(get_physical_address_from_virtual(addr), 0xFFFF);
        }
    }

    if(FLEX_START_ADDR == 0x10000){
        for(addr = FLEX_START_ADDR; addr <= FLEX_END_ADDR; addr+=2){
            if(downarea==0){
                /* Erase all the application area */
                __data20_write_short(addr, 0xFFFF);
            }
            else{
                /* Erase the download area */
                __data20_write_short(get_physical_address_from_virtual(addr), 0xFFFF);
            }
        }
    }
}


uint32_t get_physical_address_from_virtual(uint32_t addr)
{
    volatile uint32_t ret;
    volatile uint32_t address;
    /*! Download Offset Size */
    extern uint32_t _Down_Offset_Size;
    /*! Download Offset 1 */
    extern uint32_t _Down_Offset1;
    /*! Download Offset 2 */
    extern uint32_t _Down_Offset2;

    /* Application will be stored to 2 areas in memory. I.e.
       data from 4400-7DFF  will be stored to BE00-F7FF
       data from 7E00-BDFF will be stored to 10000-13FFF */

    address = addr;

    /* If the address-offset fits in 1st application area */
    if (address <=  (uint32_t )&_Down_Offset_Size)
        ret = (addr + (uint32_t )&_Down_Offset1);
    /* else, place it in the 2nd download area */
    else
        ret = (addr + (uint32_t )&_Down_Offset2);

    return ret;
}


static uint8_t writebytedirect(uint32_t addr, uint8_t data)
{
    __data20_write_char(addr, data);    /* Write to memory */

    if ((addr >= APP_VECTOR_TABLE) && (addr < APP_RESET_VECTOR_ADDR - 2)){
        /* If address is an interrupt vector,
           copy directly to interrupt table */
        addr      = (addr - APP_VECTOR_TABLE) + BOOT_VECTOR_TABLE;
        MPUCTL0   = MPUPW | MPUENA;     /* Enable access to MPU registers   */
        MPUSAM   |= MPUSEG2WE;          /* Enable Write access              */
        __data20_write_char(addr, data);/* Write to interrupt table         */
        MPUSAM   &= ~MPUSEG2WE;         /* Disable Write access             */
        MPUCTL0_H = 0x00;               /* Disable access to MPU registers  */
    }
    return RET_OK;
}


static uint8_t readbytedirect(uint32_t addr)
{
    uint8_t rval;

    rval = __data20_read_char(addr);    /* Read from memory */

    if ((addr >= APP_VECTOR_TABLE) && (addr < APP_RESET_VECTOR_ADDR - 2)){
        /* If address is an interrupt vector,
           copy directly to interrupt table */
        addr      = (addr - APP_VECTOR_TABLE) + BOOT_VECTOR_TABLE;
        MPUCTL0   = MPUPW | MPUENA;          /* Enable access to MPU registers   */
        MPUSAM   |= MPUSEG2RE;               /* Enable Read access               */
        rval      = __data20_read_char(addr);/* Read from interrupt table        */
        MPUSAM   &= ~MPUSEG2RE;              /* Disable Read access              */
        MPUCTL0_H = 0x00;                    /* Disable access to MPU registers  */
    }
    return rval;
}
