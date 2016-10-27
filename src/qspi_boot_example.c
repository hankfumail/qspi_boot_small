/******************************************************************************
*
* Copyright (C) 2012 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/
/******************************************************************************
*
* @file xspi_numonyx_flash_quad_example.c
*
* This file contains a design example using the SPI driver (XSpi) and axi_qspi
* device with a Numonyx quad serial flash device in the interrupt mode.
* This example erases a Sector, writes to a Page within the Sector, reads back
* from that Page and compares the data.
*
* This example  has been tested with an N25Q128 device on KC705 and ZC770
* board. The bytes per page (PAGE_SIZE) in N25Q128 is 256.
*
* @note
*
* None.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -----------------------------------------------
* 1.00a bss  08/08/12 First release
* 3.04a bss  02/11/13 Modified to use ScuGic in case of Zynq (CR#683510)
* </pre>
*
******************************************************************************/

/***************************** Include Files *********************************/

#include <stdio.h>
#include <stdlib.h>

#include "xparameters.h"	/* EDK generated parameters */
#include "xspi.h"		/* SPI device driver */
#include "xil_exception.h"


#include "platform.h"
#include "xil_printf.h"

/************************** Constant Definitions *****************************/

/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define SPI_DEVICE_ID			XPAR_SPI_0_DEVICE_ID

#ifdef XPAR_INTC_0_DEVICE_ID
 #define INTC_DEVICE_ID		XPAR_INTC_0_DEVICE_ID
 #define SPI_INTR_ID		XPAR_INTC_0_SPI_0_VEC_ID
#else
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
 #define SPI_INTR_ID		XPAR_FABRIC_SPI_0_VEC_ID
#endif /* XPAR_INTC_0_DEVICE_ID */


#ifdef XPAR_INTC_0_DEVICE_ID
#define INTC		    static XIntc
#define INTC_HANDLER	XIntc_InterruptHandler
#else
#define INTC			static XScuGic
#define INTC_HANDLER	XScuGic_InterruptHandler
#endif /* XPAR_INTC_0_DEVICE_ID */

/*
 * The following constant defines the slave select signal that is used to
 * to select the Flash device on the SPI bus, this signal is typically
 * connected to the chip select of the device.
 */
#define SPI_SELECT 			0x01

/*
 * Definitions of the commands shown in this example.
 */
#define COMMAND_PAGE_PROGRAM		0x02 /* Page Program command */
#define COMMAND_QUAD_WRITE		0x32 /* Quad Input Fast Program */
#define COMMAND_RANDOM_READ		0x03 /* Random read command */
#define COMMAND_DUAL_READ		0x3B /* Dual Output Fast Read */
#define COMMAND_DUAL_IO_READ		0xBB /* Dual IO Fast Read */
#define COMMAND_QUAD_READ		0x6B /* Quad Output Fast Read */
#define COMMAND_QUAD_IO_READ		0xEB /* Quad IO Fast Read */
#define	COMMAND_WRITE_ENABLE		0x06 /* Write Enable command */
#define COMMAND_SECTOR_ERASE		0xD8 /* Sector Erase command */
#define COMMAND_BULK_ERASE		0xC7 /* Bulk Erase command */
#define COMMAND_STATUSREG_READ		0x05 /* Status read command */

#define READ_ID			0x9F

/**
 * This definitions specify the EXTRA bytes in each of the command
 * transactions. This count includes Command byte, address bytes and any
 * don't care bytes needed.
 */
#define READ_WRITE_EXTRA_BYTES		4 /* Read/Write extra bytes */
#define	WRITE_ENABLE_BYTES		1 /* Write Enable bytes */
#define SECTOR_ERASE_BYTES		4 /* Sector erase extra bytes */
#define BULK_ERASE_BYTES		1 /* Bulk erase extra bytes */
#define STATUS_READ_BYTES		2 /* Status read bytes count */
#define STATUS_WRITE_BYTES		2 /* Status write bytes count */
#define RD_ID_SIZE		4 /* Read ID command + 3 bytes ID response */

/*
 * Flash not busy mask in the status register of the flash device.
 */
#define FLASH_SR_IS_READY_MASK		0x01 /* Ready mask */

/*
 * Number of bytes per page in the flash device.
 */
#define PAGE_SIZE			256

/*
 * Address of the page to perform Erase, Write and Read operations.
 */
#define FLASH_TEST_ADDRESS		0x00

/*
 * Byte Positions.
 */
#define BYTE1				0 /* Byte 1 position */
#define BYTE2				1 /* Byte 2 position */
#define BYTE3				2 /* Byte 3 position */
#define BYTE4				3 /* Byte 4 position */
#define BYTE5				4 /* Byte 5 position */
#define BYTE6				5 /* Byte 6 position */
#define BYTE7				6 /* Byte 7 position */
#define BYTE8				7 /* Byte 8 position */

/*
 * The following definitions specify the number of dummy bytes to ignore in the
 * data read from the flash, through various Read commands. This is apart from
 * the dummy bytes returned in reponse to the command and address transmitted.
 */
/*
 * After transmitting Dual Read command and address on DIO0,the quad spi device
 * configures DIO0 and DIO1 in input mode and receives data on both DIO0 and
 * DIO1 for 8 dummy clock cycles. So we end up with 16 dummy bits in DRR. The
 * same logic applies Quad read command, so we end up with 4 dummy bytes in
 * that case.
 */
#define DUAL_READ_DUMMY_BYTES		2
#define QUAD_READ_DUMMY_BYTES		4

#define DUAL_IO_READ_DUMMY_BYTES	2
#define QUAD_IO_READ_DUMMY_BYTES	5


#define QSPI_BOOT_FLASH_ADDR	0xe00000  	// 14MB
#define QSPI_BOOT_ROM_ADDR		0x00000000  //
#define QSPI_BOOT_VECTOR_SIZE	0x00000040  //
#define QSPI_BOOT_APP_ADDR		0x80000000  //
#define QSPI_BOOT_APP_SIZE		0x100000  	// 
#define QSPI_BOOT_READ_CMD		COMMAND_RANDOM_READ  // COMMAND_QUAD_READ

#if ( QSPI_BOOT_READ_CMD==COMMAND_QUAD_READ )
#define QSPI_BOOT_DUMMY_BYTES	(READ_WRITE_EXTRA_BYTES+QUAD_READ_DUMMY_BYTES)
#elif ( QSPI_BOOT_READ_CMD==COMMAND_RANDOM_READ )
#define QSPI_BOOT_DUMMY_BYTES	(READ_WRITE_EXTRA_BYTES)
#else
error here
#endif

/**************************** Type Definitions *******************************/
typedef void (*XilAppEntry)(void *data);


/***************** Macros (Inline Functions) Definitions *********************/

#define print_func	xil_printf
#define print_flush_func	 
//#define print_flush_func	 fflush(stdout);

#if 0
#define err_print(format,args...) 			do { print_func(format, ##args); print_flush_func; }while(0)

#else

#define err_print(format,args...)						do { ; }while(0)

#endif

#if 0
#define dbg_print(format,args...) 			do { print_func(format, ##args); print_flush_func; }while(0)
#define dbg_print_line( format,args...) 	do {  print_func("Func: %16s line:%6d: ", __func__, __LINE__);  print_func(format, ##args); print_flush_func; }while(0)
#define dbg_print_var_hex(xxx) 				do {  print_func( "%s = 0x%08x\r\n", #xxx, (unsigned int)xxx ); }while(0)

#else

#define dbg_print(format,args...)			do { ; }while(0)
#define dbg_print_line( format,args...) 	do { ; }while(0)
#define dbg_print_var_hex(xxx)

#endif

#if 1
#define err_put_str(format,args...) 			do { print(format); }while(0)

#else

#define err_put_str(format,args...)			do { ; }while(0)

#endif

/************************** Function Prototypes ******************************/
int FlashReadID(XSpi *SpiPtr);
int SpiFlashWriteEnable(XSpi *SpiPtr);
int SpiFlashWrite(XSpi *SpiPtr, u32 Addr, u32 ByteCount, u8 WriteCmd);
int SpiFlashRead(XSpi *SpiPtr, u32 Addr, u32 ByteCount, u8 ReadCmd);
int SpiFlashReadLong(XSpi *SpiPtr, u8 *ReadBuff, u32 Addr, u32 ByteCount);
int SpiFlashBulkErase(XSpi *SpiPtr);
int SpiFlashSectorErase(XSpi *SpiPtr, u32 Addr);
int SpiFlashGetStatus(XSpi *SpiPtr);
int SpiFlashQuadEnable(XSpi *SpiPtr);
int SpiFlashEnableHPM(XSpi *SpiPtr);
static int SpiFlashWaitForFlashReady(void);
void SpiHandler(void *CallBackRef, u32 StatusEvent, unsigned int ByteCount);
//static int SetupInterruptSystem(XSpi *SpiPtr);

/************************** Variable Definitions *****************************/

/*
 * The instances to support the device drivers are global such that they
 * are initialized to zero each time the program runs. They could be local
 * but should at least be static so they are zeroed.
 */
static XSpi Spi;
//INTC InterruptController;

/*
 * The following variables are shared between non-interrupt processing and
 * interrupt processing such that they must be global.
 */
//volatile static int TransferInProgress;

/*
 * The following variable tracks any errors that occur during interrupt
 * processing.
 */
static int ErrorCount;

/*
 * Buffers used during read and write transactions.
 */
static u8 ReadBuffer[PAGE_SIZE + READ_WRITE_EXTRA_BYTES + 4];
//static u8 WriteBuffer[PAGE_SIZE + READ_WRITE_EXTRA_BYTES];  // 0nly 32 bytes is enough
static u8 WriteBuffer[ 32 + READ_WRITE_EXTRA_BYTES];  // 0nly 32 bytes is enough

/*
 * Byte offset value written to Flash. This needs to be redefined for writing
 * different patterns of data to the Flash device.
 */
//static u8 TestByte = 0x20;

u32 u32_boot_vector_backup_content[ QSPI_BOOT_VECTOR_SIZE/sizeof(u32) ]={};

extern int __rodata_start[];
extern int __rodata_end[];


/************************** Function Definitions *****************************/

// release version does not work if memcpy() is used.
void mb_mem_cpy( u32 *pu32_dst, u32 *pu32_src, u32 u32_byte_len )
{

	int i;

	for( i=0; i<u32_byte_len/sizeof(u32); i++ )
	{
		pu32_dst[i] = pu32_src[i];
	}
}

/*****************************************************************************/
/**
*
* Main function to run the quad flash example.
*
* @param	None
*
* @return	XST_SUCCESS if successful else XST_FAILURE.
*
* @note		None
*
******************************************************************************/
// error for kcu105
int main(void)
{
	int Status;
	u32 *pu32_magic;
	u32 *pu32_app_len;
	u32 u32_app_len;
	XSpi_Config *ConfigPtr;	/* Pointer to Configuration data */

    init_platform();

	/* Disable exceptions and interrupts.	 */
	Xil_ExceptionDisable();

    dbg_print("\n\rQSPI boot test in %s, %s, %s\n\r", __FILE__, __DATE__, __TIME__ );
	dbg_print_var_hex(__rodata_start);
	dbg_print_var_hex(__rodata_end);
	if ( (u32)__rodata_start > QSPI_BOOT_APP_ADDR) {
    	err_print("Warning: Boot is in DDR memory.\n\rError at Line: %d\n\r", __LINE__ );
    	err_put_str("Warning: Boot is in DDR memory.\n\r");
	}

#if 1
	// memcpy (void * dest ,const void *src, size_t n);
	// backup content in vector section of bootrom
    if(  ( 0 == u32_boot_vector_backup_content[0] )
            && ( 0 == u32_boot_vector_backup_content[1] )
			&& ( 0 == u32_boot_vector_backup_content[2] )
			&& ( 0 == u32_boot_vector_backup_content[3] )
			&& ( 0 == u32_boot_vector_backup_content[4] )    )
    {
    	// backup
    	dbg_print("Backup boot vector at Line: %d\n\r", __LINE__ ); 
    	//memcpy( u32_boot_vector_backup_content, (void *)QSPI_BOOT_ROM_ADDR, QSPI_BOOT_VECTOR_SIZE);
    	mb_mem_cpy( u32_boot_vector_backup_content, (u32 *)QSPI_BOOT_ROM_ADDR, QSPI_BOOT_VECTOR_SIZE );
    }
    else
    {
    	// restore
    	dbg_print("Restore boot vector at Line: %d\n\r", __LINE__ );  
    	//memcpy( (void *)QSPI_BOOT_ROM_ADDR, u32_boot_vector_backup_content, QSPI_BOOT_VECTOR_SIZE);
    	mb_mem_cpy( (u32 *)QSPI_BOOT_ROM_ADDR, u32_boot_vector_backup_content, QSPI_BOOT_VECTOR_SIZE );
    }
#endif
   	dbg_print("Backup boot vector done at Line: %d\n\r", __LINE__ );

	/*
	 * Initialize the SPI driver so that it's ready to use,
	 * specify the device ID that is generated in xparameters.h.
	 */
	ConfigPtr = XSpi_LookupConfig(SPI_DEVICE_ID);
	if (ConfigPtr == NULL) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 0353: %d\n\r", __LINE__ );
		return XST_DEVICE_NOT_FOUND;
	}
   	dbg_print("XSpi_LookupConfig done at Line: %d\n\r", __LINE__ );

	Status = XSpi_CfgInitialize(&Spi, ConfigPtr,
				  ConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 0362: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}
   	dbg_print("XSpi_CfgInitialize done at Line: %d\n\r", __LINE__ );

#if 0
	/*
	 * Connect the SPI driver to the interrupt subsystem such that
	 * interrupts can occur. This function is application specific.
	 */
	Status = SetupInterruptSystem(&Spi);
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 0373: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}
	
#endif

	/*
	 * Setup the handler for the SPI that will be called from the interrupt
	 * context when an SPI status occurs, specify a pointer to the SPI
	 * driver instance as the callback reference so the handler is able to
	 * access the instance data.
	 */
	//XSpi_SetStatusHandler(&Spi, &Spi, (XSpi_StatusHandler)SpiHandler);

	/*
	 * Set the SPI device as a master and in manual slave select mode such
	 * that the slave select signal does not toggle for every byte of a
	 * transfer, this must be done before the slave select is set.
	 */
	Status = XSpi_SetOptions(&Spi, XSP_MASTER_OPTION |
				 XSP_MANUAL_SSELECT_OPTION);
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ ); 
    	err_put_str("Error at Line 0396: %d\n\r", __LINE__ ); 
		return XST_FAILURE;
	}
   	dbg_print("XSpi_SetOptions done at Line: %d\n\r", __LINE__ );

	/*
	 * Select the quad flash device on the SPI bus, so that it can be
	 * read and written using the SPI bus.
	 */
	Status = XSpi_SetSlaveSelect(&Spi, SPI_SELECT);
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ ); 
    	err_put_str("Error at Line 0407: %d\n\r", __LINE__ ); 
		return XST_FAILURE;
	}
   	dbg_print("XSpi_SetSlaveSelect done at Line: %d\n\r", __LINE__ );

	/*
	 * Start the SPI driver so that interrupts and the device are enabled.
	 */
	XSpi_Start(&Spi);
  	dbg_print("XSpi_Start done at Line: %d\n\r", __LINE__ );

	XSpi_IntrGlobalDisable(&Spi);
   	dbg_print("XSpi_IntrGlobalDisable done at Line: %d\n\r", __LINE__ );

	//FlashReadID( &Spi );

	// Check Flash status
	Status = SpiFlashGetStatus( &Spi );
	if(Status != XST_SUCCESS) {
    	err_print("SpiFlashGetStatus Error at Line: %d\n\r", __LINE__ ); 
    	err_put_str("SpiFlashGetStatus Error at Line 0422: %d\n\r", __LINE__ ); 
		return XST_FAILURE;
	}
	dbg_print("SpiFlashGetStatus done at Line: %d\n\r", __LINE__ );

	// Clear DDR
	memset((void *)QSPI_BOOT_APP_ADDR, 0, QSPI_BOOT_APP_SIZE);
	dbg_print("DDR is cleared in Boot at Line: %d\n\r", __LINE__ );
		
	// read application header.
	dbg_print("Begin to load application header in Boot at Line: %d\n\r", __LINE__ );
	Status = SpiFlashRead(&Spi, QSPI_BOOT_FLASH_ADDR,
					PAGE_SIZE, QSPI_BOOT_READ_CMD);
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 0437: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	// 0x50: magic word, 'xlnx'= 78 6c 6e 78 .
	pu32_magic = (u32 *)&ReadBuffer[QSPI_BOOT_DUMMY_BYTES+0x50];
	if( pu32_magic[0] != 0x786c6e78 ) {
		err_print("Application header magic word is wrong, %lu=0x%08x\n\r", pu32_magic[0], (unsigned int)pu32_magic[0] );
		err_put_str("Application header magic word is wrong, %lu=0x%08x\n\r", pu32_magic[0], (unsigned int)pu32_magic[0] );
	}
	else {
		dbg_print("Application header magic word is correct, %lu=0x%08x\n\r", pu32_magic[0], (unsigned int)pu32_magic[0] );
	}

	pu32_app_len = (u32 *)&ReadBuffer[QSPI_BOOT_DUMMY_BYTES+0x60];
	pu32_app_len = (u32 *)&ReadBuffer[QSPI_BOOT_DUMMY_BYTES+0x60];
	dbg_print("Application length 1: %lu=0x%08x\n\r", pu32_app_len[0], (unsigned int)pu32_app_len[0] );
	dbg_print("Application length 2: %lu=0x%08x\n\r", pu32_app_len[1], (unsigned int)pu32_app_len[1] );
	

	u32_app_len = QSPI_BOOT_APP_SIZE;
	if( pu32_app_len[0] == pu32_app_len[1] ) {
		
		// value match, check valid again.
		if( pu32_app_len[0] < QSPI_BOOT_APP_SIZE ) {
			u32_app_len = pu32_app_len[0];
		}
	}
	dbg_print("Application length to be used: %lu=0x%08x\n\r", u32_app_len, (unsigned int)u32_app_len );
	
	dbg_print("Begin to load application in Boot at Line: %d\n\r", __LINE__ );
	Status = SpiFlashReadLong(&Spi, (u8 *)QSPI_BOOT_APP_ADDR, 
					QSPI_BOOT_FLASH_ADDR, QSPI_BOOT_APP_SIZE);
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ ); 
    	err_put_str("Error at Line 0472: %d\n\r", __LINE__ ); 
		return XST_FAILURE;
	}
	dbg_print("\n\rLoad application... Done at Line: %d.\n\r", __LINE__ );

	dbg_print("Clear machine statue at Line: %d.\n\r", __LINE__ );

	// clean branch target cache
	mbar(0);
	mbar(1);
	mbar(2);

	// Disable cache
    cleanup_platform();  // disable_caches

	// set exception address.

    // Relocatable base vectors
	
	// Jump to application.
	{
		dbg_print("Jump to application at Line: %d.\n\r", __LINE__ );
		XilAppEntry p_jump_func_ptr=(XilAppEntry)QSPI_BOOT_APP_ADDR;
		p_jump_func_ptr(0);
	}


	return XST_SUCCESS;
}

#if 0

/******************************************************************************
*
* This function reads serial FLASH ID connected to the SPI interface.
*
* @param	None.
*
* @return	XST_SUCCESS if read id, otherwise XST_FAILURE.
*
* @note		None.
*
******************************************************************************/
int FlashReadID(XSpi *SpiPtr)
{
	int i;
	int Status;

	/*
	 * Read ID in Auto mode.
	 */
	WriteBuffer[BYTE1] = READ_ID;
	WriteBuffer[BYTE2] = 0x00;		/* 3 dummy bytes */
	WriteBuffer[BYTE3] = 0x00;
	WriteBuffer[BYTE4] = 0x00;

    for( i=0; i<10; i++ )
    {
    	/*
    	 * Initiate the Transfer.
    	 */
    	//TransferInProgress = TRUE;
    	Status = XSpi_Transfer(SpiPtr, WriteBuffer, ReadBuffer,
    			RD_ID_SIZE);
    	if(Status != XST_SUCCESS) {
        	err_print("Error at Line: %d\n\r", __LINE__ );
        	err_put_str("Error at Line 0541: %d\n\r", __LINE__ );
    		return XST_FAILURE;
    	}

    	/*
    	 * Wait till the Transfer is complete and check if there are any errors
    	 * in the transaction..
    	 */
    	//while(TransferInProgress);
    	if(ErrorCount != 0) {
        	err_print("Error at Line: %d\n\r", __LINE__ );
        	err_put_str("Error at Line 0552: %d\n\r", __LINE__ );
    		ErrorCount = 0;
    		return XST_FAILURE;
    	}
		
    	if( ( 0xff != ReadBuffer[1] )
    		|| ( 0xff != ReadBuffer[2] )
			|| ( 0xff != ReadBuffer[3] ) )
    	{
    		break;
    	}
    }


	dbg_print("FlashID=0x%x 0x%x 0x%x\n\r", ReadBuffer[1], ReadBuffer[2],
		   ReadBuffer[3]);

	return XST_SUCCESS;
}

#endif

#if 0

/*****************************************************************************/
/**
*
* This function enables writes to the Numonyx Serial Flash memory.
*
* @param	SpiPtr is a pointer to the instance of the Spi device.
*
* @return	XST_SUCCESS if successful else XST_FAILURE.
*
* @note		None
*
******************************************************************************/
int SpiFlashWriteEnable(XSpi *SpiPtr)
{
	int Status;

	/*
	 * Wait while the Flash is busy.
	 */
	Status = SpiFlashWaitForFlashReady();
	if(Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Prepare the WriteBuffer.
	 */
	WriteBuffer[BYTE1] = COMMAND_WRITE_ENABLE;

	/*
	 * Initiate the Transfer.
	 */
	//TransferInProgress = TRUE;
	Status = XSpi_Transfer(SpiPtr, WriteBuffer, NULL,
				WRITE_ENABLE_BYTES);
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 0613: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	/*
	 * Wait till the Transfer is complete and check if there are any errors
	 * in the transaction..
	 */
	//while(TransferInProgress);
	if(ErrorCount != 0) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 0624: %d\n\r", __LINE__ );
		ErrorCount = 0;
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function writes the data to the specified locations in the Numonyx Serial
* Flash memory.
*
* @param	SpiPtr is a pointer to the instance of the Spi device.
* @param	Addr is the address in the Buffer, where to write the data.
* @param	ByteCount is the number of bytes to be written.
*
* @return	XST_SUCCESS if successful else XST_FAILURE.
*
* @note		None
*
******************************************************************************/
int SpiFlashWrite(XSpi *SpiPtr, u32 Addr, u32 ByteCount, u8 WriteCmd)
{
	u32 Index;
	int Status;

	/*
	 * Wait while the Flash is busy.
	 */
	Status = SpiFlashWaitForFlashReady();
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 0658: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	/*
	 * Prepare the WriteBuffer.
	 */
	WriteBuffer[BYTE1] = WriteCmd;
	WriteBuffer[BYTE2] = (u8) (Addr >> 16);
	WriteBuffer[BYTE3] = (u8) (Addr >> 8);
	WriteBuffer[BYTE4] = (u8) Addr;


	/*
	 * Fill in the TEST data that is to be written into the Numonyx Serial
	 * Flash device.
	 */
	for(Index = 4; Index < ByteCount + READ_WRITE_EXTRA_BYTES; Index++) {
		WriteBuffer[Index] = (u8)((Index - 4) + TestByte);
	}

	/*
	 * Initiate the Transfer.
	 */
	//TransferInProgress = TRUE;
	Status = XSpi_Transfer(SpiPtr, WriteBuffer, NULL,
				(ByteCount + READ_WRITE_EXTRA_BYTES));
	if(Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Wait till the Transfer is complete and check if there are any errors
	 * in the transaction.
	 */
	//while(TransferInProgress);
	if(ErrorCount != 0) {
		ErrorCount = 0;
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 0697: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

#endif

/*****************************************************************************/
/**
*
* This function reads the data from the Numonyx Serial Flash Memory
*
* @param	SpiPtr is a pointer to the instance of the Spi device.
* @param	Addr is the starting address in the Flash Memory from which the
*		data is to be read.
* @param	ByteCount is the number of bytes to be read.
*
* @return	XST_SUCCESS if successful else XST_FAILURE.
*
* @note		None
*
******************************************************************************/
int SpiFlashRead(XSpi *SpiPtr, u32 Addr, u32 ByteCount, u8 ReadCmd)
{
	int Status;

	/*
	 * Wait while the Flash is busy.
	 */
	Status = SpiFlashWaitForFlashReady();
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 0731: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	/*
	 * Prepare the WriteBuffer.
	 */
	WriteBuffer[BYTE1] = ReadCmd;
	WriteBuffer[BYTE2] = (u8) (Addr >> 16);
	WriteBuffer[BYTE3] = (u8) (Addr >> 8);
	WriteBuffer[BYTE4] = (u8) Addr;

	if (ReadCmd == COMMAND_DUAL_READ) {
		ByteCount += DUAL_READ_DUMMY_BYTES;
	} else if (ReadCmd == COMMAND_DUAL_IO_READ) {
		ByteCount += DUAL_READ_DUMMY_BYTES;
	} else if (ReadCmd == COMMAND_QUAD_IO_READ) {
		ByteCount += QUAD_IO_READ_DUMMY_BYTES;
	} else if (ReadCmd==COMMAND_QUAD_READ) {
		ByteCount += QUAD_READ_DUMMY_BYTES;
	}

	/*
	 * Initiate the Transfer.
	 */
	//TransferInProgress = TRUE;
	Status = XSpi_Transfer( SpiPtr, WriteBuffer, ReadBuffer,
				(ByteCount + READ_WRITE_EXTRA_BYTES));
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 0761: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	/*
	 * Wait till the Transfer is complete and check if there are any errors
	 * in the transaction.
	 */
	//while(TransferInProgress);
	if(ErrorCount != 0) {
		ErrorCount = 0;
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 0773: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}


/*****************************************************************************/
/**
*
* This function reads the data from the Numonyx Serial Flash Memory
*
* @param	SpiPtr is a pointer to the instance of the Spi device.
* @param	ReadBuff is the starting address in the RAM from which the
*		data is to be written.
* @param	Addr is the starting address in the Flash Memory from which the
*		data is to be read.
* @param	ByteCount is the number of bytes to be read.
*
* @return	XST_SUCCESS if successful else XST_FAILURE.
*
* @note		None
*
******************************************************************************/
int SpiFlashReadLong(XSpi *SpiPtr, u8 *ReadBuff, u32 Addr, u32 ByteCount)
{
	int	i,j;
	u32 AddrCurrent;
	u32 AddrNext;
	u32 ByteCountLeft;
	u32 ByteCountCurrent;
	int Status;

	dbg_print("QSPI boot read flash:0x%08x at Line:%d\n\r", (unsigned int)Addr, __LINE__ );
	dbg_print("WriteBuffer:0x%08x\n\r", (unsigned int)WriteBuffer );
	dbg_print("ReadBuffer:0x%08x\n\r", (unsigned int)ReadBuffer );

	AddrNext = Addr;
	ByteCountLeft=ByteCount;
    for( i=0; ByteCountLeft>0; i++ )
    {
    	//dbg_print("No. %d QSPI boot read at Line:%d\n\r", i, __LINE__ );
    	//dbg_print("Old ByteCountLeft:%lu=0x%08x\n\r", ByteCountLeft, (unsigned int)ByteCountLeft );
		if( 0== (i%16) ) {
			dbg_print(".");
		}

		if( ByteCountLeft>PAGE_SIZE ) {
	    	ByteCountCurrent=PAGE_SIZE;
		}
		else{
	    	ByteCountCurrent=ByteCountLeft;
		}
		AddrCurrent = AddrNext;
		AddrNext += ByteCountCurrent;
		ByteCountLeft -= ByteCountCurrent;
    	//dbg_print("AddrCurrent:%lu=0x%08x\n\r", AddrCurrent, (unsigned int)AddrCurrent );
    	//dbg_print("AddrNext:%lu=0x%08x\n\r", AddrNext, (unsigned int)AddrNext );
    	//dbg_print("ByteCountCurrent:%lu=0x%08x\n\r", ByteCountCurrent, (unsigned int)ByteCountCurrent );
    	//dbg_print("New ByteCountLeft:%lu=0x%08x\n\r", ByteCountLeft, (unsigned int)ByteCountLeft );

		Status = SpiFlashRead(SpiPtr, AddrCurrent, ByteCountCurrent, QSPI_BOOT_READ_CMD);
		if(Status != XST_SUCCESS) {
	    	err_print("Error at Line: %d\n\r", __LINE__ ); 
	    	err_put_str("Error at Line 0838: %d\n\r", __LINE__ ); 
			return XST_FAILURE;
		}

#if 1
		dbg_print("%02x %02x %02x %02x %02x %02x %02x %02x \n\r",
				ReadBuffer[0 + QSPI_BOOT_DUMMY_BYTES],
				ReadBuffer[1 + QSPI_BOOT_DUMMY_BYTES],
				ReadBuffer[2 + QSPI_BOOT_DUMMY_BYTES],
				ReadBuffer[3 + QSPI_BOOT_DUMMY_BYTES],
				ReadBuffer[4 + QSPI_BOOT_DUMMY_BYTES],
				ReadBuffer[5 + QSPI_BOOT_DUMMY_BYTES],
				ReadBuffer[6 + QSPI_BOOT_DUMMY_BYTES],
				ReadBuffer[7 + QSPI_BOOT_DUMMY_BYTES]
				);
#endif

#if 1
    	{
    		// for demo only.
    		static int i_null_line_cnt=0;
    		if( ( 0xff ==	ReadBuffer[0 + QSPI_BOOT_DUMMY_BYTES] )
    			&& ( 0xff == ReadBuffer[1 + QSPI_BOOT_DUMMY_BYTES] )
				&& ( 0xff == ReadBuffer[2 + QSPI_BOOT_DUMMY_BYTES] )
				&& ( 0xff == ReadBuffer[3 + QSPI_BOOT_DUMMY_BYTES] )
				&& ( 0xff == ReadBuffer[4 + QSPI_BOOT_DUMMY_BYTES] )
				&& ( 0xff == ReadBuffer[5 + QSPI_BOOT_DUMMY_BYTES] )
				&& ( 0xff == ReadBuffer[6 + QSPI_BOOT_DUMMY_BYTES] )
				&& ( 0xff == ReadBuffer[7 + QSPI_BOOT_DUMMY_BYTES] ) )
			{
				i_null_line_cnt++;
				if( i_null_line_cnt>10)
				{
					break;  // null data, app might be loaded successfully.
				}
			}
    	}
#endif

		for( j=0; j<ByteCountCurrent; j++ )
		{
			ReadBuff[i*PAGE_SIZE+j] = 	ReadBuffer[QSPI_BOOT_DUMMY_BYTES+j];
		}
		
	}

	return XST_SUCCESS;
}

#if 0

/*****************************************************************************/
/**
*
* This function erases the entire contents of the Numonyx Serial Flash device.
*
* @param	SpiPtr is a pointer to the instance of the Spi device.
*
* @return	XST_SUCCESS if successful else XST_FAILURE.
*
* @note		The erased bytes will read as 0xFF.
*
******************************************************************************/
int SpiFlashBulkErase(XSpi *SpiPtr)
{
	int Status;

	/*
	 * Wait while the Flash is busy.
	 */
	Status = SpiFlashWaitForFlashReady();
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	/*
	 * Prepare the WriteBuffer.
	 */
	WriteBuffer[BYTE1] = COMMAND_BULK_ERASE;

	/*
	 * Initiate the Transfer.
	 */
	//TransferInProgress = TRUE;
	Status = XSpi_Transfer(SpiPtr, WriteBuffer, NULL,
						BULK_ERASE_BYTES);
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 0927: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	/*
	 * Wait till the Transfer is complete and check if there are any errors
	 * in the transaction..
	 */
	//while(TransferInProgress);
	if(ErrorCount != 0) {
		ErrorCount = 0;
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 0938: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function erases the contents of the specified Sector in the Numonyx
* Serial Flash device.
*
* @param	SpiPtr is a pointer to the instance of the Spi device.
* @param	Addr is the address within a sector of the Buffer, which is to
*		be erased.
*
* @return	XST_SUCCESS if successful else XST_FAILURE.
*
* @note		The erased bytes will be read back as 0xFF.
*
******************************************************************************/
int SpiFlashSectorErase(XSpi *SpiPtr, u32 Addr)
{
	int Status;

	/*
	 * Wait while the Flash is busy.
	 */
	Status = SpiFlashWaitForFlashReady();
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 0971: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	/*
	 * Prepare the WriteBuffer.
	 */
	WriteBuffer[BYTE1] = COMMAND_SECTOR_ERASE;
	WriteBuffer[BYTE2] = (u8) (Addr >> 16);
	WriteBuffer[BYTE3] = (u8) (Addr >> 8);
	WriteBuffer[BYTE4] = (u8) (Addr);

	/*
	 * Initiate the Transfer.
	 */
	//TransferInProgress = TRUE;
	Status = XSpi_Transfer(SpiPtr, WriteBuffer, NULL,
					SECTOR_ERASE_BYTES);
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 0991: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	/*
	 * Wait till the Transfer is complete and check if there are any errors
	 * in the transaction..
	 */
	//while(TransferInProgress);
	if(ErrorCount != 0) {
		ErrorCount = 0;
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

#endif

/*****************************************************************************/
/**
*
* This function reads the Status register of the Numonyx Flash.
*
* @param	SpiPtr is a pointer to the instance of the Spi device.
*
* @return	XST_SUCCESS if successful else XST_FAILURE.
*
* @note		The status register content is stored at the second byte
*		pointed by the ReadBuffer.
*
******************************************************************************/
int SpiFlashGetStatus(XSpi *SpiPtr)
{
	int Status;

	/*
	 * Prepare the Write Buffer.
	 */
	WriteBuffer[BYTE1] = COMMAND_STATUSREG_READ;
	ReadBuffer[0 + 0] = 0;
	ReadBuffer[0 + 1] = 0;

	/*
	 * Initiate the Transfer.
	 */
	//TransferInProgress = TRUE;
	Status = XSpi_Transfer(SpiPtr, WriteBuffer, ReadBuffer,
						STATUS_READ_BYTES);
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 1044: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	/*
	 * Wait till the Transfer is complete and check if there are any errors
	 * in the transaction..
	 */
	//while(TransferInProgress);
	if(ErrorCount != 0) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line 1055: %d\n\r", __LINE__ );
		ErrorCount = 0;
		return XST_FAILURE;
	}

	//dbg_print("SPI Flash Status: %02x\n\r", ReadBuffer[0 + 1] );

	return XST_SUCCESS;
}



/*****************************************************************************/
/**
*
* This function waits till the Numonyx serial Flash is ready to accept next
* command.
*
* @param	None
*
* @return	XST_SUCCESS if successful else XST_FAILURE.
*
* @note		This function reads the status register of the Buffer and waits
*.		till the WIP bit of the status register becomes 0.
*
******************************************************************************/
int SpiFlashWaitForFlashReady(void)
{
	int Status;
	u8 StatusReg;

	while(1) {

		/*
		 * Get the Status Register. The status register content is
		 * stored at the second byte pointed by the ReadBuffer.
		 */
		Status = SpiFlashGetStatus(&Spi);
		if(Status != XST_SUCCESS) {
	    	err_print("Error at Line: %d\n\r", __LINE__ );
	    	err_put_str("Error at Line 1095: %d\n\r", __LINE__ );
			return XST_FAILURE;
		}

		/*
		 * Check if the flash is ready to accept the next command.
		 * If so break.
		 */
		StatusReg = ReadBuffer[1];
		if((StatusReg & FLASH_SR_IS_READY_MASK) == 0) {
			break;
		}
	}

	return XST_SUCCESS;
}

#if 0
/*****************************************************************************/
/**
*
* This function is the handler which performs processing for the SPI driver.
* It is called from an interrupt context such that the amount of processing
* performed should be minimized. It is called when a transfer of SPI data
* completes or an error occurs.
*
* This handler provides an example of how to handle SPI interrupts and
* is application specific.
*
* @param	CallBackRef is the upper layer callback reference passed back
*		when the callback function is invoked.
* @param	StatusEvent is the event that just occurred.
* @param	ByteCount is the number of bytes transferred up until the event
*		occurred.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void SpiHandler(void *CallBackRef, u32 StatusEvent, unsigned int ByteCount)
{
	/*
	 * Indicate the transfer on the SPI bus is no longer in progress
	 * regardless of the status event.
	 */
	//TransferInProgress = FALSE;

	/*
	 * If the event was not transfer done, then track it as an error.
	 */
	if (StatusEvent != XST_SPI_TRANSFER_DONE) {
    	err_print("XST_SPI_TRANSFER_DONE Error: %d at Line: %d\n\r",ErrorCount,  __LINE__ );
    	err_put_str("XST_SPI_TRANSFER_DONE Error: %d at Line: %d\n\r",ErrorCount,  __LINE__ );
		ErrorCount++;
	}
}
#endif

#if 0
/*****************************************************************************/
/**
*
* This function setups the interrupt system such that interrupts can occur
* for the Spi device. This function is application specific since the actual
* system may or may not have an interrupt controller. The Spi device could be
* directly connected to a processor without an interrupt controller.  The
* user should modify this function to fit the application.
*
* @param	SpiPtr is a pointer to the instance of the Spi device.
*
* @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
*
* @note		None
*
******************************************************************************/
static int SetupInterruptSystem(XSpi *SpiPtr)
{

	int Status;

#ifdef XPAR_INTC_0_DEVICE_ID
	/*
	 * Initialize the interrupt controller driver so that
	 * it's ready to use, specify the device ID that is generated in
	 * xparameters.h
	 */
	Status = XIntc_Initialize(&InterruptController, INTC_DEVICE_ID);
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	/*
	 * Connect a device driver handler that will be called when an
	 * interrupt for the device occurs, the device driver handler
	 * performs the specific interrupt processing for the device
	 */
	Status = XIntc_Connect(&InterruptController,
				SPI_INTR_ID,
				(XInterruptHandler)XSpi_InterruptHandler,
				(void *)SpiPtr);
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	/*
	 * Start the interrupt controller such that interrupts are enabled for
	 * all devices that cause interrupts, specific real mode so that
	 * the SPI can cause interrupts thru the interrupt controller.
	 */
	Status = XIntc_Start(&InterruptController, XIN_REAL_MODE);
	if(Status != XST_SUCCESS) {
    	err_print("Error at Line: %d\n\r", __LINE__ );
    	err_put_str("Error at Line: %d\n\r", __LINE__ );
		return XST_FAILURE;
	}

	/*
	 * Enable the interrupt for the SPI.
	 */
	XIntc_Enable(&InterruptController, SPI_INTR_ID);

#else /* SCUGIC */
	XScuGic_Config *IntcConfig;

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(&InterruptController, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}


	XScuGic_SetPriorityTriggerType(&InterruptController, SPI_INTR_ID,
					0xA0, 0x3);


	/*
	 * Connect the interrupt handler that will be called when an
	 * interrupt occurs for the device.
	 */
	Status = XScuGic_Connect(&InterruptController, SPI_INTR_ID,
				 (Xil_InterruptHandler)XSpi_InterruptHandler,
					 (void *)SpiPtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	/*
	 * Enable the interrupt for the SPI device.
	 */
	XScuGic_Enable(&InterruptController, SPI_INTR_ID);


#endif

	/*
	 * Initialize the exception table.
	 */
	Xil_ExceptionInit();

	/*
	 * Register the interrupt controller handler with the exception table.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				(Xil_ExceptionHandler)INTC_HANDLER,
				&InterruptController);

	/*
	 * Enable non-critical exceptions.
	 */
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

#endif

