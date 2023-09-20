#ifndef DWIN_LIB_H
#define DWIN_LIB_H

/* ----------------------- Includes -----------------------------------------*/
//#include "stm32f4xx_hal_uart.h"

#include "hmi_interface.h"
#include "dma_cmsis.h"
#include "DWIN_port.h"
#include "config_DWIN.h"
#include "DWIN_addr_conv.h"

/* ----------------------- Defines ------------------------------------------*/
#define DWIN_HMI_PACKET_DATA_SIZE		 248
#define DWIN_ADDR_START_DATA					3
#define DWIN_CRC_ENABLE							( 0 )

#define DWIN_ADDRESS_BROADCAST    	( 0 )   /*! Modbus broadcast address. */
#define DWIN_ADDRESS_MIN          	( 1 )   /*! Smallest possible slave address. */
#define DWIN_ADDRESS_MAX          	( 247 ) /*! Biggest possible slave address. */
#define DWIN_SER_PDU_SIZE_MAX				( 256 )

#define DWIN_START_BIT							( 0x5A )

#define DWIN_START_POS							( 0 )
#define DWIN_SLAVE_ID_POS						( 1 )
#define DWIN_WORD_LENGTH_POS				( 2 )
#define DWIN_CMD_POS								( 3 )


#define TRUE            1
#define FALSE           0

typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

/* ----------------------- Structs ------------------------------------------*/
typedef struct DWIN_HMI_PACKET {
	uint8_t start_byte;
	uint8_t slave_ID;
	uint8_t cmd;
	uint8_t word_cnt;
	uint8_t wait_time;
	uint8_t mode_trsmt;
	uint16_t extra_address;
	uint16_t hmi_address;
	uint16_t slave_address;
}t_dwin_him_pack;

/* ----------------------- Type definitions ---------------------------------*/

/*! \ingroup modbus
 * \brief Parity used for characters in serial mode.
 *
 * The parity which should be applied to the characters sent over the serial
 * link. Please note that this values are actually passed to the porting
 * layer and therefore not all parity modes might be available.
 */
typedef enum
{
    DWIN_PAR_NONE,                /*!< No parity. */
    DWIN_PAR_ODD,                 /*!< Odd parity. */
    DWIN_PAR_EVEN                 /*!< Even parity. */
} eDWINParity;

typedef enum
{
    DWIN_EV_READY,                   /*!< Startup finished. */
    DWIN_EV_FRAME_RECEIVED,          /*!< Start frame received. */
    DWIN_EV_EXECUTE,                 /*!< Execute function. */
    DWIN_EV_FRAME_SENT               /*!< Frame sent. */
} eDWINEventType;

/*!
 * \brief DWIN serial transmission modes (SLAVE/MASTER).
 */

typedef enum {
	DWIN_SLAVE, 					/*!< HMI is slave */
	DWIN_MASTER, 					/*!< HMI is master */
} eDWINMode;

/*!
 * \brief Errorcodes used by all function in the protocol stack.
 */
typedef enum
{
    DWIN_ENOERR,                  /*!< no error. */
    DWIN_ENOREG,                  /*!< illegal register address. */
    DWIN_EINVAL,                  /*!< illegal argument. */
    DWIN_EPORTERR,                /*!< porting layer error. */
    DWIN_ENORES,                  /*!< insufficient resources. */
    DWIN_EIO,                     /*!< I/O error. */
    DWIN_EILLSTATE,               /*!< protocol stack in illegal state. */
    DWIN_ETIMEDOUT,               /*!< timeout error occurred. */
	  DWIN_TX_BUSY,									/*!< Tx is busy */
} eDWINErrorCode;


/*!
 * \brief If register should be written or read.
 *
 * This value is passed to the callback functions which support either
 * reading or writing register values. Writing means that the application
 * registers should be updated and reading means that the modbus protocol
 * stack needs to know the current register values.
 */
typedef enum
{
    DWIN_REG_READ,                /*!< Read register values and pass to protocol stack. */
    DWIN_REG_WRITE                /*!< Update register values. */
} eDWINRegisterMode;

/* ----------------------- Prototypes  0-------------------------------------*/
typedef void    ( *pvDWINFrameStart ) ( void );

typedef void    ( *pvDWINFrameStop ) ( void );

typedef eDWINErrorCode( *peDWINFrameReceive ) ( UCHAR * pucRcvAddress,
                                            UCHAR * pucFrame,
                                            USHORT * pusLength, eDWINEventType * evType );

typedef eDWINErrorCode( *peDWINFrameSend ) ( UCHAR slaveAddress,
                                         const UCHAR * pucFrame,
                                         USHORT usLength );

typedef void( *pvDWINFrameClose ) ( void );

/* --------------------- Common dunctional -----------------------------------*/

eDWINErrorCode eDWINInit(UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eDWINParity eParity);

void eDWINStart( void );
																				 
void eDWINStop( void );		
																				 
void eDWINClose( void );		
																				 
eDWINErrorCode eDWINReceive( UCHAR * pucRcvAddress, UCHAR * pucFrame, USHORT * pusLength, eDWINEventType * evType );
												 
eDWINErrorCode eDWINSend( UCHAR slaveAddress, const UCHAR * pucFrame, USHORT usLength );

eDWINErrorCode eDWINRequestSend( UCHAR slaveAddress, const UCHAR * pucFrame, USHORT usLength );
																				 
eDWINErrorCode eDWINEnable( void );
																				 
void eDWINDisable( void );
																				 
void eDWINSetSlaveID( void );

BOOL xDWINReceiveFSM ( void );

eDWINErrorCode eDWINPoll( void );

BOOL xDWINPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eDWINParity eParity );

void vDWINPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable );

/* ----------------------- Supporting functions -----------------------------*/
BOOL xDWINPortEventInit( void );

BOOL xDWINPortEventPost( eDWINEventType eEvent );

BOOL xDWINPortEventGet(eDWINEventType* eEvent);

BOOL xDWINSetQueue( void );

/* ----------------------- Handle functions -----------------------------*/
void eDWINFuncReadRegister(UCHAR * pucFrame, USHORT * registers, USHORT * usLen);

void eDWINFuncWriteRegister(UCHAR * pucFrame, USHORT * registers, USHORT * usLen);

#endif	//DWIN_LIB_H