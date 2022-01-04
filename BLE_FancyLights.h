/*******************************************************************************/
/* File Name          : BlueFruit_BLE.h
 * Author             : Nagaraju
 * Date               : 08/06/2019
 * Description        : Header for BlueFruit_BLE.c file.
*******************************************************************************/

/* Exported Defines --------------------------------------------------------- */
// ----------------------------------------------------------------------------------------------
// These settings are used in both SW UART, HW UART and SPI mode
// ----------------------------------------------------------------------------------------------
#define BUFSIZE                        128   // Size of the read buffer for incoming data
#define VERBOSE_MODE                   true  // If set to 'true' enables debug output


// SOFTWARE UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins that will be used for 'SW' serial.
// You should use this option if you are connecting the UART Friend to an UNO
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SWUART_RXD_PIN       6   // Required for software serial!
#define BLUEFRUIT_SWUART_TXD_PIN       7   // Required for software serial!
#define BLUEFRUIT_UART_CTS_PIN         8   // Required for software serial!
#define BLUEFRUIT_UART_RTS_PIN         5   // Optional, set to -1 if unused

// RGB LED
#define 	RED_LED     			 11
#define 	GREEN_LED    			 10
#define 	BLUE_LED    			  9
#define 	RSSI_LED    			  4
#define 	PWR_SWITCH      		  2   // switch 1


// HARDWARE UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the HW serial port you are using. Uncomment
// this line if you are connecting the BLE to Leonardo/Micro or Flora
// ----------------------------------------------------------------------------------------------
#ifdef Serial1    // this makes it not complain on compilation if there's no Serial1
  #define BLUEFRUIT_HWSERIAL_NAME      Serial1
#endif


// SHARED UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following sets the optional Mode pin, its recommended but not required
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_UART_MODE_PIN        -1    // Set to -1 if unused


#define  FACTORYRESET_ENABLE          0
#define  MINIMUM_FIRMWARE_VERSION    "0.6.7"
#define  MODE_LED_BEHAVIOUR          "MODE"

#define NUM_COLORS  21
float colors[NUM_COLORS][3] ={{0.3,0.0,0.0},{0.7,0.0,0.0},{1.0,0.0,0.0}, 
    						  {0.7,0.3,0.0},{0.3,0.7,0.0},{0.0,1.0,0.0}, 
    						  {0.0,0.7,0.3},{0.0,0.3,0.7},{0.0,0.0,1.0}, 
			                  {0.3,0.3,0.7},{0.7,0.7,0.3},{1.0,1.0,0.0}, 
			                  {1.0,0.7,0.3},{1.0,0.3,0.7},{1.0,0.0,1.0},
			                  {0.7,0.3,1.0},{0.3,0.7,1.0},{0.0,1.0,1.0},
			                  {0.3,1.0,1.0},{0.7,1.0,1.0},{1.0,1.0,1.0}};


// ----------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------



