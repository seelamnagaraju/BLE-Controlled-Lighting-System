/************************************************************/
/************************************************************/
/*                                                          */
/* University of California Extension, Santa Cruz           */
/*                                                          */
/* The Internet of Things: Communication and Cloud          */
/*                                                          */
/*                                                          */
/* Author: Nagaraju Seelam                                  */
/*                                                          */
/* Project 2 :  BLE Controlled lighting System              */
/*                                                          */
/* FileName : BLE_FancyLights.c                             */
/*                                                          */
/*  Date: 08/06/2019                                        */
/*                                                          */
/*  Objective:                                              */
/*  Implementing an BLE Controlled fancy lighting system    */
/*  based on nRF51822 Bluefruit LE module                   */
/*                                                          */
/*  Sketch URL : https://codebender.cc/sketch:357793        */
/*                                                          */
/************************************************************/
/************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <Arduino.h>
#include <Timer.h>

#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BlueFruit_BLE.h"


/* Private typedef -----------------------------------------------------------*/
typedef enum 
{
  LED_STEATE  = 1, 
  LED_SWITCH  = 2
} t_Charactristic;

typedef enum 
{
  GATT_READ  = 0, 
  GATT_WRITE = 1
} t_Char_RW;

/* Private define ------------------------------------------------------------*/
#define   BLE_NAME        "NAGARAJU"

#define BLE_LED_GATT_SERVICE "UUID128=cf-4d-de-38-5b-65-11-e6-8b-77-86-f3-0c-a8-93-d3"
#define BLE_LED_STATE_CHARACTRISTIC   "UUID=0x0002,PROPERTIES=0x08,MIN_LEN=1,VALUE=0"
#define BLE_LED_SWITCH_CHARACTRISTIC  "UUID=0x0003,PROPERTIES=0x12,MIN_LEN=1,VALUE=2"

#define  BLE_DATA_HANDLE_PERIOD         200
#define  SET_LED_STATE_PERIOD           500
#define  LED_GATT_HANDLE_PERIOD         1000
#define  GET_RSSI_PERIOD         		1500
#define  SET_NW_LED_STATE_PERIOD        3000

#define  RSSI_CUTOFF				   (-70)

/* Function Objects -----------------------------------------------------------*/
// Create the bluefruit object FOR software serial
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);
Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN, BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

/* Global Functions -----------------------------------------------------------*/
// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/* Private Functions -----------------------------------------------------------*/
int gpio_init(void);
int BLEmodule_init(void);
void setLedState(void *);
void BLE_ReceiveData_Handler(void*);
void LED_ServiceHandle(void*);
void get_RSSI(void * );
void NW_Blink(void*);
void Color_Change(char option);
void Brightness_Change(char option);
int create_LED_custom_GATT_Service(void);
uint16_t Read_Write_GATTcharacteristic(t_Charactristic i_characteristic, t_Char_RW i_rw, uint32_t * o_vlaue);

/* Global variables ---------------------------------------------------------*/
Timer    timer;
static int brightness   = 0;    
static const int fadeAmount = 10;   
static int  Current_RSSI =-99;
volatile int RGD_Color   = 0;
volatile boolean ledState  = LOW;
char Led_F=0;
char RSSI_flag=0;

/* Private variables ---------------------------------------------------------*/


/**************************************************************************/
/*!
  * @brief  	  setup : Setup the UNO HW and the BLE module
  * @param[out]   None
  * @param[in]    None
  * @retval 	  None
  */
/**************************************************************************/
void setup(void)
{
	gpio_init(); // HW init
	
	BLEmodule_init(); // BLE module initialisation
	
	//-------  Running application -------------------------------------
	timer.every(BLE_DATA_HANDLE_PERIOD, BLE_ReceiveData_Handler, NULL);
	timer.every(LED_GATT_HANDLE_PERIOD, LED_ServiceHandle, NULL);
	timer.every(GET_RSSI_PERIOD, get_RSSI, NULL);
	//------------------------------------------------------------
     Serial.println(); 
  
}


/**************************************************************************/
/*!
  * @brief  	  loop : Running application
  * @param[out]   None
  * @param[in]    None
  * @retval 	  None
  */
/**************************************************************************/
void loop(void)
{
    timer.update();
    setLedState(NULL);
}


/**************************************************************************/
/*!
  * @brief  	  BLE_ReceiveData_Handler : Processing received Commands
  * @param[out]   None
  * @param[in]    None
  * @retval 	  None
  */
/**************************************************************************/
void BLE_ReceiveData_Handler(void*)
{
	char n, inputs[BUFSIZE+1];
	char l_buffer[16];
	uint32_t led_State;
	
	if (RSSI_flag==0) {  /*Serial.println("Low Signal");*/return;	 }
	ble. flush();
	
	if (Serial.available()) {
		// Send characters to Bluefruit
		Serial.print("[Send] ");
		Serial.println(inputs);
		ble.print("AT+BLEUARTTX=");
		ble.println(inputs);
		if (! ble.waitForOK()) { Serial.println(F("send Fail")); }
	}

	memset(l_buffer, 0x00, sizeof(l_buffer)); 
	// Check for incoming characters from Bluefruit
	ble.println("AT+BLEUARTRX"); 
	ble.readline();
	if (strcmp(ble.buffer, "OK") == 0) { return; }  // no data
	// data was found, its in the buffer
	strcpy(l_buffer, ble.buffer);
	ble.waitForOK();	
	led_State = ledState;
 
	if (strcmp(l_buffer, "SW") == 0) {
		Led_F=0;
		ledState = !ledState;
		delay(1);
		led_State = ledState;
		Serial.print("\nControlling led:");
		Serial.println(led_State);
		Read_Write_GATTcharacteristic(LED_STEATE, GATT_WRITE, &led_State); 
	}
	else if (strcmp(l_buffer, "LF") == 0) {
		Serial.print("\n Change Color Inc. \n");
		Color_Change(1);
	}

   else if (strcmp(l_buffer, "RT") == 0)  {
   		Serial.print("\n Change Color Dec. \n");
   		Color_Change(0);
   }
   
   else if (strcmp(l_buffer, "UP") == 0)  {
   		Serial.print("\n Change Brightness Inc. : ");
   		Serial.println(brightness);
   		Brightness_Change(1);
   }
   else if (strcmp(l_buffer, "DN") == 0)  {
   		Serial.print("\n Change Brightness Dec. : ");
   		Serial.println(brightness);
   		Brightness_Change(0);
   }
   return;
}


/**************************************************************************/
/*!
  * @brief  	  Read_Write_GATTcharacteristic : Read/Write GATT characteristics
  * @param[out]   characteristic vlalue for set/get
  * @param[in]    characteristic
  * @param[in]    i_rw=0:read; 1:write
  * @retval 	  Function status
  */
/**************************************************************************/
 uint16_t Read_Write_GATTcharacteristic(t_Charactristic i_characteristic, t_Char_RW i_rw, uint32_t * o_vlaue)
 {
	char buffer[10];
	if(i_rw==GATT_WRITE) {
	 	ble.print( F("AT+GATTCHAR=") );
  		ble.print(i_characteristic);
  		ble.print(',');
  		ble.println(*o_vlaue);
  		ble.waitForOK();
	}
	if(i_rw==GATT_READ)  {
		memset(buffer, 0x00, sizeof(buffer)); 
		ble.print( F("AT+GATTCHAR="));
		ble.println(i_characteristic); // (i_characteristic, HEX);
		ble.readline();
		if (strcmp(ble.buffer, "OK") == 0) { Serial.println("No data"); return 1;}
		strcpy(buffer, ble.buffer);
		ble.waitForOK();
		*o_vlaue = atoi(&buffer[2]);
	}
	return 0;
}


/**************************************************************************/
/*!
  * @brief  	  NW_Blink   : RSSI LED BLINK
	@				No rssi  : NW Led off
	@				rssi     : single LED Blink
	@				Low rssi : Multi flash
  * @param[out]   None
  * @param[in]    None
  * @retval 	  None
  */
/**************************************************************************/
void NW_Blink(void*)
{
	char i, j;
	int toggle=1;
	
	if(Current_RSSI<RSSI_CUTOFF)  { /*Serial.println("Low rssi");*/
		 RSSI_flag=0;  j = 6; 
	}
	else { 
		RSSI_flag=1;  j = 2; 
	}

	for(i=0; i<j; i++)	{
		switch(toggle) 
		{ 
		case 0:
		    toggle=1;  
		    digitalWrite(RSSI_LED, LOW);   // turn the LED off by making the voltage LOW
		    break;
		case 1:
		    toggle=0;  
		    digitalWrite(RSSI_LED, HIGH);   // turn the LED on (HIGH is the voltage level) 
		    break;
		} 	delay(100);
	}
	return;
}


/**************************************************************************/
/*!
  * @brief  	  get_RSSI   : get RSSI of connected device
  * @param[out]   Current_RSSI : saves the current RSSI
  * @param[in]    None
  * @retval 	  None
  */
/**************************************************************************/
void get_RSSI(void *)
 {
	char buffer[16];
	if (! ble.isConnected()) { /* Serial.println("No rssi");  */
		RSSI_flag=0; Current_RSSI=-99; return;
	}
	RSSI_flag=1;
	memset(buffer, 0x00, sizeof(buffer));   
	ble.println("AT+BLEGETRSSI");
	ble.readline();
	if (strcmp(ble.buffer, "OK") == 0) { /*Serial.println("No data"); */
	}
	strcpy(buffer, ble.buffer);
	ble.waitForOK();
	Current_RSSI = atoi(&buffer[0]);
	NW_Blink(NULL);
	return;
}

/**************************************************************************/
/*!
  * @brief  	  LED_ServiceHandle  : Handler for LED custom services
  * @param[out]   ledState 
  * @param[in]    None
  * @retval 	  None
  */
/**************************************************************************/
void LED_ServiceHandle(void*)
{
	uint32_t led_State, ch;
	led_State=ledState;
	if (RSSI_flag==0) {	/*Serial.println("Low Signal2"); */
		return;  
	}
	Read_Write_GATTcharacteristic(LED_STEATE, GATT_WRITE, &led_State); 
	Read_Write_GATTcharacteristic(LED_SWITCH, GATT_READ, &ch);
	if(ch == 11) {   // Led on	
		ledState=1;	
	}  
	if(ch == 22) {   // Led off
		ledState=0;  
	}  
	else {  /*Serial.println("Invalid option"); */ 
	} 
	return;
}


/**************************************************************************/
/*!
  * @brief  	  setLedState  : Handler for RGB LED on/off
  * @param[out]   None 
  * @param[in]    Led_F  : brightness flag, if set return from function 
  * @param[in]    ledState: 0:off; 1:on led
  * @retval 	  None
  */
/**************************************************************************/
void setLedState(void *) 
{
	if (Led_F) return;
	if (!ledState) 	{
		analogWrite(RED_LED, 0);
		analogWrite(GREEN_LED, 0);
		analogWrite(BLUE_LED, 0);
		delay(10);
		return;
	}
	else {
		analogWrite(RED_LED, colors[RGD_Color][0]*255);
		analogWrite(GREEN_LED, colors[RGD_Color][1]*255);
		analogWrite(BLUE_LED, colors[RGD_Color][2]*255);
	}
	return;
}	


/**************************************************************************/
/*!
  * @brief  	  OnOffSwitchPushed  : push button interrupt Handler
  * @param[in]    None 
  * @param[out]   ledState: Toggles led flag state
  * @retval 	  None
  */
/**************************************************************************/
void OnOffSwitchPushed(void) 
{
  static long lastTime = 0;
  Led_F=0;
  if (millis() - lastTime < 200) return;
  lastTime = millis();
  Serial.println("Toggle LED");
  ledState = !ledState;
  return;
}


/**************************************************************************/
/*!
  * @brief  	  Color_Change  : changes the color
  * @param[in]    option : changes the color in inc/ dec order 
  * @param[out]   RGD_Color
  * @retval 	  None
  */
/**************************************************************************/
void Color_Change(char option) 
{
	static long lastTime = 0;
	Led_F=0;
	// don't change color when light is not on 
	if (!ledState) { Serial.println("Power ON LED "); return;} //  if (!ledState || stepping>0) return;
	
	if (millis() - lastTime < 10)  { Serial.println(" WAIT "); return;};
	lastTime = millis();
	
	if(option==1) {
		if (++RGD_Color == NUM_COLORS) RGD_Color = NUM_COLORS-1;
	}
	else {
		if (--RGD_Color == 0) RGD_Color = 1;
	}
	return;
}


/**************************************************************************/
/*!
  * @brief  	  Brightness_Change  : changes the Brightness of led
  * @param[in]    option :   1:Increase; 0:Decrease Brightness
  * @param[out]   brightness
  * @retval 	  None
  */
/**************************************************************************/
void Brightness_Change(char option)  
{ 
   static long last_Time = 0;
  
	// don't change color when light is not on 
	if (!ledState) { Serial.println("Power ON LED "); return;}
	
	if (millis() - last_Time < 10)  { Serial.println(" WAIT "); return;};
	last_Time = millis();
	
	if(Led_F==0) { Led_F=1; brightness=50; }
	if(option==1) {
	    brightness = brightness + fadeAmount; 	// Increase the brightness 
 	    if (brightness >= 255) brightness = 255;
	}
	else if(option==0) {
	    brightness = brightness - fadeAmount; 	// Decrease the brightness 
	    if (brightness <= 0)  brightness = 1;
	}
	 	
	// set the brightness of led
	analogWrite(RED_LED, brightness);    
	analogWrite(GREEN_LED, brightness);    
	analogWrite(BLUE_LED, brightness);    
    delay(30);    // wait for 30 milliseconds  
    return;
}


/**************************************************************************/
/*!
  * @brief  	  gpio_init  : gpio pins initialisation
  * @param[in]    None
  * @param[out]   None
  * @retval 	  None
  */
/**************************************************************************/
int gpio_init(void)
{
	while (!Serial);  
	delay(100);
	pinMode(RED_LED,   OUTPUT);
	pinMode(GREEN_LED, OUTPUT);
	pinMode(BLUE_LED,  OUTPUT);
	pinMode(RSSI_LED,  OUTPUT);
	digitalWrite(RSSI_LED, LOW);  
	pinMode(PWR_SWITCH, INPUT_PULLUP);
	attachInterrupt(0, OnOffSwitchPushed, FALLING); 
	Serial.begin(115200); 	delay(10);
	return 0;
}


/**************************************************************************/
/*!
  * @brief  	  create_LED_custom_GATT_Service : creates LED_custom  
  	@		        GATT_Service with  SWITCH & STATE charactristics
  * @param[in]    None
  * @param[out]   None
  * @retval 	  None
  */
/**************************************************************************/
int create_LED_custom_GATT_Service(void) 
{
 // Serial.println(F("\n Creating LED_custom_GATT_Service :\n"));
  ble.sendCommandCheckOK("AT+GATTCLEAR");
  
  ble.print("AT+GATTADDSERVICE=");
  ble.println(BLE_LED_GATT_SERVICE);
  if (! ble.waitForOK() )   { return 1;  } 
  
  ble.print("AT+GATTADDCHAR="); 
  ble.println(BLE_LED_SWITCH_CHARACTRISTIC);
  if (! ble.waitForOK() )   {  return 1;  }
  
  ble.print("AT+GATTADDCHAR="); 
  ble.println(BLE_LED_STATE_CHARACTRISTIC);
  if (! ble.waitForOK() )   {  return 1;  }
   
  ble.println("AT+GATTLIST"); 
  if (! ble.waitForOK() )   { return 1;  }

  ble.println("ATZ"); 
  if (! ble.waitForOK() )   {  return 1;  }

  Serial.println();
  return 0;
}


/**************************************************************************/
/*!
  * @brief  	  BLEmodule_init :  BLEmodule initialisation
  * @param[in]    None
  * @param[out]   None
  * @retval 	  None
  */
/**************************************************************************/
int BLEmodule_init(void)
{
	/* Initialise the module */
	Serial.println(F("*** Initialising BLE Module ***"));
	
	if ( !ble.begin(VERBOSE_MODE) )  {
    	error(F("Couldn't find BLE"));
    	return 0;
	}
	if ( FACTORYRESET_ENABLE ) 	{
		Serial.println(F("Factory reset:")); 
		if ( ! ble.factoryReset()) {	/* Perform a factory reset  */
			error(F("Couldn't factory reset"));
			return 0;
		}
	}
	
	/* Disable command echo from Bluefruit */
	ble.echo(false);
	
	Serial.println(F("Setting Dev Name:"));
	ble.print("AT+GAPDEVNAME=");
    ble.println(BLE_NAME); 
    if (! ble.waitForOK() )   {  
    	error(F("Didn't set the Name")); 
    }

	Serial.println("Requesting BLE info:");
	ble.info();	/* Print Bluefruit information */
	
	delay(10);
	Serial.println(F("Creating GATT Service"));
	create_LED_custom_GATT_Service();
	delay(10);  
	
	// debug info is a little annoying after this point!
	ble.verbose(false); 
	
	Serial.println( F("Waiting for connection!") );
	while (! ble.isConnected()) {  	/* Wait for connection */
		 setLedState(NULL);
		 delay(200); 
	} 
	
	// LED Activity command is only supported from 0.6.6
	if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) ) {
		Serial.println(F("Change LED activity " MODE_LED_BEHAVIOUR));  // Change Mode LED Activity
		ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
	}

	// Set module to CMD mode
	Serial.println( F("Switching to CMD mode!") );
	ble.setMode(BLUEFRUIT_MODE_COMMAND);

	Serial.println(F("*** Initialising BLE OK! ***"));
	return 0;
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
/***********************  END OF FILE  **************************************/






