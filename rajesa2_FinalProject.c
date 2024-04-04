/*  

	Written by Aman Rajesh
	Final Project code for COMPENG 2DX3
	
*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"





#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

//---------- Inits ----------

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

//Initialize motor port
void init_portK(){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R9; //enable clock and wait for it to load
	while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R9) == 0)
    {

    };

	//ditigal enable, set to output
	GPIO_PORTK_DIR_R = 0b00001111; // Enable PL0, PL1, PL2, and PL3 as outputs.
    GPIO_PORTK_DEN_R = 0b00001111; // Enable PL0, PL1, PL2 and PL3 as digital pins.
    return;
}



//---------- Global variables ----------

#define STEPS 32
#define ROTATIONS 2

uint16_t distances[STEPS*ROTATIONS];
char float_distances[STEPS*ROTATIONS];

int motorSteps[] = {0b00000011, 0b00000110, 0b00001100, 0b00001001};
int rev_motorSteps[] = {0b00001001, 0b00001100, 0b00000110, 0b00000011};
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
uint8_t ToFMode= 1;
uint8_t dataReady;
int status = 0;

uint16_t collect_data(){
	uint16_t Distance = 0;
	uint8_t xx = 0;
	
	//Wait for ToF sensor to be ready
	
	while (xx == 0){
		status = VL53L1X_StartRanging(dev);
		status = VL53L1X_CheckForDataReady(dev, &xx);

          FlashLED1(1);
          SysTick_Wait10ms(1);
		  VL53L1X_StopRanging(dev);
		  status = VL53L1X_ClearInterrupt(dev);

	  }
		xx = 0;
		
		status = VL53L1X_GetDistance(dev, &Distance);	//getc distance measurement

		status = VL53L1X_ClearInterrupt(dev);
		
		return Distance;
	
	
}

void spin_motor(uint16_t* distances){
	int counter = 0;
	int direction = 0;
	for(int i=0;i<ROTATIONS;i++){
				for(int x=0;x<STEPS;x++){ //take this many measurements per rotation
				
				for(int j = 0;j<(2048/(STEPS*4));j++){
					for(int k=0;k<4;k++){
						if(direction){
							GPIO_PORTK_DATA_R = rev_motorSteps[k];
						} else{
							GPIO_PORTK_DATA_R = motorSteps[k];
						}
						
						SysTick_Wait(200000);
					}
				}
				
				distances[counter] = collect_data();
				FlashLED3(1); //flash D3 for every measurement taken
				SysTick_Wait(200000);
				counter++;
				
			}

			//return to home
			for(int x=0;x<STEPS;x++){ //take this many measurements per rotation
				
				for(int j = 0;j<(2048/(STEPS*4));j++){
					for(int k=0;k<4;k++){
						if(!direction){
							GPIO_PORTK_DATA_R = rev_motorSteps[k];
						} else{
							GPIO_PORTK_DATA_R = motorSteps[k];
						}
						
						SysTick_Wait(200000);
					}
				}
				
				SysTick_Wait(200000);

			}
				
			direction ^= 1;
			FlashAllLEDs();
			SysTick_Wait(1000000);
			FlashAllLEDs();
			SysTick_Wait(1000000);
			FlashAllLEDs();
			SysTick_Wait(1000000);
			
		
	}
}



	



//---------- Main Function ----------

int main(void) {
  uint16_t wordData;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
  uint8_t sensorState = 0;
	
	int start;
	
		

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	init_portK();
	

	// Wait for ToF Sensor to turn on
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	UART_printf("ToF Initialized.\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

  //Initialize sensor with predefined mode
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

  //Start ranging of ToF sensor
  status = VL53L1X_StartRanging(dev);
  Status_Check("StartRanging", status);
	
	while(1){
			start = UART_InChar(); // wait for acknowledgement from PC
			if (start == 115)
			{
				FlashLED1(1);
				start = 0;
				break;
			}
		}
	
	
	spin_motor(distances);
		
	
	for(int i=0;i< (int)(ROTATIONS * (STEPS));i++){
		sprintf(float_distances, "%u\r\n", distances[i]);
		UART_printf(float_distances);
	}
  
	UART_printf("x\r\n");

  
	VL53L1X_StopRanging(dev);

}