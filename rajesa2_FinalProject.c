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


//---------- Preprocessor Directives & Global variables ----------

#define I2C_MCS_ACK 0x00000008 // Data Acknowledge Enable
#define I2C_MCS_DATACK 0x00000008 // Acknowledge Data
#define I2C_MCS_ADRACK 0x00000004 // Acknowledge Address
#define I2C_MCS_STOP 0x00000004 // Generate STOP
#define I2C_MCS_START 0x00000002 // Generate START
#define I2C_MCS_ERROR 0x00000002 // Error
#define I2C_MCS_RUN 0x00000001 // I2C Master Enable
#define I2C_MCS_BUSY 0x00000001 // I2C Busy
#define I2C_MCR_MFE 0x00000010 // I2C Master Function Enable

#define MAXRETRIES 5 // number of receive attempts before giving up

#define STEPS 128 // amount of steps per rotation. calculate with 360/angle
#define ROTATIONS 3 // amount of rotations

uint16_t dev = 0x29; // address of the ToF sensor as an I2C slave peripheral
uint8_t ToFMode = 1; // default mode of ToF sensor (single mode)
uint8_t dataReady; // boolean if data from ToF is ready
int status = 0;

uint16_t distances[STEPS * ROTATIONS]; // store distances as integer
char float_distances[STEPS * ROTATIONS]; // store distances as char string for UART

// motor steps in forward and back direction
int motorSteps[] = {
    0b00000011,
    0b00000110,
    0b00001100,
    0b00001001
};
int rev_motorSteps[] = {
    0b00001001,
    0b00001100,
    0b00000110,
    0b00000011
};

//---------- Inits ----------

void I2C_Init(void) {
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0; // activate I2C0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // activate port B
    while ((SYSCTL_PRGPIO_R & 0x0002) == 0) {}; // ready?

    GPIO_PORTB_AFSEL_R |= 0x0C; // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08; // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C; // 5) enable digital I/O on PB2,3
    //    GPIO_PORTB_AMSEL_R &= ~0x0C;          			
	
	// 7) disable analog functionality on PB2,3
    // 6) configure PB2,3 as I2C
    //  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00002200; //TED
    I2C0_MCR_R = I2C_MCR_MFE; // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011; // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
    //    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock

}

// Give clock to Port J and initalize PJ1 as Digital Input GPIO
void PortJ_Init(void)
{
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8; // Activate clock for Port J
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0)
    {
    };                         // Allow time for clock to stabilize
    GPIO_PORTJ_DIR_R &= ~0x02; // Make PJ1 input
    GPIO_PORTJ_DEN_R |= 0x02;  // Enable digital I/O on PJ1

    GPIO_PORTJ_PCTL_R &= ~0x000000F0; //  Configure PJ1 as GPIO
    GPIO_PORTJ_AMSEL_R &= ~0x02;      //  Disable analog functionality on PJ1
    GPIO_PORTJ_PUR_R |= 0x02;         //	Enable weak pull up resistor on PJ1
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void) {
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6; // activate clock for Port N
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R6) == 0) {}; // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00; // make PG0 in (HiZ)
    GPIO_PORTG_AFSEL_R &= ~0x01; // disable alt funct on PG0
    GPIO_PORTG_DEN_R |= 0x01; // enable digital I/O on PG0
    // configure PG0 as GPIO
    //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
    GPIO_PORTG_AMSEL_R &= ~0x01; // disable analog functionality on PN0

    return;
}

//Initialize motor port
void init_portK() {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R9; //enable clock and wait for it to load
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R9) == 0) {

    };

    //ditigal enable, set to output
    GPIO_PORTK_DIR_R = 0b00001111; // Enable PK0, PK1, PK2, and PK3 as outputs.
    GPIO_PORTK_DEN_R = 0b00001111; // Enable PK0, PK1, PK2, and PK3 as digital pins.
    return;
}

//Initialize AD2 measurement port
void init_portM() {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11; //enable clock and wait for it to load
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0) {

    };

    //ditigal enable, set to output
    GPIO_PORTM_DIR_R = 0b00000001; // Enable PM0 as output.
    GPIO_PORTM_DEN_R = 0b00000001; // Enable PM0 as digital pin.
    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void) {
    GPIO_PORTG_DIR_R |= 0x01; // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110; //PG0 = 0
    FlashLED4(1);
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01; // make PG0 input (HiZ)

}

// Function to collect data from ToF sensor
uint16_t collect_data() {
    uint16_t Distance = 0;
    uint8_t dataReady = 0;
		uint8_t RangeStatus = 0;

    // wait for data to be ready, flashing LED4 if not ready
    while (dataReady == 0) {
        status = VL53L1X_StartRanging(dev);
        status = VL53L1X_CheckForDataReady(dev, & dataReady);

        VL53L1_WaitMs(dev, 2);
        VL53L1X_StopRanging(dev);
        status = VL53L1X_ClearInterrupt(dev);

    }
    dataReady = 0;

		status = VL53L1X_GetRangeStatus(dev, &RangeStatus); // make sure status ok
    status = VL53L1X_GetDistance(dev, & Distance); // Get distance measurement 

    status = VL53L1X_ClearInterrupt(dev);

    return Distance;

}

// Function to spin motor
void spin_motor(uint16_t * distances) {
    int counter = 0;
    int direction = 0;

    for (int i = 0; i < ROTATIONS; i++) { // how many rotations to spin for
        for (int x = 0; x < STEPS; x++) { //how many steps per rotation

            for (int j = 0; j < (2048 / (STEPS * 4)); j++) {
                for (int k = 0; k < 4; k++) {

                    if (direction) { //forward rotation
                        GPIO_PORTK_DATA_R = rev_motorSteps[k];
                    } else { //backwards rotation
                        GPIO_PORTK_DATA_R = motorSteps[k];
                    }

                    SysTick_Wait(40000);
                }
            }

            distances[counter] = collect_data(); //collect data and add it to distances array
            FlashLED3(1); //flash D3 for every measurement taken
            SysTick_Wait(40000);
            counter++;

        }

        // Returns to home after every rotation
        for (int x = 0; x < STEPS; x++) { //take this many measurements per rotation

            for (int j = 0; j < (2048 / (STEPS * 4)); j++) {
                for (int k = 0; k < 4; k++) {

                    if (!direction) {
                        GPIO_PORTK_DATA_R = rev_motorSteps[k];
                    } else {
                        GPIO_PORTK_DATA_R = motorSteps[k];
                    }

                    SysTick_Wait(40000);
                }
            }

            SysTick_Wait(40000);

        }

		//flash led 4 to incicate user to go to take a step
				for(int i=0;i<5;i++){
					FlashLED4(1);
					SysTick_Wait10ms(10);
				}
        
    }
}

void measure_bus(){
    while(1){
        GPIO_PORTM_DATA_R ^= 1;
        SysTick_Wait10ms(1);
    }
}

//---------- Main Function ----------

int main(void) {

    uint8_t dataReady;
    uint8_t sensorState = 0;

    int start = 0; //boolean to start measurements
    int start_pressed = 0;

    //initialize functions
    PLL_Init();
    SysTick_Init();
    onboardLEDs_Init();
    I2C_Init();
    UART_Init();
    init_portK();
    PortJ_Init();
    init_portM();

    //Uncomment this function to measure BUS speed with AD2 or any other oscilloscope.
    //Note: Rest of program will not run if uncommented.
	  //measure_bus();

    // Wait for ToF Sensor to turn on
    while (sensorState == 0) {
        status = VL53L1X_BootState(dev, & sensorState);
        SysTick_Wait10ms(10);
    }
    UART_printf("ToF Initialized.\r\n");

    status = VL53L1X_ClearInterrupt(dev);

    //Initialize sensor with predefined mode
    status = VL53L1X_SensorInit(dev);
    Status_Check("SensorInit", status);

    //Start ranging of ToF sensor
    status = VL53L1X_StartRanging(dev);
    Status_Check("StartRanging", status);

    //Check if letter 's' was recieved from PC to start measurements
    while (1) {
        start = UART_InChar();

        if (start == 115) {
            FlashLED4(1);
            start = 0;
            break;
        }
    }

    while(1) {
        start_pressed = GPIO_PORTJ_DATA_R;

        if(!start_pressed){
            break;
        }

    }

    spin_motor(distances); //spin motor and measure data

    //convert distances to string and transfer to Python via UART
    for (int i = 0; i < (int)(ROTATIONS * (STEPS)); i++) {
        sprintf(float_distances, "%u\r\n", distances[i]);
        UART_printf(float_distances);
    }

    //print 'x' to let Python know all data has been transferred
    UART_printf("x\r\n");

    //turn off ToF sensor ranging
    VL53L1X_StopRanging(dev);

}