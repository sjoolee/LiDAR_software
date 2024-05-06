#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"
#include <math.h>

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

uint16_t	dev = 0x29;
int status=0;

uint8_t byteData, sensorState=0, ByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
uint16_t AmbientRate;
uint16_t SignalRate;
uint8_t dataReady;
uint16_t SpadNum; 
uint16_t Distance;
uint16_t wordData;
uint8_t RangeStatus;


int steps[] = {0b1100, 0b0110, 0b0011, 0b1001};
float curr_angle = 0;
int program_stat = -1; //waiting to be armed //-1 means it's measuring and talking to the PC

float deg_per_measure = 5.625; //2.8125*2=5.625
int measurement_data[256]; //128*2 = 256
int sample_num = 2; //10 divided by 5

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;   //Enables I2C0					
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;  //Enable GPIO Port B				
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};	//Wait for GPIO Port B to be ready									
    GPIO_PORTB_ODR_R |= 0x08;  //Set PB3 as output
    GPIO_PORTB_DEN_R |= 0x0C;  //Enable digital function for PB2 and PB3										
		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) | 0x00002200; //Configure PB2 and PB3 for I2C    
    I2C0_MCR_R = I2C_MCR_MFE;                      											
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       
}

void PortG_Init(void){
    //We are using PortG0 since VL53L1X needs to be reset using XSHUT
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6; //enables the clock for GPIO Port G
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){}; //waits for Port G to be ready
    GPIO_PORTG_DIR_R &= 0x00; 
		GPIO_PORTG_DEN_R |= 0x01; 			
		GPIO_PORTG_AFSEL_R &= ~0x01; //disables alternate function                                                                    
		GPIO_PORTG_AMSEL_R &= ~0x01; //disables analog functionality
    return;
}

void PortH_Init(void){	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};
	GPIO_PORTH_DIR_R = 0xFF;
	GPIO_PORTH_DEN_R = 0xFF;                        		
	return;
}

void PortM_Init(void){	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};
	GPIO_PORTM_DIR_R = 0b1;
	GPIO_PORTM_DEN_R = 0b1;                        		
	return;
}


void PortL_Init(void){	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};
	GPIO_PORTL_DIR_R = 0b00;
	GPIO_PORTL_DEN_R = 0b11;                        		
	return;
}

void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        
    GPIO_PORTG_DATA_R &= 0b11111110;                                 
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            
}

void updateAngle(int direction){
	curr_angle =(curr_angle+direction)*0.703125;
	if(curr_angle < 0){
		curr_angle += 360;
	}
	else if(curr_angle == 360){
		curr_angle = 0;
	}
}

static inline void setStepAndWait(int step, int waitTime) {
    GPIO_PORTH_DATA_R = steps[step];
    SysTick_Wait(waitTime);
}

void rotate(int direction, float num_step) {
    int numFullSteps = (2048/360) * num_step;
    int stepsPerIteration = numFullSteps/4;
    int waitTime = direction == 1 ? 192000 : 60000; // 2ms for direction 1, 30ms for direction -1
    for (int j = 0; j < stepsPerIteration; j++) {
        if (direction == 1) {
            for (int i = 0; i < 4; i++) {
                setStepAndWait(i, waitTime); }
        } 
				else if (direction == -1) {
            for (int i = 3; i >= 0; i--) {
                setStepAndWait(i, waitTime);  }
        }
        updateAngle(direction); }
}

void return_home(void){
	rotate(-1,curr_angle);
}

// Function to get distance and clear interrupt
int update_status(void) {
    int status = VL53L1X_GetDistance(dev, &Distance);
    status = VL53L1X_ClearInterrupt(dev);
    return status;
}

int measureYZ(void){
	uint32_t total = 0; //reset total for each measurement
	for(int i=0;i<32;i++) {
		int total = 0;
		for(int j=0; j<sample_num; j++){
			status = VL53L1X_GetDistance(dev, &Distance);
			total = total + Distance;
			status = VL53L1X_ClearInterrupt(dev);
			FlashLED4(1); //Blinky Light for Port
		}
		measurement_data[i] = total/sample_num;
		rotate(1,11.25);
		while((GPIO_PORTL_DATA_R&0b11) == 0b01){
			for(int delay = 0; delay < 1000; delay++) {} // 1ms delay
		}
		if((GPIO_PORTL_DATA_R&0b11) == 0b01){
				return_home();
				return -1;
			}
	}
	rotate(-1,360);
	return 0;
}

void bootToFChip() {
    int status;
    while (sensorState == 0) {
        status = VL53L1X_BootState(dev, &sensorState);
        SysTick_Wait10ms(10);
    }
    UART_printf("Please Wait ToF Booting!\r\n");
}

void initialize_sensor(){
	int status;
	status = VL53L1X_ClearInterrupt(dev); 
  status = VL53L1X_SensorInit(dev);
	
	Status_Check("SensorInit", status);
	
  status = VL53L1X_SetDistanceMode(dev, 2); 
  status = VL53L1X_SetTimingBudgetInMs(dev, 100); 
  status = VL53L1X_SetInterMeasurementInMs(dev, 200); 
	status = VL53L1X_StartRanging(dev);  
	UART_printf("Press Button 1 to start/stop measurement process\r\n");
	UART_printf("Press Button 2 to measure plane\r\n");
}


int main(void) {
	//initialize
	PortH_Init();
	PortM_Init();
	PortL_Init();
	I2C_Init();
	UART_Init();
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	
	GPIO_PORTN_DATA_R = 0b00; //Initalize Port 
	
	bootToFChip(); // Booting ToF chip
	
	initialize_sensor();	
	while(1){
		while(1){
			if((GPIO_PORTL_DATA_R&0b11) == 0b10){
				while((GPIO_PORTL_DATA_R&0b11) == 0b10){}
				program_stat *= -1;
				if(program_stat == 1){
					UART_printf("Starting to measure\r\n");
					sprintf(printf_buffer, "%f\r\n", deg_per_measure);
					UART_printf(printf_buffer);
					GPIO_PORTN_DATA_R ^= 0b01; //ON LED if not it will be 0b10
				}else if(program_stat == -1){
					UART_printf("Ending measurement process\r\n");
					GPIO_PORTN_DATA_R ^= 0b01;
					break; }
			}
			else if((GPIO_PORTL_DATA_R&0b11) == 0b01){
				while((GPIO_PORTL_DATA_R&0b11) == 0b01){}
				if((measureYZ() == 0) && (program_stat == 1)){
					for(int i=0; i<(360.0/deg_per_measure); i++){
						sprintf(printf_buffer,"%u ", measurement_data[i]);
						UART_printf(printf_buffer);
					}
					UART_printf("\r\n"); }
			}
			GPIO_PORTM_DATA_R ^= 0b1;
			SysTick_Wait(3000); }
		UART_printf("3D Map Developing\r\n"); }
	VL53L1X_StopRanging(dev);
}