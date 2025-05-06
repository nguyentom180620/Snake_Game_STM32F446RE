/*
 * Name: Bare_Metal_Snake_Game
 * Purpose: To develop a game using the STM32F446RE and the MAX7219 w/ LED matrix display
 * Author: Tom Nguyen
 * Date: 5/4/2025
 */

#include <time.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define RCC 			0x40023800
#define RCC_CR			(RCC + 0x00)
#define RCC_CFGR		(RCC + 0x08)
#define RCC_AHB1ENR		(RCC + 0x30)
#define RCC_APB1		(RCC + 0x40)
#define RCC_APB2ENR		(RCC + 0x44)
#define FLASH			0x40023C00
#define FLASH_ACR		(FLASH + 0x00)
#define TIM3			0x40000400
#define TIM3_CR1		(TIM3 + 0x00)
#define TIM3_DIER		(TIM3 + 0x0C)
#define TIM3_SR			(TIM3 + 0x10)
#define TIM3_CNT		(TIM3 + 0x24)
#define TIM3_PSC		(TIM3 + 0x28)
#define TIM3_ARR		(TIM3 + 0x2C)
#define SPI1			0x40013000
#define SPI1_CR1		(SPI1 + 0x00)
#define SPI1_CR2		(SPI1 + 0x04)
#define SPI1_SR			(SPI1 + 0x08)
#define SPI1_DR			(SPI1 + 0x0C)
#define GPIOA			0x40020000
#define GPIOA_MODER		(GPIOA + 0x00)
#define GPIOA_AFRL		(GPIOA + 0x20)
#define CS_Port			GPIOA
#define CS_Pin			4
#define CS_BSRR			(CS_Port + 0x18)
#define GPIOC			0x40020800
#define GPIOC_MODER		(GPIOC + 0x00)
#define SYSCFG			0x40013800
#define SYSCFG_EXTICR1	(SYSCFG + 0x08)
#define SYSCFG_EXTICR4	(SYSCFG + 0x14)
#define EXTI			0x40013C00
#define EXTI_IMR		(EXTI + 0x00)
#define EXTI_FTSR		(EXTI + 0x0C)
#define EXTI_PR			(EXTI + 0x14)
#define EXTI0IRQn		6
#define EXTI1IRQn		7
#define EXTI2IRQn		8
#define EXTI3IRQn		9
#define EXTI15_10IRQn	40
#define NVIC_ISER		0xE000E100
#define UP				0
#define RIGHT			1
#define DOWN			2
#define LEFT			3

static void SetSystemClockto16MHz(void);
static void ConfigureTimer3(void);
static void RandomSeedLoader(void);
static void Delay(uint32_t ms);
static void SPI1ClockEnable(void);
static void GPIOAClockEnable(void);
static void GPIOCClockEnable(void);
static void SPI1Init(void);
static void SPI1WriteToDR(uint16_t data);
static void WaitForTransmissionEnd(void);
static void EnableSlave(void);
static void DisableSlave(void);
static void SPI1_Transmit(uint16_t data);
static void SPI1PinsInit(void);
static void max7219_write(uint8_t addr, uint8_t data);
static void matrixClear(void);
static void matrixInit(void);
static uint8_t intToHexPosition(uint8_t val);
static void positionToMatrixPos(uint8_t x_pos[], uint8_t y_pos[], int numberOfCords, uint8_t outputArray[]);
static void LEDMatrixWrite(uint8_t outputArray[]);
static void LEDMatrixRowWrite(uint8_t outputArray[], uint8_t row);
static void LEDMatrixColumnWrite(uint8_t outputArray[], uint8_t col);
static void ResetButtonInit(void);
static void MovementButtonsInit(void);
void NVIC_EnableIRQ(uint32_t IRQn);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void playLoseScreen(void);

static volatile int snake_direction = RIGHT;
static volatile int previous_direction = RIGHT;
static volatile bool alive = true;
static volatile bool reset = false;

typedef struct {
	volatile uint32_t ISER[3];
} NVIC_Type;

#define NVIC ((NVIC_Type*)NVIC_ISER)

typedef struct {
	volatile int snakeSize;
	uint8_t x_pos[64];
	uint8_t y_pos[64];
	uint8_t outputArray[9];
} snake_Type;

static void DisplaySnake(snake_Type *snake);
static void SnakeInit(snake_Type *snake);
static void SnakeDead(void);
static void SnakeCheckAfterMove(snake_Type *snake);
static void ClearOutputArray(snake_Type *snake);
static void SnakeGrow(snake_Type *snake);

typedef struct {
	uint8_t x_pos[1];
	uint8_t y_pos[1];
} apple_Type;

static void AppleInit(apple_Type *apple);
static void DisplayApple(snake_Type *snake, apple_Type *apple);
static bool CheckIfAppleCollected(uint8_t snakeHeadx, uint8_t snakeHeady, apple_Type *apple);
static void ReplaceApple(snake_Type *snake, apple_Type *apple);

static void DisplayGame(snake_Type *snake, apple_Type *apple);
static void ResetGame(snake_Type *snake, apple_Type *apple);
static void MoveSnake(snake_Type *snake, apple_Type *apple);

int main(void)
{
	SetSystemClockto16MHz();
	ConfigureTimer3();
	SPI1ClockEnable();
	GPIOAClockEnable();
	GPIOCClockEnable();

	ResetButtonInit();
	MovementButtonsInit();

	NVIC_EnableIRQ(EXTI0IRQn);
	NVIC_EnableIRQ(EXTI1IRQn);
	NVIC_EnableIRQ(EXTI2IRQn);
	NVIC_EnableIRQ(EXTI3IRQn);
	NVIC_EnableIRQ(EXTI15_10IRQn);

	SPI1PinsInit();
	SPI1Init();

	matrixInit();
	RandomSeedLoader();

	// Write data here
	// Snake starts with head at (3, 2), tail at (1, 2), and size 3
	snake_Type snake;
	snake_Type *snake_Ptr = &snake;
	SnakeInit(snake_Ptr);

	// Apple starts at (3, 7) and will update to a random square after collected
	apple_Type apple;
	apple_Type *apple_Ptr = &apple;
	AppleInit(apple_Ptr);

	while(1)
	{
		// If reset, restart snake and apple values
		if (reset == true)
		{
			ResetGame(snake_Ptr, apple_Ptr);
		}
		// If dead, play the dead sequence and queue for restart
		if (alive == false)
		{
			playLoseScreen();
		}

		// First, display to screen
		DisplayGame(snake_Ptr, apple_Ptr);

		// Next, delay by set amount (default 1 second)
		Delay(250);

		// Then, move the snake by one
		MoveSnake(snake_Ptr, apple_Ptr);

		// Finally, check if head is currently at an apple (then set that bool)
		// and lastly check if we are alive (Are we out of bounds or have we hit ourself)
		SnakeCheckAfterMove(snake_Ptr);
	}

	return EXIT_SUCCESS;
}

void SetSystemClockto16MHz(void)
{
	// Initialize System Clock
	uint32_t *RCC_CR_Ptr = (uint32_t*)RCC_CR;
	// Turn on HSI
	*RCC_CR_Ptr |= (uint32_t)0x1;
	// Wait for HSI Clock to be ready
	while ((*RCC_CR_Ptr & 0x2) == 0);

	// Configure Prescalers
	uint32_t *RCC_CFGR_Ptr = (uint32_t*)RCC_CFGR;
	// HPRE
	*RCC_CFGR_Ptr &= ~(uint32_t)(0b1111 << 4);
	// PPRE1
	*RCC_CFGR_Ptr &= ~(uint32_t)(0b111 << 10);
	// PPRE2
	*RCC_CFGR_Ptr &= ~(uint32_t)(0b111 << 13);

	// Set HSI as Clock Source
	*RCC_CFGR_Ptr &= ~(uint32_t)(0b11);

	// Configure Flash
	uint32_t *FLASH_ACR_Ptr = (uint32_t*)FLASH_ACR;
	// Latency
	*FLASH_ACR_Ptr |= (uint32_t)(0b0000 << 0);
	// ICEN
	*FLASH_ACR_Ptr |= (uint32_t)(0b1 << 9);
	// DCEN
	*FLASH_ACR_Ptr |= (uint32_t)(0b1 << 10);

	// Turn off HSE
	*RCC_CR_Ptr &= ~((uint32_t)0x1 << 16);
}

void ConfigureTimer3(void)
{
	// Enable TIM3 Clock
	uint32_t *RCC_APB1_Ptr = (uint32_t*)RCC_APB1;
	*RCC_APB1_Ptr |= (uint32_t)0x2;

	// Set Prescaler
	uint32_t *TIM3_PSC_Ptr = (uint32_t*)TIM3_PSC;
	*TIM3_PSC_Ptr |= (uint32_t)0xF;

	// Set Auto-Reload
	uint32_t *TIM3_ARR_Ptr = (uint32_t*)TIM3_ARR;
	*TIM3_ARR_Ptr = (uint32_t)0x3E7;

//	// Enable Interrupt
//	uint32_t *TIM3_DIER_Ptr = (uint32_t*)TIM3_DIER;
//	*TIM3_DIER_Ptr |= (uint32_t)0x1;

	// Clear UIF Bit
	uint32_t *TIM3_SR_Ptr = (uint32_t*)TIM3_SR;
	*TIM3_SR_Ptr &= (uint32_t)0xFFFE;

//	// Enable NVIC Interrupt for Timer 3
//	NVIC_EnableIRQ(TIM3_IRQn);

	// Enable TIM3
	uint32_t *TIM3_CR1_Ptr = (uint32_t*)TIM3_CR1;
	*TIM3_CR1_Ptr = (uint32_t)0b1 << 0;
}

void RandomSeedLoader(void)
{
	// For random seeding upon reset
	volatile uint32_t *TIM3_CNT_Ptr = (volatile uint32_t*)TIM3_CNT;
	uint32_t timer3Val = *TIM3_CNT_Ptr;
	srand((uint32_t)time(NULL) ^ (uint32_t)timer3Val);
}

void Delay(uint32_t ms)
{
	volatile uint32_t i;
	uint32_t *TIM3_CNT_Ptr = (uint32_t*)TIM3_CNT;
	uint32_t *TIM3_SR_Ptr = (uint32_t*)TIM3_SR;
	for (i = 0; i <= ms; i++)
	{
		// Check if reset is true (this is where most polling happens)
		if (reset) return;

		// Clear TIM3 Count
		*TIM3_CNT_Ptr = 0;

		// Wait for UIF (1 cycle of 1kHz clocking)
		while((*TIM3_SR_Ptr & 0x1) == 0);	// This will make a 1ms delay

		// Reset UIF
		*TIM3_SR_Ptr &= (uint32_t)0xFFFE;
	}
}

void SPI1ClockEnable(void)
{
	// First, SPI clock through APB2 Bus
	uint32_t *RCC_APB2ENR_Ptr = (uint32_t*)RCC_APB2ENR;
	*RCC_APB2ENR_Ptr |= (uint32_t)(0x1 << 12);
}

void GPIOAClockEnable(void)
{
	// Now, Enable GPIOA Clock through AHB1 Bus
	uint32_t *RCC_AHB1ENR_Ptr = (uint32_t*)RCC_AHB1ENR;
	*RCC_AHB1ENR_Ptr |= (uint32_t)0x1;
}

void GPIOCClockEnable(void)
{
	// Now, Enable GPIOC Clock through AHB1 Bus
	uint32_t *RCC_AHB1ENR_Ptr = (uint32_t*)RCC_AHB1ENR;
	*RCC_AHB1ENR_Ptr |= (uint32_t)0b1 << 2;
}

void SPI1Init(void)
{
	// Set Up SPI Init
	uint32_t *SPI1_CR1_Ptr = (uint32_t*)SPI1_CR1;

	// NOTE: Simplex is basically just full duplex but we don't use MISO

	// BIDIMODE off
	*SPI1_CR1_Ptr &= ~(uint32_t)(0x1 << 15);
	// CRC Calculations off
	*SPI1_CR1_Ptr &= ~(uint32_t)(0x1 << 13);
	// DFF to 16 bits
	*SPI1_CR1_Ptr |= (uint32_t)(0x1 << 11);
	// RXOnly off since we are transferring from master to slave
	*SPI1_CR1_Ptr &= ~(uint32_t)(0x1 << 10);
	// SSM Disabled
	// MSB Selected
	*SPI1_CR1_Ptr &= ~(uint32_t)(0x1 << 7);
	// Baud Rate of 2 MBits/S
	*SPI1_CR1_Ptr &= ~(uint32_t)(0b111 << 3);
	*SPI1_CR1_Ptr |= (uint32_t)(0b010 << 3);
	// Put into Master Mode
	*SPI1_CR1_Ptr |= (uint32_t)(0x1 << 2);
	// Set CPOL and CPHA
	*SPI1_CR1_Ptr &= ~(uint32_t)(0x3);

	// SSOE enabled
	uint32_t *SPI1_CR2_Ptr = (uint32_t*)SPI1_CR2;
	*SPI1_CR2_Ptr |= 0x4;

	// Finally, enable SPI
	*SPI1_CR1_Ptr |= (uint32_t)(0x1 << 6);
}

void SPI1WriteToDR(uint16_t data)
{
	// Load data into SPI1 data register
	uint32_t *SPI1_DR_Ptr = (uint32_t*)SPI1_DR;
	*SPI1_DR_Ptr = (uint32_t)data;
}

void WaitForTransmissionEnd(void)
{
	// Wait for transmission to end by checking BSY and TXE
	uint32_t *SPI1_SR_Ptr = (uint32_t*)SPI1_SR;
	while ((*SPI1_SR_Ptr & (0b1 << 7)) != 0);
	while ((*SPI1_SR_Ptr & (0b1 << 1)) == 0);
}

void EnableSlave(void)
{
	// Enable Slave
	uint32_t *CS_BSRR_Ptr = (uint32_t*)CS_BSRR;
	*CS_BSRR_Ptr |= (uint32_t)(0b1 << (CS_Pin + 16));
}

void DisableSlave(void)
{
	// Disable Slave
	uint32_t *CS_BSRR_Ptr = (uint32_t*)CS_BSRR;
	*CS_BSRR_Ptr |= (uint32_t)(0b1 << CS_Pin);
}

void SPI1_Transmit(uint16_t data)
{
	EnableSlave();
	SPI1WriteToDR(data);
	WaitForTransmissionEnd();
	DisableSlave();
}

void SPI1PinsInit(void)
{
	// Initialize SPI GPIO Pins
	// First, PinA5 for SCLK
	uint32_t *GPIOA_MODER_Ptr = (uint32_t*)GPIOA_MODER;
	// Set to Alternate Function
	*GPIOA_MODER_Ptr &= ~(uint32_t)(0b11 << 10);
	*GPIOA_MODER_Ptr |= (uint32_t)(0b10 << 10);
	// Next, PinA7 for MOSI
	*GPIOA_MODER_Ptr &= ~(uint32_t)(0b11 << 14);
	*GPIOA_MODER_Ptr |= (uint32_t)(0b10 << 14);

	// Set a GPIO Pin for CS Pin
	uint32_t *CS_Port_Ptr = (uint32_t*)CS_Port;
	// Set Pin 4 to output
	*CS_Port_Ptr &= ~(uint32_t)(0b11 << 2 * CS_Pin);
	*CS_Port_Ptr |= (uint32_t)(0b01 << 2 * CS_Pin);

	// Set up alternate function by selecting AF5 (According to datasheet)
	uint32_t *GPIOA_AFRL_Ptr = (uint32_t*)GPIOA_AFRL;
	*GPIOA_AFRL_Ptr |= (uint32_t)(0b0101 << 16);
	*GPIOA_AFRL_Ptr |= (uint32_t)(0b0101 << 20);
	*GPIOA_AFRL_Ptr |= (uint32_t)(0b0101 << 28);
	// Initialize to High
	DisableSlave();
}

void max7219_write(uint8_t addr, uint8_t data)
{
	uint16_t writeData = (addr << 8) | data;
	SPI1_Transmit(writeData);
}

void matrixClear(void)
{
	for (int i = 1; i <= 8; i++)
	{
		max7219_write(i, 0x00);	// Clear Screen
	}
}

void matrixInit(void)
{
	max7219_write(0x09, 0);		// No Decoding
	max7219_write(0x0A, 0x02);	// 5/32 Light Intensity
	max7219_write(0x0B, 0x07);	// Scan all columns (Turn them all on)
	max7219_write(0x0C, 0x01);	// Normal Operation (No shutdown mode)
	max7219_write(0x0F, 0x00);	// No Display Test

	matrixClear();
}

uint8_t intToHexPosition(uint8_t val)
{
	switch (val)
	{
		case 1:
			return 0x01;
			break;
		case 2:
			return 0x02;
			break;
		case 3:
			return 0x04;
			break;
		case 4:
			return 0x08;
			break;
		case 5:
			return 0x10;
			break;
		case 6:
			return 0x20;
			break;
		case 7:
			return 0x40;
			break;
		case 8:
			return 0x80;
			break;
		default:
			return -1;		// Should never get this, only enter values between 1 and 8
	}
}

void positionToMatrixPos(uint8_t x_pos[], uint8_t y_pos[], int numberOfCords, uint8_t outputArray[])
{
	for (int i = 0; i < numberOfCords; i++)
	{
		uint8_t x = x_pos[i];
		uint8_t y = y_pos[i];
		x = intToHexPosition(x);
		outputArray[y] |= x;
	}
}

void LEDMatrixWrite(uint8_t outputArray[])
{
	uint16_t writePos;
	for (int i = 1; i <= 8; i++)
	{
		writePos = (i << 8) | outputArray[i];
		SPI1_Transmit(writePos);
	}
}

void LEDMatrixRowWrite(uint8_t outputArray[], uint8_t row)
{
	uint16_t writePos;
	writePos = (row << 8) | outputArray[row];
	SPI1_Transmit(writePos);
}

void LEDMatrixColumnWrite(uint8_t outputArray[], uint8_t col)
{
	// We want to write from col 1 to this col variable (inclusive)
	for (int i = 1; i <= 8; i++)
	{
		uint8_t row_val = outputArray[i];
		for (int j = 8; j > col; j--)
		{
			row_val &= ~intToHexPosition(j);
		}
		uint16_t writeRow = (i << 8) | row_val;
		SPI1_Transmit(writeRow);
	}
}

void ResetButtonInit(void)
{
	// Sets button as inputs
	uint32_t *GPIOC_MODER_Ptr = (uint32_t*)GPIOC_MODER;
	// Reset Button PC13 (User button)
	*GPIOC_MODER_Ptr &= ~((uint32_t)0b11 << (13 * 2));

	// Falling Edge interrupt
	// SYSCLK Enabled
	uint32_t *RCC_APB2ENR_Ptr = (uint32_t*)RCC_APB2ENR;
	*RCC_APB2ENR_Ptr |= (uint32_t)0b1 << 14;

	// Configure EXTI13 for PC13
	uint32_t *SYSCFG_EXTICR4_Ptr = (uint32_t*)SYSCFG_EXTICR4;
	*SYSCFG_EXTICR4_Ptr |= (uint32_t)0b0010 << 4;

	// Enable falling trigger mode
	uint32_t *EXTI_FTSR_Ptr = (uint32_t*)EXTI_FTSR;
	*EXTI_FTSR_Ptr |= (uint32_t)0b1 << 13;

	// Unmask interrupt
	uint32_t *EXTI_IMR_Ptr = (uint32_t*)EXTI_IMR;
	*EXTI_IMR_Ptr |= (uint32_t)0b1 << 13;
}

void MovementButtonsInit(void)
{
	// Sets buttons as inputs
	uint32_t *GPIOC_MODER_Ptr = (uint32_t*)GPIOC_MODER;
	// Up Pin PC0
	*GPIOC_MODER_Ptr &= ~((uint32_t)0b11 << (0 * 2));
	// Right Pin PC1
	*GPIOC_MODER_Ptr &= ~((uint32_t)0b11 << (1 * 2));
	// Down Pin PC2
	*GPIOC_MODER_Ptr &= ~((uint32_t)0b11 << (2 * 2));
	// Left Pin PC3
	*GPIOC_MODER_Ptr &= ~((uint32_t)0b11 << (3 * 2));

	// Now set up falling edge interrupt for buttons
	// Enable SYSCFG Clock
	uint32_t *RCC_APB2ENR_Ptr = (uint32_t*)RCC_APB2ENR;
	*RCC_APB2ENR_Ptr |= (uint32_t)0b1 << 14;

	// Configure EXTI0 to EXTI3 for PC0-PC3
	uint32_t *SYSCFG_EXTICR1_Ptr = (uint32_t*)SYSCFG_EXTICR1;
	for (int i = 0; i < 4; i++)
	{
		*SYSCFG_EXTICR1_Ptr |= (uint32_t)0b0010 << (i * 4);
	}

	// Enable falling trigger
	uint32_t *EXTI_FTSR_Ptr = (uint32_t*)EXTI_FTSR;
	for (int i = 0; i < 4; i++)
	{
		*EXTI_FTSR_Ptr |= (uint32_t)0b1 << i;
	}

	// Unmask the interrupt
	uint32_t *EXTI_IMR_Ptr = (uint32_t*)EXTI_IMR;
	for (int i = 0; i < 4; i++)
	{
		*EXTI_IMR_Ptr |= (uint32_t)0b1 << i;
	}
}

void NVIC_EnableIRQ(uint32_t IRQn)
{
	if (IRQn <= 96)
	{
		uint32_t iserIndex = IRQn >> 5;
		uint32_t iserBit = IRQn & 0x1F;

		NVIC->ISER[iserIndex] |= (0x1 << iserBit);
	}
}

void EXTI0_IRQHandler(void)
{
	uint32_t *EXTI_PR_Ptr = (uint32_t*)EXTI_PR;

	if (previous_direction != DOWN)
	{
		snake_direction = UP;
	}

	// Clear PinC0 interrupt Bit
	*EXTI_PR_Ptr = (uint32_t)0b1 << 0;
}

void EXTI1_IRQHandler(void)
{
	uint32_t *EXTI_PR_Ptr = (uint32_t*)EXTI_PR;

	if (previous_direction != LEFT)
	{
		snake_direction = RIGHT;
	}

	// Clear PinC1 interrupt Bit
	*EXTI_PR_Ptr = (uint32_t)0b1 << 1;
}

void EXTI2_IRQHandler(void)
{
	uint32_t *EXTI_PR_Ptr = (uint32_t*)EXTI_PR;

	if (previous_direction != UP)
	{
		snake_direction = DOWN;
	}

	// Clear PinC2 interrupt Bit
	*EXTI_PR_Ptr = (uint32_t)0b1 << 2;
}

void EXTI3_IRQHandler(void)
{
	uint32_t *EXTI_PR_Ptr = (uint32_t*)EXTI_PR;

	if (previous_direction != RIGHT)
	{
		snake_direction = LEFT;
	}

	// Clear PinC3 interrupt Bit
	*EXTI_PR_Ptr = (uint32_t)0b1 << 3;
}

void EXTI15_10_IRQHandler(void)
{
	uint32_t *EXTI_PR_Ptr = (uint32_t*)EXTI_PR;

	reset = true;

	// Clear PinC13 interrupt Bit
	*EXTI_PR_Ptr = (uint32_t)0b1 << 13;
}

void DisplaySnake(snake_Type *snake)
{
	positionToMatrixPos(snake->x_pos, snake->y_pos, snake->snakeSize, snake->outputArray);
}

void SnakeInit(snake_Type *snake)
{
	snake->snakeSize = 3;
	for (int i = 0; i < 64; i++)
	{
		snake->x_pos[i] = 0;
		snake->y_pos[i] = 0;
	}
	snake->x_pos[0] = 3;
	snake->x_pos[1] = 2;
	snake->y_pos[0] = 2;
	snake->y_pos[1] = 2;
	snake->x_pos[2] = 1;
	snake->y_pos[2] = 2;
	for (int i = 0; i < 9; i++)
	{
		snake->outputArray[i] = 0;
	}
}

void playLoseScreen(void)
{
	// Double X
	int doubleXSize = 16;
	uint8_t doubleX_x_pos[] = {1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8};
	uint8_t doubleX_y_pos[] = {1,8,2,7,3,6,4,5,4,5,3,6,2,7,1,8};
	uint8_t doubleXoutputArray[9] = {0};
	positionToMatrixPos(doubleX_x_pos, doubleX_y_pos, doubleXSize, doubleXoutputArray);

	LEDMatrixWrite(doubleXoutputArray);
	if (reset) return;
	Delay(500);
	matrixClear();
	if (reset) return;
	Delay(500);
	LEDMatrixWrite(doubleXoutputArray);
	if (reset) return;
	Delay(500);
	matrixClear();
	if (reset) return;
	Delay(500);
	if (reset) return;

	// R? (Restart question)
	int RSize = 23;
	uint8_t R_x_pos[] = {1,1,1,1,1,1,2,2,3,3,3,4,4,4,6,6,7,7,7,8,8,8,8};
	uint8_t R_y_pos[] = {1,2,3,4,5,6,4,6,3,4,6,1,2,5,5,6,1,3,6,3,4,5,6};
	uint8_t RoutputArray[9] = {0};
	positionToMatrixPos(R_x_pos, R_y_pos, RSize, RoutputArray);

	while(1)
	{
		for (volatile int i = 1; i <= 8; i++)
		{
			for (volatile int j = 1; j <= i; j++)
			{
				LEDMatrixRowWrite(RoutputArray, j);
				if (reset) return;
			}
			Delay(500);
		}
		matrixClear();
		if (reset) return;
		Delay(500);
		LEDMatrixWrite(RoutputArray);
		if (reset) return;
		Delay(500);
		matrixClear();
		if (reset) return;
		Delay(500);
		LEDMatrixWrite(RoutputArray);
		if (reset) return;
		Delay(4000);
		matrixClear();
		if (reset) return;
		Delay(500);
	}
}

void SnakeDead(void)
{
	alive = false;
}

void SnakeCheckAfterMove(snake_Type *snake)
{
	// Check if out of bounds
	uint8_t snake_head_x = snake->x_pos[0];
	uint8_t snake_head_y = snake->y_pos[0];
	if (snake_head_x < 1 || snake_head_x > 8)
	{
		SnakeDead();
	}
	if (snake_head_y < 1 || snake_head_y > 8)
	{
		SnakeDead();
	}
	// Check if head hits body
	for (int i = 1; i < snake->snakeSize; i++)
	{
		if (snake_head_x == snake->x_pos[i])
		{
			if (snake_head_y == snake->y_pos[i])
			{
				SnakeDead();
			}
		}
	}
}

void ClearOutputArray(snake_Type *snake)
{
	for (int i = 0; i < 9; i++)
	{
		snake->outputArray[i] = 0;
	}
}

void SnakeGrow(snake_Type *snake)
{
	snake->snakeSize += 1;
}

void AppleInit(apple_Type *apple)
{
	apple->x_pos[0] = 3;
	apple->y_pos[0] = 7;
}

void DisplayApple(snake_Type *snake, apple_Type *apple)
{
	positionToMatrixPos(apple->x_pos, apple->y_pos, 1, snake->outputArray);
}

bool CheckIfAppleCollected(uint8_t snakeHeadx, uint8_t snakeHeady, apple_Type *apple)
{
	if (snakeHeadx == apple->x_pos[0])
	{
		if (snakeHeady == apple->y_pos[0])
		{
			return true;
		}
	}
	return false;
}

void ReplaceApple(snake_Type *snake, apple_Type *apple)
{
	uint8_t newCords[8][8] = {0};

	for (int i = 0; i < snake->snakeSize; i++)
	{
		newCords[snake->y_pos[i] - 1][snake->x_pos[i] - 1] = 1;
	}

	uint8_t newSize = 0;
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 8; j++)
		{
			if (newCords[i][j] == 0)
			{
				newSize++;
			}
		}
	}
	uint8_t newPossiblePos[newSize];
	uint8_t newPossiblePosPtr = 0;
	uint8_t counter = 0;
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 8; j++)
		{
			if (newCords[i][j] == 0)
			{
				newPossiblePos[newPossiblePosPtr] = counter;
				newPossiblePosPtr++;
			}
			counter++;
		}
	}

	uint8_t randomPosIndex = rand() % newSize;
	uint8_t randomPos = newPossiblePos[randomPosIndex];

	apple->y_pos[0] = randomPos / 8 + 1;
	apple->x_pos[0] = randomPos % 8 + 1;
}

void DisplayGame(snake_Type *snake, apple_Type *apple)
{
	DisplaySnake(snake);
	DisplayApple(snake, apple);
	LEDMatrixWrite(snake->outputArray);
	ClearOutputArray(snake);
}

void ResetGame(snake_Type *snake, apple_Type *apple)
{
	SnakeInit(snake);
	AppleInit(apple);
	snake_direction = RIGHT;
	previous_direction = RIGHT;
	alive = true;
	reset = false;

	RandomSeedLoader();
}

void MoveSnake(snake_Type *snake, apple_Type *apple)
{
	uint8_t tempToPlacex = snake->x_pos[0];
	uint8_t tempToPlacey = snake->y_pos[0];
	uint8_t tempToStorex = 0;
	uint8_t tempToStorey = 0;
	switch(snake_direction)
	{
		case UP:
			tempToPlacey += 1;
			break;
		case RIGHT:
			tempToPlacex += 1;
			break;
		case DOWN:
			tempToPlacey -= 1;
			break;
		case LEFT:
			tempToPlacex -= 1;
	}

	bool apple_collected = CheckIfAppleCollected(tempToPlacex, tempToPlacey, apple);
	if (apple_collected)
	{
		SnakeGrow(snake);
	}

	for (int i = 0; i < snake->snakeSize; i++)
	{
		tempToStorex = snake->x_pos[i];
		tempToStorey = snake->y_pos[i];
		snake->x_pos[i] = tempToPlacex;
		snake->y_pos[i] = tempToPlacey;
		tempToPlacex = tempToStorex;
		tempToPlacey = tempToStorey;
	}
	previous_direction = snake_direction;

	if (apple_collected)
	{
		ReplaceApple(snake, apple);
	}
}
