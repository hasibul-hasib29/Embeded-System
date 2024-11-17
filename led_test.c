#include "stm32f10x.h" // STM32 Device header
#include <stdint.h>
#include <stdlib.h>

// Function Prototypes
void GPIO_Clock_Enable(void);
void GPIO_Pin_Init(void);
void EXTI_Config(void);
void systick_init(void);
void delay_ms(void);
void delay_t(unsigned long t);

// GPIO Clock Enable
void GPIO_Clock_Enable(void)
{
    RCC->APB2ENR |= (1UL << 2); // A clock
    RCC->APB2ENR |= (1UL << 4); // c
    RCC->APB2ENR |= (1UL << 0); // clk afio
    RCC->APB2ENR |= (1UL << 3); // clk b
}

// GPIO Pin Initialization
void GPIO_Pin_Init(void)
{
    // Configure PC15 as input (floating)
    GPIOC->CRH &= ~(15UL << 28); // Clear CNF15[1:0] and MODE15[1:0]
    GPIOC->CRH |= (4UL << 28);   // Set CNF15[1:0] to floating input (CNF=01, MODE=00)

    // Configure PA1�PA6 as outputs (push-pull, max speed 50 MHz)
    for (int pin = 1; pin <= 6; pin++)
    {
        GPIOA->CRL &= ~(15UL << (pin * 4)); // Clear MODE and CNF bits
        GPIOA->CRL |= (3UL << (pin * 4));   // Set MODE to 50 MHz and CNF to push-pull
    }
}
volatile int led_position = 3; // position
volatile int flag = 0;
// EXTI Configuration
void EXTI_Config(void)
{
    // Map PC15 to EXTI15
    AFIO->EXTICR[3] &= ~(15UL << 12); // Clear EXTI15 mapping
    AFIO->EXTICR[3] |= (2UL << 12);   // Map PC15 to EXTI15

    AFIO->EXTICR[0] &= ~(15UL << 0);
    AFIO->EXTICR[0] &= ~(15UL << 4);
    AFIO->EXTICR[0] |= (1UL << 0);
    AFIO->EXTICR[0] |= (1UL << 4);

    // Configure EXTI15 for rising edge trigger
    EXTI->IMR |= (1UL << 15) | (1UL << 0) | (1UL << 1);  // Unmask EXTI15
    EXTI->RTSR |= (1UL << 15) | (1UL << 0) | (1UL << 1); // Trigger on rising edge
    // For falling edge: EXTI->FTSR |= (1UL << 15);

    NVIC_EnableIRQ(EXTI15_10_IRQn); // Enable EXTI15_10 interrupt in NVIC
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
}

// SysTick Timer Initialization
void systick_init(void)
{
    SysTick->LOAD = 72000 - 1; // Set reload register for 1 ms
    SysTick->VAL = 0;          // Clear current value register
    SysTick->CTRL = 5;         // Enable SysTick with processor clock
}

// SysTick Delay (1 ms)
void delay_ms(void)
{
    while (!(SysTick->CTRL & (1 << 16)))
        ; // Wait for COUNTFLAG to be 1
}

// SysTick Delay for Multiple Milliseconds
void delay_t(unsigned long t)
{
    while (t--)
    {
        delay_ms();
    }
}
void Update_LED(int i);
void Update_LED(int i)
{

    GPIOA->ODR = (1UL << i); //  LED position
}
volatile int start = 0;
// EXTI Interrupt Handler
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1UL << 15))
    {                            // Check if EXTI15 triggered
        EXTI->PR |= (1UL << 15); // Clear the interrupt pending flag
        start = 1;
        flag = 0;
        led_position = 3;
        GPIOA->ODR &= ~(0xFFUL << 0);

        GPIOA->ODR |= (1UL << 3); // Light up corresponding LED (PA1�PA6
    }
}

void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & (1UL << 0))
    {
        EXTI->PR |= (1UL << 0); // Clear interrupt pending flag
        if (!start)
            return;

        if (!flag)
        {
            led_position--;
            Update_LED(led_position);
        }
        if (led_position <= 1)
        {
            flag = 1;
            GPIOA->ODR &= 0;
            GPIOA->ODR |= (1UL << 1);
            GPIOA->ODR |= (1UL << 2);
        }
    }
}

void EXTI1_IRQHandler(void)
{
    if (EXTI->PR & (1UL << 1))
    {
        EXTI->PR |= (1UL << 1); // Clear interrupt pending flag
        if (!start)
            return;

        if (!flag)
        {
            led_position++;
            Update_LED(led_position);
        }
        if (led_position >= 5)
        {
            flag = 1;
            GPIOA->ODR &= ~(0xFFUL << 0);
            GPIOA->ODR |= (1UL << 4);
            GPIOA->ODR |= (1UL << 5);
        }
    }
}

// Main Function
int main(void)
{
    GPIO_Clock_Enable(); // Enable clocks for GPIOA and GPIOC
    GPIO_Pin_Init();     // Configure GPIO pins
    EXTI_Config();       // Configure EXTI
    systick_init();      // Initialize SysTick Timer

    while (1)
    {
        // Main loop does nothing; interrupt handles the dice roll
    }
}
