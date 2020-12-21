
/* APB* AND AHB* peripheral base address */

#define APB1_PERIADDR    0X40000000U   //ApB1 peripheral base address
#define APB2_PERIADDR    0X40010000U   //ApB2 peripheral base address
#define AHB1_PERIADDR    0X40020000U   //AHB1 peripheral base address
#define AHB2_PERIADDR    0X50000000U   //AHB2 peripheral base address
#define AHB3_PERIADDR    0XA0000000U   //AHB3 peripheral base address

/* base address for all GPIO pins FROM A TO I */

#define GPIO_A_ADDR (AHB1_PERIADDR + 0X0000)
#define GPIO_B_ADDR (AHB1_PERIADDR + 0X0400)
#define GPIO_C_ADDR (AHB1_PERIADDR + 0X0800)
#define GPIO_D_ADDR (AHB1_PERIADDR + 0X0C00)
#define GPIO_E_ADDR (AHB1_PERIADDR + 0X1000)
#define GPIO_F_ADDR (AHB1_PERIADDR + 0X1400)
#define GPIO_G_ADDR (AHB1_PERIADDR + 0X1800)
#define GPIO_H_ADDR (AHB1_PERIADDR + 0X1C00)
#define GPIO_I_ADDR (AHB1_PERIADDR + 0X2000)
/* base address for all USART pins FROM 1 TO 6*/
#define USART1_ADDR (APB2_PERIADDR + 0X1000)
#define USART2_ADDR (APB1_PERIADDR + 0X4400)
#define USART3_ADDR (APB1_PERIADDR + 0X4800)
#define USART4_ADDR (APB1_PERIADDR + 0X4C00)
#define USART5_ADDR (APB1_PERIADDR + 0X5000)
#define USART6_ADDR (APB2_PERIADDR + 0X1400)

/*GPIO defintion */

#define GPIOA    ((GPIO_regdef_t*)GPIO_A_ADDR)
#define GPIOB    ((GPIO_regdef_t*)GPIO_B_ADDR)
#define GPIOC    ((GPIO_regdef_t*)GPIO_C_ADDR)
#define GPIOD    ((GPIO_regdef_t*)GPIO_D_ADDR)
#define GPIOE    ((GPIO_regdef_t*)GPIO_E_ADDR)
#define GPIOF    ((GPIO_regdef_t*)GPIO_F_ADDR)
#define GPIOG    ((GPIO_regdef_t*)GPIO_G_ADDR)
#define GPIOH    ((GPIO_regdef_t*)GPIO_H_ADDR)
#define GPIOI    ((GPIO_regdef_t*)GPIO_I_ADDR)
/* USART definition */

#define USART1  ((USART_RegDef_t*)USART1_ADDR)
#define USART2  ((USART_RegDef_t*)USART2_ADDR)
#define USART3  ((USART_RegDef_t*)USART3_ADDR)
#define USART4  ((USART_RegDef_t*)USART4_ADDR)
#define USART5  ((USART_RegDef_t*)USART5_ADDR)
#define USART6  ((USART_RegDef_t*)USART6_ADDR)
//system configuration definition
#define SYSCFG  ((SYSCFG_regdef_t*)SYSCFG_ADDR)
//GPIO clock enable macro
#define GPIOA_PCLK_EN  (RCC -> AHB1ENR |= (1 << 0))//set bit0 in register(AHB1ENR) to enable Clock for GPIOA
#define GPIOB_PCLK_EN  (RCC -> AHB1ENR |= (1 << 1))//set bit1 in register(AHB1ENR) to enable Clock for GPIOB
#define GPIOC_PCLK_EN  (RCC -> AHB1ENR |= (1 << 2))//set bit2 in register(AHB1ENR) to enable Clock for GPIOC
#define GPIOD_PCLK_EN  (RCC -> AHB1ENR |= (1 << 3))//set bit3 in register(AHB1ENR) to enable Clock for GPIOD
#define GPIOE_PCLK_EN  (RCC -> AHB1ENR |= (1 << 4))//set bit4 in register(AHB1ENR) to enable Clock for GPIOE
#define GPIOF_PCLK_EN  (RCC -> AHB1ENR |= (1 << 5))//set bit5 in register(AHB1ENR) to enable Clock for GPIOF
#define GPIOG_PCLK_EN  (RCC -> AHB1ENR |= (1 << 6))//set bit6 in register(AHB1ENR) to enable Clock for GPIOG
#define GPIOH_PCLK_EN  (RCC -> AHB1ENR |= (1 << 7))//set bit7 in register(AHB1ENR) to enable Clock for GPIOH
#define GPIOI_PCLK_EN  (RCC -> AHB1ENR |= (1 << 8))//set bit8 in register(AHB1ENR) to enable Clock for GPIOI

//UART peripheral clock enable MACRO (4 USART / 2 UART)
#define USART1_PCLK_EN  (RCC -> APB2ENR |= (1 << 4)) //set bit4 in register(APB2ENR) to enable Clock for UART1
#define USART2_PCLK_EN  (RCC -> APB1ENR |= (1 << 17))//set bit17 in register(APB1ENR) to enable Clock for UART2
#define USART3_PCLK_EN  (RCC -> APB1ENR |= (1 << 18))//set bit18 in register(APB1ENR) to enable Clock for UART3
#define USART4_PCLK_EN  (RCC -> APB1ENR |= (1 << 19))//set bit19 in register(APB1ENR) to enable Clock for UART4
#define USART5_PCLK_EN  (RCC -> APB1ENR |= (1 << 20))//set bit20 in register(APB1ENR) to enable Clock for UART5
#define USART6_PCLK_EN  (RCC -> APB2ENR |= (1 << 5)) //set bit5 in register(APB2ENR) to enable Clock for UART6
//sysCFG PERIPHERAL CLOCK ENABLE MACRO
#define SYSCFG_PCLK_EN  (RCC->APB2ENR  |= (1 << 14))
//GPIO clock disable macro
#define GPIOA_PCLK_DI  (RCC -> AHB1ENR &= ~(1 << 0))//Clear bit0 in register(AHB1ENR) to disable Clock for GPIOA
#define GPIOB_PCLK_DI  (RCC -> AHB1ENR &= ~(1 << 1))//Clear bit1 in register(AHB1ENR) to disable Clock for GPIOB
#define GPIOC_PCLK_DI  (RCC -> AHB1ENR &= ~(1 << 2))//Clear bit2 in register(AHB1ENR) to disable Clock for GPIOC
#define GPIOD_PCLK_DI  (RCC -> AHB1ENR &= ~(1 << 3))//Clear bit3 in register(AHB1ENR) to enable Clock for GPIOD
#define GPIOE_PCLK_DI  (RCC -> AHB1ENR &= ~(1 << 4))//Clear bit4 in register(AHB1ENR) to disable Clock for GPIOE
#define GPIOF_PCLK_DI  (RCC -> AHB1ENR &= ~(1 << 5))//Clear bit5 in register(AHB1ENR) to disable Clock for GPIOF
#define GPIOG_PCLK_DI  (RCC -> AHB1ENR &= ~(1 << 6))//Clear bit6 in register(AHB1ENR) to disable Clock for GPIOG
#define GPIOH_PCLK_DI  (RCC -> AHB1ENR &= ~(1 << 7))//Clear bit7 in register(AHB1ENR) to disable Clock for GPIOH
#define GPIOI_PCLK_DI  (RCC -> AHB1ENR &= ~(1 << 8))//Clear bit8 in register(AHB1ENR) to disable Clock for GPIOI
//UART peripheral clock DISABLE MACRO
#define USART1_PCLK_DI  (RCC -> APB2ENR &= ~(1 << 4))//Clear bit4 in register(APB2ENR) to disable Clock for UART1
#define USART2_PCLK_DI  (RCC -> APB1ENR &= ~(1 << 17))//Clear bit17 in register(APB1ENR) to disable Clock for UART2
#define USART3_PCLK_DI  (RCC -> APB1ENR &= ~(1 << 18))//Clear bit18 in register(APB1ENR) to disable Clock for UART3
#define USART4_PCLK_DI  (RCC -> APB1ENR &= ~(1 << 19))//Clear bit19 in register(APB1ENR) to disable Clock for UART4
#define USART5_PCLK_DI  (RCC -> APB1ENR &= ~(1 << 20))//Clear bit20 in register(APB1ENR) to disable Clock for UART5
#define USART6_PCLK_DI  (RCC -> APB2ENR &= ~(1 << 5))//Clear bit5 in register(APB2ENR) to disable Clock for UART6
//sysCFG PERIPHERAL CLOCK DISABLE MACRO
#define SYSCFG_PCLK_DI  (RCC -> APB2ENR &= ~(1 << 14))
//Macro to reset GPIO peripherals
#define GPIOA_REG_RESET()    do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));  }while(0)// Set bit0 in Register(AHB1RSTR)to reset then Clear then stop
#define GPIOB_REG_RESET()    do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));  }while(0)// Set bit1 in Register(AHB1RSTR)to reset then Clear then stop
#define GPIOC_REG_RESET()    do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));  }while(0)// Set bit2 in Register(AHB1RSTR)to reset then Clear then stop
#define GPIOD_REG_RESET()    do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));  }while(0)// Set bit3 in Register(AHB1RSTR)to reset then Clear then stop
#define GPIOE_REG_RESET()    do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));  }while(0)// Set bit4 in Register(AHB1RSTR)to reset then Clear then stop
#define GPIOF_REG_RESET()    do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));  }while(0)// Set bit5 in Register(AHB1RSTR)to reset then Clear then stop
#define GPIOG_REG_RESET()    do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));  }while(0)// Set bit6 in Register(AHB1RSTR)to reset then Clear then stop
#define GPIOH_REG_RESET()    do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));  }while(0)// Set bit7 in Register(AHB1RSTR)to reset then Clear then stop
#define GPIOI_REG_RESET()    do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));  }while(0)// Set bit8 in Register(AHB1RSTR)to reset then Clear then stop
/*GPIO peripheral structure definition*/

typedef struct
{
	__vol uint32_t MODER;               //GPIO port mode register
	__vol uint32_t OTYPER;              //GPIO port output type register
	__vol uint32_t OSPEEDR;             //GPIO port output speed register
	__vol uint32_t PUPDR;               //GPIO port pull-up/pull-down register
	__vol uint32_t IDR;                 //GPIO port input data register
	__vol uint32_t ODR;                 //GPIO port output data register
	__vol uint32_t BSRR;                //GPIO port bit set/reset register
	__vol uint32_t LCKR;                //GPIO port configuration lock register
	__vol uint32_t AFR[2];              //AFR[0] :GPIO alternate function low register AFR[1] :GPIO alternate function high register
}GPIO_regdef_t;

/*
 * UART peripheral structure definition
 */

typedef struct
{
	__vol uint32_t USART_SR;
	__vol uint32_t USART_DR;
	__vol uint32_t USART_BRR;
	__vol uint32_t USART_CR1;
	__vol uint32_t USART_CR2;
	__vol uint32_t USART_CR3;
	__vol uint32_t USART_GTPR;
}USART_RegDef_t; //list of Registers
//Bit position macro for USART status register

#define USART_SR_PE    0
#define USART_SR_FE    1
#define USART_SR_NF    2
#define USART_SR_ORE   3
#define USART_SR_IDLE  4
#define USART_SR_RXNE  5
#define USART_SR_TC    6
#define USART_SR_TXE   7
#define USART_SR_LBD   8
#define USART_SR_CTS   9
//Bit position macro for USART control register 1,2,3

#define USART_CR1_SBK     0
#define USART_CR1_RWU     1
#define USART_CR1_RE      2
#define USART_CR1_TE      3
#define USART_CR1_IDLEIE  4
#define USART_CR1_RXNEIE  5
#define USART_CR1_TCIE    6
#define USART_CR1_TXEIE   7
#define USART_CR1_PEIE    8
#define USART_CR1_PS      9
#define USART_CR1_PCE    10
#define USART_CR1_WAKE   11
#define USART_CR1_M    	 12
#define USART_CR1_UE     13
#define USART_CR1_OVER8  15

#define USART_CR2_ADD     0
#define USART_CR2_LBDL    5
#define USART_CR2_LBDIE   6
#define USART_CR2_LBCL    8
#define USART_CR2_CPHA    9
#define USART_CR2_CPOL    10
#define USART_CR2_CLKEN   11
#define USART_CR2_STOP    12
#define USART_CR2_LINEN   14

#define USART_CR3_EIE      0
#define USART_CR3_IREN     1
#define USART_CR3_IRLP     2
#define USART_CR3_HDSEL    3
#define USART_CR3_NACK     4
#define USART_CR3_SCEN     5
#define USART_CR3_DMAR     6
#define USART_CR3_DMAT     7
#define USART_CR3_RTSE     8
#define USART_CR3_CTSE     9
#define USART_CR3_CTSIE    10
#define USART_CR3_ONEBIT   11

//Generic macros
#define ENABLE   1
#define DISABLE  0
#define SET   ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET  SET
#define GPIO_PIN_RESET  RESET
#define FLAG_SET    SET
#define FLAG_RESET  RESET

#include "stm32f407xx_GPIO_Driver.h"
#include "stm32f407xx_USART_Driver.h"

#endif /* INC_STM32F407XX_H_ */