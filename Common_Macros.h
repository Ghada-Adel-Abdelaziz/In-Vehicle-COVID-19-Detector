/*
 * Common_Macros.h
 *
 *  Created on: Jan 19, 2021
 *      Author: Ghada & Toqa
 */

//IRQ Number for stm32f407xx MCU

#define IRQ_NO_EXTI0      6
#define IRQ_NO_EXTI1      7
#define IRQ_NO_EXTI2      8
#define IRQ_NO_EXTI3      9
#define IRQ_NO_EXTI4      10
#define IRQ_NO_EXTI9_5    23
#define IRQ_NO_EXTI15_10  40


//Generic macros
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET
#define FLAG_SET        SET
#define FLAG_RESET      RESET


// descriptive macros for magic numbers
#define One_bit_shift                        1
#define Two_bits_shift                       2
#define Four_bits_shift                      4
#define Eight_Pins_for_GPIOxAFRH_or_AFRL     8
#define Four_Pins_for_SYSCFG_EXTICR          4
#define Four_Pins_for_IPR_reg                4
#define Eight_reg_bits                       8
#define Number_one_in_HEX                0x00000001


#define One_bit_mask                         1
#define One_bit_mask_by_HEX                 0x1
#define Two_consecutive_bits_mask_by_HEX    0x3
#define bits_mask_by_HEX                    0xF

#define NUM_OF_REG_BITS                      32