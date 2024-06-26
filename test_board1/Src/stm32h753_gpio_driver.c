/*
 * stm32h753_gpio_driver.c
 *
 *  Created on: Jun 25, 2024
 *      Author: Bharath
 */

#include <stm32h753xx.h>
#include <stm32h753_gpio_driver.h>
/* documentation of every function has to be done for all the functions separately */

//periperal clock configuartion

void GPIO_PeriClockContrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == Enable)
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_EN();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_EN();
	}else
	{
		if(pGPIOx == GPIOA)
					GPIOA_PCLK_DEN();
				else if(pGPIOx == GPIOB)
					GPIOB_PCLK_DEN();
				else if(pGPIOx == GPIOC)
					GPIOC_PCLK_DEN();
				else if(pGPIOx == GPIOD)
					GPIOD_PCLK_DEN();
				else if(pGPIOx == GPIOE)
					GPIOE_PCLK_DEN();
				else if(pGPIOx == GPIOF)
					GPIOF_PCLK_DEN();
				else if(pGPIOx == GPIOG)
					GPIOG_PCLK_DEN();
				else if(pGPIOx == GPIOH)
					GPIOH_PCLK_DEN();
	}

};








// init and de-init


/*
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint8_t temp=0;  //temporary register

	//ENABLE THE PHERIPHERAL CLOCK

	GPIO_PeriClockContrl(pGPIOHandle-> pGPIOx, Enable);

	//1. configure the mode of GPIO pin
          if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
          {
        	  temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
        	  pGPIOHandle -> pGPIOx-> MODER &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); //clearing
        	  pGPIOHandle -> pGPIOx-> MODER |= temp;  //setting


          }else
          {
        	  //will code later(IRQ)
        	  if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        	  {
        		  //1.C ONFIGURE FTSR
        		  EXTI-> FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        		  //CLEAR THE CORRESPONDING RTSR BIT
        		   EXTI-> RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        	  }else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        	  {
        		  //1. CONFIGURE RTSR
        		  EXTI-> RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        		  //CLEAR THE CORRESPONDING FTSR BIT
        		   EXTI-> FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        	  }else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        	  {
        		  //1.CONFIGURE FOR BOTH FTSR AND RTSR
        		  EXTI-> RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

        		  EXTI-> FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

        	  }

        	  //2. CONFIGURE THE GPIO PORT SELECTION IN SYSCFG_EXTICR
        	  	 uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
        	  	uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
        	  	uint8_t  portcode = GPIO_BADDR_TO_CODE(pGPIOHandle->pGPIOx);
        	  	SYSCFG_PCLK_EN();
        	  	SYSCFG-> EXTICR[temp1] = portcode << (temp2*4);

           	  //3. ENABLE THE EXTI INTERRUPT DELIVERY USING IMR
          EXTI-> IMR1 |= 1<< pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber;
          }

          temp = 0;




	//2. configure the speed
          temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
          pGPIOHandle -> pGPIOx-> OSPEEDER &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); //clearing
          pGPIOHandle -> pGPIOx-> OSPEEDER |= temp; //setting

          temp = 0;

	//3. configure the pupd settings
          temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
          pGPIOHandle -> pGPIOx-> PUPDR &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); //clearing
          pGPIOHandle -> pGPIOx-> PUPDR |= temp; //setting

                    temp = 0;

	//4. c0nfigure the otype
          temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinOtype << (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
          pGPIOHandle -> pGPIOx-> OTYPER &= ~(0x1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); //clearing
          pGPIOHandle -> pGPIOx-> OTYPER |= temp;  //setting

                     temp = 0;

	//5. configure the alt functionality
       if(pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF)
       {
    	   // configure the alt function registers.
    	   	   uint8_t temp1,temp2;

    	   	   temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
    	   	   temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
    	   	   pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
    	   	   pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunction << (4*temp2));
       }


}


*/



void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint8_t temp = 0;  // Temporary register
    uint16_t Rand = 0;  // Temporary register

    // Enable the peripheral clock
    GPIO_PeriClockContrl(pGPIOHandle->pGPIOx, Enable);

    // 1. Configure the mode of GPIO pin
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {

    	Rand = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing
        pGPIOHandle->pGPIOx->MODER |= Rand;  // Setting
    }
    else
    {
        // Handle interrupt mode configuration later
        if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            // 1. Configure FTSR
            EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // Clear the corresponding RTSR bit
            EXTI->RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            // 1. Configure RTSR
            EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // Clear the corresponding FTSR bit
            EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            // 1. Configure for both FTSR and RTSR
            EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        // 2. Configure the GPIO port selection in SYSCFG_EXTICR
        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        uint8_t portcode = GPIO_BADDR_TO_CODE(pGPIOHandle->pGPIOx);
        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

        // 3. Enable the EXTI interrupt delivery using IMR
        EXTI->IMR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

    Rand = 0;

    // 2. Configure the speed
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing
    pGPIOHandle->pGPIOx->OSPEEDER |= temp; // Setting

    temp = 0;

    // 3. Configure the pull-up/pull-down settings
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing
    pGPIOHandle->pGPIOx->PUPDR |= temp; // Setting

    temp = 0;

    // 4. Configure the output type
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOtype << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing
    pGPIOHandle->pGPIOx->OTYPER |= temp;  // Setting

    temp = 0;

    // 5. Configure the alternate functionality
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF)
    {
        // Configure the alt function registers.
        uint8_t temp1, temp2;
        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunction << (4 * temp2));
    }
}



void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}

}









// Data Read and Write

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
     uint8_t value;
     value =  (uint8_t) ((pGPIOx->IDR    >> PinNumber) & 0x00000001);
     return value;

}



uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t Value;
	     Value =  (uint16_t)pGPIOx->IDR ;
	     return Value;
}


void GPIO_WriteToOutputPIn(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value)
{
   if(Value == GPIO_PIN_SET)
   {
	   // write 1 to the output data register at the bit field corresponding to the pin number
	   	   pGPIOx->ODR |= ( 1 << PinNumber);
   }else
   {
	   //write 0
        pGPIOx->ODR &= ~( 1 << PinNumber);
   }
}




void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
        pGPIOx->ODR = Value;
}



void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
   pGPIOx->ODR ^= ( 1 << PinNumber);
}



// IRQ configuration and ISR Handling

void GPIO_IRQITConfig(uint8_t IRQNumber ,uint8_t EnorDi)
{
	if(EnorDi == Enable)
	{
	  if(IRQNumber <= 31)
	  {
		  //program ISER0 register
		  *NVIC_ISER0 |= (1 << IRQNumber);

	  }else if(IRQNumber > 31 && IRQNumber <64) //32 to 63
	  {
		  //program ISER1 register
		  *NVIC_ISER1 |= (1 << IRQNumber % 32);

	  }else if(IRQNumber >= 64 && IRQNumber < 96) //64 to 95
	  {
		  //program ISER2 register
		  *NVIC_ISER3 |= (1 << IRQNumber % 64);

	  }else
	  {
		  if(IRQNumber <- 31)
		  {
			  //program ICER0 register
			  *NVIC_ICER0 |= (1 << IRQNumber);

		  }else if(IRQNumber > 31 && IRQNumber <64) //32 to 63
		  {
			  //program ICER1 register
			  *NVIC_ICER1 |= (1 << IRQNumber % 32);

		  }else if(IRQNumber >=64 && IRQNumber <96) //64 to 95
		  {
			  //program ICER2 register
			  *NVIC_ICER3 |= (1 << IRQNumber % 64);

		  }
	  }
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber ,uint8_t IRQPriority)
{
		//1. find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR1_BADDR + iprx*4) |= IRQPriority << (shift_amount);
	// *(NVIC_PR_BADDR + iprx*4) |= IRQPriority << (8 * iprx_section);
	//go through this
}




void GPIO_IRQHandling(uint8_t PinNumber)      // Complexity of interrupt handling depends on the peripheral used(EX: I2C Peripherals are way to difficult to handle when compared to GPIO Peripheral)
{
		//clear the exti pc register corresponding to the pin number
	if(EXTI-> PR1 & (1<<PinNumber))
	{
		//clear
		EXTI-> PR1 |= (1<<PinNumber);
	}
}

