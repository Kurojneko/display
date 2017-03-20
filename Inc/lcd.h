#ifndef __LCD_H
	#define __LCD_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>

void 							TFT_FillScreen(uint16_t color);
/*############################### SPIx #######################################*/
#define DISCOVERY_SPIx                          SPI5
#define DISCOVERY_SPIx_CLK_ENABLE()             __SPI5_CLK_ENABLE()
#define DISCOVERY_SPIx_GPIO_PORT                GPIOF                      /* GPIOF */
#define DISCOVERY_SPIx_AF                       GPIO_AF5_SPI5
#define DISCOVERY_SPIx_GPIO_CLK_ENABLE()        __GPIOF_CLK_ENABLE()
#define DISCOVERY_SPIx_GPIO_CLK_DISABLE()       __GPIOF_CLK_DISABLE()
#define DISCOVERY_SPIx_SCK_PIN                  GPIO_PIN_7                 /* PF.07 */
#define DISCOVERY_SPIx_MISO_PIN                 GPIO_PIN_8                 /* PF.08 */
#define DISCOVERY_SPIx_MOSI_PIN                 GPIO_PIN_9                 /* PF.09 */
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define SPIx_TIMEOUT_MAX              ((uint32_t)0x1000)


/*################################ LCD #######################################*/
/* Chip Select macro definition */
#define LCD_CS_LOW()       HAL_GPIO_WritePin(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, GPIO_PIN_RESET)
#define LCD_CS_HIGH()      HAL_GPIO_WritePin(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, GPIO_PIN_SET)

/* Set WRX High to send data */
#define LCD_WRX_LOW()      HAL_GPIO_WritePin(LCD_WRX_GPIO_PORT, LCD_WRX_PIN, GPIO_PIN_RESET)
#define LCD_WRX_HIGH()     HAL_GPIO_WritePin(LCD_WRX_GPIO_PORT, LCD_WRX_PIN, GPIO_PIN_SET)

/* Set WRX High to send data */
#define LCD_RDX_LOW()      HAL_GPIO_WritePin(LCD_RDX_GPIO_PORT, LCD_RDX_PIN, GPIO_PIN_RESET)
#define LCD_RDX_HIGH()     HAL_GPIO_WritePin(LCD_RDX_GPIO_PORT, LCD_RDX_PIN, GPIO_PIN_SET)

/** 
  * @brief  LCD Control pin  
  */ 
#define LCD_NCS_PIN                             GPIO_PIN_2
#define LCD_NCS_GPIO_PORT                       GPIOC
#define LCD_NCS_GPIO_CLK_ENABLE()               __GPIOC_CLK_ENABLE()
#define LCD_NCS_GPIO_CLK_DISABLE()              __GPIOC_CLK_DISABLE()
/**             
  * @}
  */ 
/** 
  * @brief  LCD Command/data pin  
  */
#define LCD_WRX_PIN                             GPIO_PIN_13
#define LCD_WRX_GPIO_PORT                       GPIOD
#define LCD_WRX_GPIO_CLK_ENABLE()               __GPIOD_CLK_ENABLE()
#define LCD_WRX_GPIO_CLK_DISABLE()              __GPIOD_CLK_DISABLE()
  
#define LCD_RDX_PIN                             GPIO_PIN_12
#define LCD_RDX_GPIO_PORT                       GPIOD
#define LCD_RDX_GPIO_CLK_ENABLE()               __GPIOD_CLK_ENABLE()
#define LCD_RDX_GPIO_CLK_DISABLE()              __GPIOD_CLK_DISABLE()


#ifdef __cplusplus
}
#endif

#endif /* __LCD_H */
