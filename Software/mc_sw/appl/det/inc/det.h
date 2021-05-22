/*
 * det.h
 *
 *  Created on: 22 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

#ifndef DET_INC_DET_H_
#define DET_INC_DET_H_

#ifdef __cplusplus
 extern "C" {
#endif
 /**
   * @brief  This function is executed in case of error occurrence.
   * @param  file: The file name as string.
   * @param  line: The line in file as a number.
   * @retval None
   */
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line);
#endif /* USE_FULL_ASSERT */

#endif /* DET_INC_DET_H_ */
