/*******************************************************************************
  * @file           : boot.h
  * @author         : Andrii Iakovenko
  * @date           : 27 Mar 2020
  * @brief          : Header for state.с file.
  ******************************************************************************/
#ifndef _BOOT_H_
#define _BOOT_H_

void reboot();
void jump_to_app(uint32_t addr);

#endif /* _BOOT_H_ */
