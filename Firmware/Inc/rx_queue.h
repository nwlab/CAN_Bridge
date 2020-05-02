/*
 * ring_buffer.h
 *
 *  Created on: Apr 19, 2020
 *      Author: nightworker
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <stdint.h>

void rx_queue_put(uint8_t data);
uint8_t rx_queue_get();
uint8_t rx_queue_not_empty();


#endif /* INC_RING_BUFFER_H_ */
