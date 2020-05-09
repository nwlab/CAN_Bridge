/*
 * ring.h
 *
 *  Created on: May 9, 2020
 *      Author: nightworker
 */

#ifndef INC_RING_H_
#define INC_RING_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
  uint8_t err_code;
  uint32_t ptr_in;
  uint32_t ptr_out;
  uint32_t length;
} ring_node_t;

uint8_t  ringInit(void);

uint8_t  ringCreate(ring_node_t *p_node, uint32_t length);

uint8_t  ringWriteUpdate(ring_node_t *p_node);
uint32_t ringWriteAvailable(ring_node_t *p_node);
uint32_t ringGetWriteIndex(ring_node_t *p_node);

uint8_t  ringReadUpdate(ring_node_t *p_node);
uint32_t ringReadAvailable(ring_node_t *p_node);
uint32_t ringGetReadIndex(ring_node_t *p_node);
uint32_t ringGetReadOffsetIndex(ring_node_t *p_node, uint32_t offset);

uint8_t ringFlush(ring_node_t *p_node);

#ifdef __cplusplus
}
#endif

#endif /* INC_RING_H_ */
