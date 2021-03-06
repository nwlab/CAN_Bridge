/*
 * ring.c
 *
 *  Created on: May 9, 2020
 *      Author: nightworker
 */

#include "ring.h"


uint8_t ringInit(void)
{
  return 0;
}

uint8_t ringCreate(ring_node_t *p_node, uint32_t length)
{
  uint8_t err_code = 0;


  p_node->ptr_in  = 0;
  p_node->ptr_out = 0;
  p_node->length  = length;

  return err_code;
}

uint32_t ringReadAvailable(ring_node_t *p_node)
{
  uint32_t length;

  length = (p_node->length + p_node->ptr_in - p_node->ptr_out) % p_node->length;

  return length;
}

uint32_t ringWriteAvailable(ring_node_t *p_node)
{
  uint32_t length;
  uint32_t read_length;

  read_length = ringReadAvailable(p_node);

  length = p_node->length - read_length - 1;

  return length;
}

uint32_t ringGetWriteIndex(ring_node_t *p_node)
{
  return p_node->ptr_in;
}

uint8_t ringWriteUpdate(ring_node_t *p_node)
{
  uint8_t err_code = 0;
  uint32_t next_index;

  next_index = p_node->ptr_in + 1;

  if (next_index == p_node->length)
  {
    next_index = 0;
  }

  if (next_index != p_node->ptr_out)
  {
    p_node->ptr_in = next_index;
  }
  else
  {
    //err_code = ERR_FULL;
    ringReadUpdate(p_node);
    p_node->ptr_in = next_index;
  }

  return err_code;
}

uint8_t ringReadUpdate(ring_node_t *p_node)
{
  uint8_t err_code = 0;
  uint32_t index;
  uint32_t next_index;


  index      = p_node->ptr_out;
  next_index = p_node->ptr_out + 1;

  if (next_index == p_node->length)
  {
    next_index = 0;
  }

  if (index != p_node->ptr_in)
  {
    p_node->ptr_out = next_index;
  }
  else
  {
    err_code = 1;
  }

  return err_code;
}

uint32_t ringGetReadIndex(ring_node_t *p_node)
{
  return p_node->ptr_out;
}

uint32_t ringGetReadOffsetIndex(ring_node_t *p_node, uint32_t offset)
{
  uint32_t index;

  index = (p_node->length + p_node->ptr_out + offset) % p_node->length;

  return index;
}

uint8_t ringFlush(ring_node_t *p_node)
{
  uint8_t err_code = 0;

  p_node->ptr_in  = 0;
  p_node->ptr_out = 0;

  return err_code;
}


