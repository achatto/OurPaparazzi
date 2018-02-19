#include "prqueue.h"
#include <string.h> // for memcopy()

/**
 * Compare two messages
 * Returns true if A < B, false otherwise
 */
bool pq_isless(const msg_container_t a, const msg_container_t b)
{
  if (a.priority > b.priority)
    return false;  // A !< B
  if (a.priority < b.priority)
    return true;  // A < B
  // both priorities equal
  if (a.insertion_time < b.insertion_time)
    return true;  // A is older than B => A < B
  return false;  // A is newer than B => A !< B
}

/**
 * Initialize queue
 */
void pq_init(pqueue_t *queue)
{
  for (uint8_t idx = 0; idx < MAX_Q_SIZE; idx++) {
    memset(&queue->elements[idx], 0, sizeof(msg_container_t));
  }
  queue->N = 0;
}

/**
 * Return current size of the queue
 * Size is always <= MAX_Q_SIZE
 */
uint8_t pq_size(pqueue_t *queue)
{
  return queue->N;
}

/**
 * Push new message into the queue
 * returns true if the message was added,
 * false if the queue is full
 */
bool pq_push(pqueue_t *queue, const msg_container_t* element)
{
  if (queue->N >= MAX_Q_SIZE)
    return false;
  memcpy(&queue->elements[queue->N], element, sizeof(msg_container_t));
  queue->N++;
  return true;
}

/**
 * Push new message, and if the queue is full and the new message has
 * higher priority replace an exisiting message in the queue.
 *
 * In case there is more messages with the same priority, the oldest one
 * is removed.
 *
 * Always returns true.
 */
bool pq_push_over(pqueue_t *queue, const msg_container_t* element)
{
  uint8_t id = queue->N;

  if (id >= MAX_Q_SIZE) {
    id = pq_get_min_index(queue);
  } else {
    queue->N++;
  }

  memcpy(&queue->elements[id], element, sizeof(msg_container_t));

  return true;
}

/**
 * Returns index of the lowest priority/oldest message
 */
uint8_t pq_get_min_index(pqueue_t *queue)
{
  uint8_t min_id = 0;
  uint8_t i = 0;

  if (queue->N <= 0)
    return 0;
  if (queue->N > 1)
    min_id = 0;

  for (i = 1; i < queue->N; i++)
    if (pq_isless(queue->elements[i], queue->elements[min_id]))
      min_id = i;

  return min_id;
}

/**
 * Pop the highest priority/newest message into `max_element`
 * and remove it from the queue.
 * Returns false if the queue is empty, true otherwise.
 */
bool pq_getmax(pqueue_t *queue, msg_container_t *max_element)
{
  uint8_t max_id = 0;
  uint8_t i = 0;

  if (queue->N <= 0)
    return false;
  if (queue->N > 1)
    max_id = 0;

  for (i = 1; i < queue->N; i++)
    if (pq_isless(queue->elements[max_id], queue->elements[i]))
      max_id = i;

  memcpy(max_element, &queue->elements[max_id], sizeof(msg_container_t));
  memcpy(&queue->elements[max_id], &queue->elements[queue->N - 1],
      sizeof(msg_container_t));
  queue->N--;

  return true;
}

/**
 * Copy the highest priority/newest message into `max_element`
 * and retain it in the queue.
 * Returns false if the queue is empty, true otherwise.
 */
bool pq_peek(pqueue_t *queue, msg_container_t *max_element)
{
  uint8_t max_id = 0;
  uint8_t i;

  if (queue->N <= 0)
    return false;
  for (i = 1; i < queue->N; i++)
    if (pq_isless(queue->elements[max_id], queue->elements[i]))
      max_id = i;

  memcpy(max_element, &queue->elements[max_id], sizeof(msg_container_t));
  return true;
}
