/* -------------------------------------------------------------------------
 * queue.h - htun queue interface defs
 * -------------------------------------------------------------------------
 */

#ifndef QUEUE_H
#define QUEUE_H

#include <semaphore.h>
#include <pthread.h>

#define Q_WAIT 1<<0
#define Q_PUSH 1<<1

#define q_isempty(q) (!(q)->nr_nodes)

/* Request queue node */
typedef struct _qnode_t {   /*请求队列节点*/
    void *data;
    size_t size;
    struct _qnode_t *next;
} qnode_t;

typedef struct {
    qnode_t *head;              /*队列头部*/
    qnode_t **tail;             /*队列尾部*/
    size_t nr_nodes;            /*节点个数*/
    size_t max_nodes;           /*最大节点个数*/
    pthread_mutex_t mutex;      /*互斥量用于保护链表*/
    pthread_cond_t reader_cond; /*条件变量*/
    pthread_cond_t writer_cond;
    sem_t cleanup_sem;          /*信号量*/
    size_t totsize;
    int readers;
    int writers;
    int shutdown;
    struct timeval lastadd;     /*最后一个任务时间*/
} queue_t;

/*
 * Returns a dynamically allocated queue_t to start a new queue.
 */
queue_t *q_init( void );

/*
 * Destroys a queue. The memory pointed to by q will be free()d and should not
 * be accessed after calling this function.
 */
void q_destroy( queue_t **q );

/* 
 * Pass in a pointer to the data you want to enqueue. Returns 0 on success, or
 * -1 on failure.
 * flags is the bitwise "or" of zero or more of the following flags:
 *  Q_WAIT - When queue is full, block until data can be added
 *  Q_PUSH - Put data at head of queue instead of end
 */
int q_add( queue_t *q, void *data, int flags, size_t elem_size );

/*
 * Returns the item at the top of the queue, or NULL if there is no data on
 * the queue and Q_WAIT is not set.
 * flags is the bitwise "or" of zero or more of the following flags:
 *  Q_WAIT - Block until there is data to return. Never return NULL.
 * If 'wait' is non-null, q_remove() will return after the specified amount of
 *  time if no data appeared yet.
 */
void *q_remove( queue_t *q, int flags, const struct timespec *wait );

/*
 * Returns 0 if the queue does not get data on it before the amount of time
 * specified in ts.
 * Returns nonzero if the queue already has data on it, or if it gets data on
 * it before the time runs out.
 */
int q_timedwait( queue_t *q, struct timespec *ts );

#endif


