/* -------------------------------------------------------------------------
 * log.h 
  * -------------------------------------------------------------------------
 */

#ifndef __LOG_H
#define __LOG_H

#include <stdio.h>
#include <semaphore.h>

#define LOGLINE_MAX 200

typedef struct {
	char name[50];
    int fd;
    sem_t sem;
    int flags;
} log_t;
extern log_t log_my[]; 


#define DEBUG 1
#define INFO  2
#define WARN  3
#define ERROR 4
#define FATAL 5

/*
 * Logs to the logfile using printf()-like format strings.
 * 
 * log_t - The value you got back from a log_open() call.
 * level - can be one of: DEBUG, INFO, WARN, ERROR, FATAL
 * fmt   - is a printf()-like format string, followed by the parameters.
 *
 * Returns 0 on success, or -1 on failure.
 */
int lprintf( log_t *log, unsigned int level, char *fmt, ... );

#define LOG_TRUNC   1<<0
#define LOG_NODATE  1<<1
#define LOG_NOLF    1<<2
#define LOG_NOLVL   1<<3
#define LOG_DEBUG   1<<4
#define LOG_STDERR  1<<5
#define LOG_NOTID   1<<6

/*
 * Initializes the logfile to be written to with fprintf().
 *
 * fname - The name of the logfile to write to
 * flags - The bitwise 'or' of zero or more of the following flags:
 *          LOG_TRUNC   - Truncates the logfile on opening
 *          LOG_NODATE  - Omits the date from each line of the log
 *          LOG_NOLF    - Keeps from inserting a trailing '\n' when you don't.
 *          LOG_NOLVL   - Keeps from inserting a log level indicator.
 *          LOG_STDERR  - Sends log data to stderr as well as to the log.
 *                        (this not implemented yet)
 *
 * Returns NULL on failure, and a valid log_t (value > 0) on success.
 */
log_t *log_open( log_t* log,char *fname, int flags );

/*
 * Closes a logfile when it's no longer needed
 * 
 * log  - The log_t corresponding to the log you want to close
 */
void log_close( log_t *log );

extern log_t log_my[5];

extern int logtext( log_t log, unsigned int level, char *fmt, ... ) ;



#endif

