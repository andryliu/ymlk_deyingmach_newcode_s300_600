/* -------------------------------------------------------------------------
 * common.h 
 * -------------------------------------------------------------------------
 */

#ifndef __COMMON_H
#define __COMMON_H

#include <netinet/in.h>
#include <time.h>

#include "log.h"

#ifdef _DEBUG
#define dprintf lprintf
#else
#define dprintf(...)
#endif



/* Become a daemon: fork, die, setsid, fork, die, disconnect */
void daemonize( void );

/* Check the validity of the configuration data */
/*int config_check( config_data_t *c );
*/
/* Drop privileges to nobody. Optional reason for log */
void dropprivs(char *reason);

/* Get root privileges back. Optional reason for log */
void getprivs(char *reason);

/* initialize signames[] array */
/*void init_signames(void);
*/
/* Variables used by other modules */
extern log_t *log_g;

/*extern config_data_t *config;*/
/*extern char *signames[64]; *//* 64 names of signals */

#endif /* _COMMON_DEFS_H_ */

