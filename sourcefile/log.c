/* -------------------------------------------------------------------------
 * log.c - htun logging functions
  * -------------------------------------------------------------------------
 */

#include <stdio.h>
#include <unistd.h>
#include <semaphore.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "log.h"
log_t log_my[5];


/* Allows printf()-like interface to file descriptors without the
 * complications that arise from mixing stdio and low level calls 
 * FIXME: Needs date and time before logfile entries.
 */
int logtext( log_t log, unsigned int level, char *fmt, ... ) {
    int fd;
    int rc;
	// int size;
    va_list ap;
    time_t now;
    char date[50];
    static char line[LOGLINE_MAX];
    static char threadnum[10];
    int cnt = 0;
	
	struct stat buf;  
    static char *levels[6] = { "[(bad)] ", 
                               "[debug] ", 
                               "[info ] ", 
                               "[warn ] ", 
                               "[error] ", 
                               "[fatal] " };

   
    if(stat(log.name, &buf)<0)  
    {  
        return 0;  
    }  
		 
   if (buf.st_size > 35000000)// 35m
   {
       // log.fd = 
        fmt = "LOG file overload size \r\n";

        // printf("size = %d\n", buf.st_size);
   		// close(log.fd);
		
   		// for(i = 0;i<4;i++)
	    //     rename(log[i+1].name, log[i].name);
		
		// log[4].fd = open("/mnt/nandflash/log4", O_WRONLY|O_CREAT|O_NOCTTY | 
        //         (0&LOG_TRUNC ? O_TRUNC : O_APPEND) , 0666);
		// strcpy(log[0].name,"/mnt/nandflash/log4");

   }

    /* Prepare the date string   日期*/
    if( !(log.flags&LOG_NODATE) ) {
        now = time(NULL);
        strcpy(date, ctime(&now));
        date[strlen(date) - 6] =' ';
        date[strlen(date) - 5] ='\0';
    }

    /*线程号*/
//    if( !(log->flags&LOG_NOTID) ) {
 //       sprintf(threadnum, "(%lu) ", pthread_self());
 //   }

    cnt = snprintf(line, sizeof(line), "%s%s%s",
                   log.flags&LOG_NODATE ? "" : date,
                   log.flags&LOG_NOLVL  ? "" : 
                       (level > FATAL ? levels[0] : levels[level]),
                   log.flags&LOG_NOTID  ? "" : threadnum);
	
    fd = log.fd;
    va_start(ap, fmt);
    vsnprintf(line + cnt, sizeof(line) - cnt, fmt, ap);    /*如果输入的日志过长会自动截取*/
    va_end(ap);

    line[sizeof(line)-1] = '\0';

    //  printf("%s",line);
    //   return rc;
	  
    sem_wait(&log.sem);         /*用信号实现同步*/
    rc = write(fd, line, strlen(line));
    sem_post(&log.sem);

    if( !rc ) errno = 0;
    return rc;
}




/* Allows printf()-like interface to file descriptors without the
 * complications that arise from mixing stdio and low level calls 
 * FIXME: Needs date and time before logfile entries.
 */
int lprintf( log_t *log, unsigned int level, char *fmt, ... ) {
    int fd = 0;
    int rc = 0;
	// int size = 0;
    va_list ap;
    // time_t now;
    // char date[50];
    static char line[LOGLINE_MAX];
    // static char threadnum[10];
    int cnt = 0, i = 0;
	
	struct stat buf = {0};  
    // static char *levels[6] = { "[(bad)] ", 
    //                            "[debug] ", 
    //                            "[info ] ", 
    //                            "[warn ] ", 
    //                            "[error] ", 
    //                            "[fatal] " };

    if(stat(log[4].name, &buf)<0)  
    {  
        return 0;  
    }  
		 
   if (buf.st_size > 5000000)//5m
   {
       // printf("size = %d\n", buf.st_size);

   		close(log[4].fd);
		
   		for(i = 0;i<4;i++)
            rename(log[i+1].name, log[i].name);
        
        #if(0 == S600_CONFIG_DIR)
            log[4].fd = open("/mnt/nandflash/log4", O_WRONLY|O_CREAT|O_NOCTTY | (0&LOG_TRUNC ? O_TRUNC : O_APPEND) , 0666);
            strcpy(log[4].name,"/mnt/nandflash/log4");
        #else
            log[4].fd = open(S600log_path, O_WRONLY|O_CREAT|O_NOCTTY|(0&LOG_TRUNC?O_TRUNC:O_APPEND), 0666);
            strcpy(log[4].name, S600log_path);
        #endif   
   }
   
#if 0

    /* Prepare the date string   日期*/
    if( !(log->flags&LOG_NODATE) ) {
        now=time(NULL);
        strcpy(date,ctime(&now));
        date[strlen(date)-6]=' ';
        date[strlen(date)-5]='\0';
    }

    /*线程号*/
//    if( !(log->flags&LOG_NOTID) ) {
 //       sprintf(threadnum, "(%lu) ", pthread_self());
 //   }

    cnt = snprintf(line, sizeof(line), "%s%s%s",
                   log->flags&LOG_NODATE ? "" : date,
                   log->flags&LOG_NOLVL  ? "" : 
                       (level > FATAL ? levels[0] : levels[level]),
                   log->flags&LOG_NOTID  ? "" : threadnum);
#endif

	
    fd=log[4].fd;
    va_start(ap, fmt);
    vsnprintf(line+cnt, sizeof(line)-cnt, fmt, ap);    /*如果输入的日志过长会自动截取*/
    va_end(ap);

    line[sizeof(line)-1] = '\0';

    //  printf("%s",line);
    //   return rc;
	  
    sem_wait(&log[4].sem);         /*用信号实现同步*/
    rc = write(fd, line, strlen(line));
    sem_post(&log[4].sem);

    if( !rc ) errno = 0;
    return rc;
}

log_t *log_open(log_t* log, char *fname, int flags ) {

    printf("in open\n");
	
    log->flags=flags;	
    {
        log->fd = open(fname, O_WRONLY | O_CREAT | O_NOCTTY | 
                (flags & LOG_TRUNC ? O_TRUNC : O_APPEND), 0666);
    }
    if( log->fd == -1 ) {
        fprintf(stderr, "log_open: Opening logfile %s: %s", 
                fname, strerror(errno));
        goto log_open_b;
    }
    if( sem_init(&log->sem, 0, 1) == -1 ) {
        fprintf(stderr, "log_open: Could not initialize log semaphore.");
        goto log_open_c;
    }

	strcpy(log->name, fname);
    return log;

log_open_c:
    close(log->fd);
log_open_b:

    // log_open_a:
    return NULL;
}

void log_close( log_t *log ) {
    
    printf("in close\n");

    sem_wait(&log->sem);
    sem_destroy(&log->sem);
    close(log->fd);
  //  free(log);
    return;
}


