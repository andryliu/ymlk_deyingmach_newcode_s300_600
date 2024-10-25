/* -------------------------------------------------------------------------
 * util.c 
 * -------------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>
#include <netdb.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>

/*#undef __EI*/
#include "util.h"
#include "log.h"
#include "common.h"

char *chomp( char *str )
{
    int i=strlen(str)-1;
    while( i >= 0 ){
        if( isspace((int)str[i]) ) str[i--]='\0'; else break;
    }
    return str;
}

inline int xstrncasecmp( const char *s1, const char *s2, int n ){
    const char *p1=s1, *p2=s2;
    int ctr=0;

    if( s1 == s2 ) return 0;

    while( *p1 && tolower(*p1) == tolower(*p2) ) {
        if( n ) {
            ctr++;
            if( ctr >= n ) return 0;
        }
        p1++; p2++;
    }
    if( !*p1 && *p2 ) return -1;
    else if( !*p2 && *p1 ) return 1;
    else return tolower(*p1)-tolower(*p2);
}

int fdprintf(int fd, char *fmt, ...) {
    FILE *fp = fdopen(dup(fd), "w");  /*复制文件描述符*/
    int rc;
    va_list ap;
 
    va_start(ap, fmt);
    rc=vfprintf(fp, fmt, ap);
    fflush(fp);
    fclose(fp);
    va_end(ap);
    return rc;
}


/*从指定的文件描述符中读取一行内容，如果行内容超过传入的缓冲则记录日志错误信息*/
char *recvline( char *buf, int len, int fd ){
    char c=0;
    int ctr=0;

    while( ctr < len-2 ){
        if( c != '\n' && read(fd,&c,1) == 1 ){
            buf[ctr]=c;
            ctr++;
        } else {
            break;
        }
    }
    if( ctr == len-2 ){
        while( c != '\n' && read(fd,&c,1) == 1 ) ctr++;
        lprintf( log_my, WARN,
                "recvline: line exceeded buffer space by %d bytes.",
                ctr-len);
    }
    buf[ctr]='\0';

    if( *buf == '\0' ) return NULL;
    else return buf;
}

int recvflush( int s ) {
    fd_set fds;
    char c=0;
    int rc, cnt=0;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(s, &fds);
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    while( (rc=select(s+1, &fds, NULL, NULL, &tv)) ) {
        if( rc == -1 ) {
            lprintf(log_my, WARN, "recvflush: select() failed: %s",
                    strerror(errno));
            break;
        }
        if( (rc=read(s, &c, 1)) < 1 ) {
            if( rc == 0 ) {
                lprintf(log_my, WARN, "recvflush: connection reset by peer");
            } else {
                lprintf(log_my, WARN, "recvflush: read() failed: %s",
                        strerror(errno));
            }
            break;
        }
        cnt++;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
    }

    dprintf(log_my, DEBUG, "recvflush: returning %d.", cnt);
    return cnt;
}

/* 
 * Reads exactly len bytes from fd and returns the data in a dynamically
 * allocated buffer
 */
char *readloop( int fd, size_t len ) {
    size_t cnt=0;
    int rc;
    char *buf = malloc(len+1);

    if(!buf) return NULL;

    dprintf(log_my, DEBUG, "readloop: attempting to read %d bytes from fd #%d",
            len, fd);
    while( cnt < len ) {
        if( (rc=read(fd,buf+cnt,len-cnt)) <= 0 ) {
            if( rc < 0 ) {
                if( errno == EINTR ) continue;
                lprintf(log_my, WARN, "Reading from sock fd %d: %s.", fd,
                        strerror(errno));
            } else {
                lprintf(log_my, WARN, 
                        "Reading from sock fd %d: Read %d bytes, expected %d",
                        fd, cnt, len);
            }
            free(buf);
            return NULL;
        }
        dprintf(log_my, DEBUG, "readloop: read() returned %d.", rc);
        cnt += rc;
    }
    buf[len] = '\0';

    return buf;
}

char **splitlines( char *buf ) {
    int cnt = 1;
    char **tmp, **ret = NULL;
    char *cp;

    if( !buf || !*buf ) return NULL;
    if( (ret=malloc(1 + cnt * sizeof(char**))) == NULL ) return NULL;
    ret[0] = buf;
    
    for( cp=buf; *cp; cp++ ) {
        if( *cp != '\n' ) continue;
        if( (tmp=realloc(ret, (++cnt + 1) * sizeof(char**))) == NULL ) {
            free(ret);
            return NULL;
        }
        ret=tmp;

        *cp = '\0';
        ret[cnt-1] = cp + 1;
    }

    ret[cnt] = NULL;
    return ret;
}

/* Thread-safe resolve */
/*
struct hostent * resolve( const char *name, struct hostent *hostbuf, char *buf, size_t len ) 
{
    struct hostent *hp;
    int herr, rc=0, i;

    for( i=0; i<3; i++ ){
        rc=gethostbyname_r(name, hostbuf, buf, len,&hp, &herr);
        if( !rc ){
            return hp;
        } else if( herr == TRY_AGAIN ){
            continue;
        } else {
            break;
        }
    }
    errno=rc;
    return NULL;
}

*/
