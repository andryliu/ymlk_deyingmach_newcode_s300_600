/* -------------------------------------------------------------------------
 * util.h 
 * -------------------------------------------------------------------------
 */

#ifndef __UTIL_H
#define __UTIL_H

#include <stdio.h>
#include <sys/types.h>
#include <stdarg.h>
#include <ctype.h>

/*#undef __EI
#define __EI extern __inline__
*/
/*
 * Provides a strncasecmp for non-BSD C libraries
 */
inline int xstrncasecmp( const char *s1, const char *s2, int n );

/*__EI*/
int xstrcasecmp( const char *s1, const char *s2 ) {
    return xstrncasecmp( s1, s2, 0 );
}

/* 
 * fprintf()-like function but uses a file descriptor or socket descriptor
 * instead of a FILE*.
 */
int fdprintf(int fd, char *fmt, ...);

/*
 * receives a line of data from fd, and puts up to len bytes into buf.
 * Returns NULL if there was no data, or buf on success.
 */
char *recvline( char *buf, int len, int fd );

/*
 * Ensures there is no extra data waiting to be received on fd.  If there is,
 * it is discarded, and the number of discarded bytes is returned.
 */
int recvflush( int fd ); 

/* 
 * Reads exactly len bytes from fd and returns the data in a dynamically
 * allocated buffer
 */
char *readloop( int fd, size_t len );

/*
 * Takes in a char * and returns a dynamically allocated array of pointers to
 * the start of each line. The pointer after the last is set to NULL to
 * indicate that there are no more pointers after it. Every occurance of '\n'
 * in the input string is CHANGED to '\0' to ease the reading of the strings.
 */
char **splitlines( char *buf );

/*
 * Removes any trailing whitespace from str by moving up the null pointer.
 */
//__EI
char *chomp( char *str );

/*
 * Pass in a hostname, a pointer to a user supplied hostent struct, a
 * sufficiently-sized buffer for user by gethostbyname_r(), and the length of
 * the buffer, and resolve() fills in the hostent struct for you, returning a
 * pointer to it on success, or NULL on failure.
 */
struct hostent *resolve( const char *name, struct hostent *hostbuf, char *buf, size_t len );

#endif

