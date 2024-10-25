/* -------------------------------------------------------------------------
 * common.c 
 * -------------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <string.h>
/*#include <sys/socket.h>*/
#include <sys/types.h>
#include <sys/stat.h>
/*#include <netinet/in.h>*/
#include <arpa/inet.h>
#include <pwd.h>


#include "log.h"
#include "common.h"


char *signames[64];


/* Become a daemon: fork, die, setsid, fork, die, disconnect */
void daemonize( void ) {
   pid_t pid ;

   fprintf( stderr, "HTun daemon backgrounding.\n" );
   pid = fork();
   if( pid < 0 ) lprintf( log_my, ERROR, "Unable to fork()!" );
   if( pid > 0 ) _exit(0) ;   /* parent exits */

   setsid();

   pid = fork() ;
   if( pid < 0 ) lprintf( log_my, ERROR, "Unable to fork()!" );
   if( pid > 0 ) _exit(0);   /* parent exits */

   chdir("~");
   freopen("/dev/null","r",stdin);
   freopen("/dev/null","w",stdout);
   freopen("/dev/null","w",stderr);
  
}

void dropprivs(char *str) {
    struct passwd *nonpriv = getpwnam("nobody");

    if( !nonpriv ) return;

    setreuid(-1, nonpriv->pw_uid);
    setregid(-1, nonpriv->pw_gid);
    if( str && *str ) 
        lprintf( log_my, INFO, "Dropped privs to 'nobody' (%s)", str );
    return;
}

void getprivs(char *str) {
    int rc = setreuid(0,0) | setregid(0,0);
    if( rc == -1 ) {
        lprintf( log_my, ERROR, "Unable to gain superuser privileges: %s",
                strerror(errno) );
    } else {
        if( str && *str ) 
            lprintf( log_my, INFO, "Got superuser privleges (%s)", str );
    }
    return;
}
/*
void init_signames(void) {
    signames[SIGHUP] = "SIGHUP";
    signames[SIGINT] = "SIGINT";
    signames[SIGQUIT] = "SIGQUIT";
    signames[SIGILL] = "SIGILL";
    signames[SIGTRAP] = "SIGTRAP";
    signames[SIGABRT] = "SIGABRT";
    signames[SIGIOT] = "SIGIOT";
    signames[SIGBUS] = "SIGBUS";
    signames[SIGFPE] = "SIGFPE";
    signames[SIGKILL] = "SIGKILL";
    signames[SIGUSR1] = "SIGUSR1";
    signames[SIGSEGV] = "SIGSEGV";
    signames[SIGUSR2] = "SIGUSR2";
    signames[SIGPIPE] = "SIGPIPE";
    signames[SIGALRM] = "SIGALRM";
    signames[SIGTERM] = "SIGTERM";
    signames[SIGSTKFLT] = "SIGSTKFLT";
    signames[SIGCLD] = "SIGCLD";
    signames[SIGCHLD] = "SIGCHLD";
    signames[SIGCONT] = "SIGCONT";
    signames[SIGSTOP] = "SIGSTOP";
    signames[SIGTSTP] = "SIGTSTP";
    signames[SIGTTIN] = "SIGTTIN";
    signames[SIGTTOU] = "SIGTTOU";
    signames[SIGURG] = "SIGURG";
    signames[SIGXCPU] = "SIGXCPU";
    signames[SIGXFSZ] = "SIGXFSZ";
    signames[SIGVTALRM] = "SIGVTALRM";
    signames[SIGPROF] = "SIGPROF";
    signames[SIGWINCH] = "SIGWINCH";
    signames[SIGPOLL] = "SIGPOLL";
    signames[SIGIO] = "SIGIO";
    signames[SIGPWR] = "SIGPWR";
    signames[SIGSYS] = "SIGSYS";
    signames[SIGUNUSED] = "SIGUNUSED";

    return;
}

*/