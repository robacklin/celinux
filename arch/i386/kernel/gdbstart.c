/*
 * This program opens a tty file and issues the GDB stub activating
 * ioctl on it.
 */

#include <sys/types.h>
#include <sys/wait.h>
#include <asm/ioctls.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#define TIOCGDB		0x547F

char		*tty_name = "/dev/ttyS0" ;	/* COM1 port */
int		 speed = 9600 ;			/* default speed */
struct termios	 save_ts ;			/* original term struct */

void print_usage(void)
{
    printf("gdbstub [-s speed] [-t tty-dev]\n") ;
    printf("  defaults:  /dev/ttyS0 with speed unmodified by this program\n");

} /* print_usage */

void tty_err(char *msg)
{
    char	buf[100] ;

    strcpy(buf, msg) ;
    strcat(buf, ": ") ;
    strcat(buf, tty_name) ;
    perror(buf) ;
    exit(1) ;

} /* tty_err */


void setup_term(int fd)
{
    struct termios	ts ;
    int			speed_code ;

    if (tcgetattr(fd, &ts) < 0) tty_err("tcgetattr") ;

    save_ts = ts ;
    switch (speed)
    {
    case 4800:
	speed_code = B4800 ;
	break ;
    case 9600:
	speed_code = B9600 ;
	break ;
    case 19200:
	speed_code = B19200 ;
	break ;
    case 38400:
	speed_code = B38400 ;
	break ;
    case 57600:
	speed_code = B57600 ;
	break ;
    case 115200:
	speed_code = B115200 ;
	break ;
    case 230400:
	speed_code = B230400 ;
	break ;
    default:
	printf("Invalid speed: %d\n", speed) ;
	exit(1) ;
    }

    ts.c_cflag = CS8 | CREAD | CLOCAL ;
    if (cfsetospeed(&ts, speed_code) < 0) tty_err("cfsetospeed") ;
    if (cfsetispeed(&ts, speed_code) < 0) tty_err("cfsetispeed") ;

    if (tcsetattr(fd, TCSANOW, &ts) < 0) tty_err("tcsetattr") ;

} /* setup_term */

void main(int argc, char **argv)
{
    int		opt ;
    int		fil ;
    int		rslt ;

    while ((opt = getopt(argc, argv, "hs:t:")) > 0)
    {
	switch (opt)
	{
	case 's':
	    speed = atol(optarg) ;
	    break ;
	case 't':
	    tty_name = optarg ;
	    break ;
	case ':':
	    printf("Invalid option\n") ;
	    break ;
	case '?':
	case 'h':
	default:
	    print_usage() ;
	    return ;
	}
    }

    fil = open(tty_name, O_RDWR) ;
    if (fil < 0)
    {
	perror(tty_name) ;
	return ;
    }


    setup_term(fil) ;

    /*
     * When we issue this ioctl, control will not return until
     * the debugger running on the remote host machine says "go".
     */
    printf("\nAbout to activate GDB stub in the kernel on %s\n", tty_name) ;
    printf("Hit CR to continue, kill program to abort -- ") ;
    getchar() ;
    sync() ;
    rslt = ioctl(fil, TIOCGDB, 0) ;
    if (rslt < 0)
    {
	perror("TIOCGDB ioctl") ;
	return ;
    }

    printf("\nGDB stub successfully activated\n") ;

    for (;;)
    {
	pause() ;
    }

    if (tcsetattr(fil, TCSANOW, &save_ts) < 0) tty_err("tcsetattr") ;

} /* main */
