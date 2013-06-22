/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <URT.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/unistd.h>

int _close(int file)
{
	return -1;
}

int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	switch(file) {
		case STDOUT_FILENO:
		case STDERR_FILENO:
		case STDIN_FILENO:
			return 1;
		default:
			errno = EBADF;
			return 0;
	}
}

int _lseek(int file, int ptr, int dir)
{
	return 0;
}

int _read(int file, char *ptr, int len)
{
	return 0;
}

int _write(int file, char *ptr, int len)
{
	int n;
	switch(file) {
		case STDOUT_FILENO:
		case STDERR_FILENO:
			for(n = 0; n < len; n++) {
				URT_PutChar(*ptr++);
			}
			break;
		default:
			errno = EBADF;
			return -1;
	}
	return len;
}
/* [] END OF FILE */
