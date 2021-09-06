/*
******************************************************************************
File:     syscalls.c
Info:     Generated by Atollic TrueSTUDIO(R) 9.3.0   2021-06-18

The MIT License (MIT)
Copyright (c) 2019 STMicroelectronics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

******************************************************************************
*/

/* Includes */
#include <stdint.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include "stm32f4xx.h"


/* Variables */
#undef errno
extern int32_t errno;

uint8_t *__env[1] = { 0 };
uint8_t **environ = __env;


/* Functions */
void initialise_monitor_handles()
{
}

int _getpid(void)
{
	errno = ENOSYS;
	return -1;
}

int _gettimeofday(struct timeval  *ptimeval, void *ptimezone)
{
  errno = ENOSYS;
  return -1;
}

int _kill(int32_t pid, int32_t sig)
{
	errno = ENOSYS;
	return -1;
}

void _exit(int32_t status)
{
	while (1) {}		/* Make sure we hang here */
}

int _write(int32_t file, uint8_t *ptr, int32_t len)
{
	/* Implement your write code here, this is used by puts and printf for example */
	/* return len; */
	
	int i;

	for(i=0; i<len; i++)
		ITM_SendChar(*ptr++);

	return len;

	errno = ENOSYS;
	return -1;
}

void * _sbrk(int32_t incr)
{
	extern char   end; /* Set by linker.  */
	static char * heap_end;
	char *        prev_heap_end;

	if (heap_end == 0) {
		heap_end = & end;
	}

	prev_heap_end = heap_end;
	heap_end += incr;

	return (void *) prev_heap_end;
}

int _close(int32_t file)
{
	errno = ENOSYS;
	return -1;
}


int _fstat(int32_t file, struct stat *st)
{
	errno = ENOSYS;
	return -1;
}

int _isatty(int32_t file)
{
	errno = ENOSYS;
	return 0;
}

int _lseek(int32_t file, int32_t ptr, int32_t dir)
{
	errno = ENOSYS;
	return -1;
}

int _read(int32_t file, uint8_t *ptr, int32_t len)
{
	errno = ENOSYS;
	return -1;
}

int _readlink(const char *path, char *buf, size_t bufsize)
{
  errno = ENOSYS;
  return -1;
}

int _open(const uint8_t *path, int32_t flags, int32_t mode)
{
	errno = ENOSYS;
	return -1;
}

int _wait(int32_t *status)
{
	errno = ENOSYS;
	return -1;
}

int _unlink(const uint8_t *name)
{
	errno = ENOSYS;
	return -1;
}

int _times(struct tms *buf)
{
	errno = ENOSYS;
	return -1;
}

int _stat(const uint8_t *file, struct stat *st)
{
	errno = ENOSYS;
	return -1;
}

int _symlink(const char *path1, const char *path2)
{
  errno = ENOSYS;
  return -1;
}

int _link(const uint8_t *old, const uint8_t *new)
{
	errno = ENOSYS;
	return -1;
}

int _fork(void)
{
	errno = ENOSYS;
	return -1;
}

int _execve(const uint8_t *name, uint8_t * const *argv, uint8_t * const *env)
{
	errno = ENOSYS;
	return -1;
}

