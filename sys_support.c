/*
* sys_support.c
*
* Created: 4/13/2020 2:43:44 PM
*  Author: kshort
*/

#include "saml21j18b.h"
#include <stdio.h>

int _write(FILE *f, char *buf, int n);
int _read(FILE *f, char *buf, int n);
int _close(FILE *f);
int _fstat(FILE *f, void *p);
int _isatty(FILE *f);
int _lseek(FILE *f, int o, int w);
void* _sbrk(int i);

static int n = 0;
static char str[80];

int _write(FILE *f, char *buf, int n) {
	int m = n;
	for (; n > 0; n--)
	UART4_write(*buf++);
	return m;
}

int _read(FILE *f, char *buf, int n) {
	*buf = UART4_read();
	if (*buf == '\r') {
		*buf = '\n';
		_write(f,"\r", 1);
	}
	_write(f, buf, 1);
	return 1;
}

int _close(FILE *f) {
	return 0;
}

int _fstat(FILE *f, void *p) {
	*((int*)p + 4) = 0x81b6; // enable read/write
	return 0;
}

int _isatty(FILE *f) {
	return 1;
}

int _lseek(FILE *f, int o, int w) {
	return 0;
}

void* _sbrk(int i) {
	return (void*)0x20006000;
}

/* _fstat() returns with permisson 666, that might not make a difference.
* _sbrk() returns an address of RAM that might be free.
* When scanf() is called, _read() gets a size of 400.
* When prinft() is called, an extra '\n' at the end of format is required
* for some reason, otherwise the last char is truncated.
*/