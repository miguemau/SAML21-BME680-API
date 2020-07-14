//***************************************************************************
//
// Title                :"sys_support.h "
// Date                 :4/25/2020
// Version              : 1.0
// Target MCU           :saml21j18b
// Target Hardware      ;
// Author               :Ramez Kaupak && Miguel Rivas
/* DESCRIPTION: SPrintf and printf enhancement.
Has necessary function and
variables declaration for lcd display
*/
// Revision History     : Initial version
//
//
//**************************************************************************

#ifndef SYS_SUPPORT_H_
#define SYS_SUPPORT_H_

#include <stdio.h>

extern int _write(FILE *f, char *buf, int n);
extern int _read(FILE *f, char *buf, int n);
extern int _close(FILE *f);
extern int _fstat(FILE *f, void *p);
extern int _isatty(FILE *f);
extern int _lseek(FILE *f, int o, int w);
extern void* _sbrk(int i);

#endif /* SYS_SUPPORT_H_ */