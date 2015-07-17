/*
 * This file is derived from GCC <stdarg.h> header.
 */

/* Copyright (C) 1989, 1997, 1998, 1999, 2000 Free Software Foundation, Inc.

This file is part of GCC.

GCC is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2, or (at your option)
any later version.

GCC is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with GCC; see the file COPYING.  If not, write to
the Free Software Foundation, 51 Franklin Street, Fifth Floor,
Boston, MA 02110-1301, USA.  */

/* As a special exception, if you include this header file into source
   files compiled by GCC, this header file does not by itself cause
   the resulting executable to be covered by the GNU General Public
   License.  This exception does not however invalidate any other
   reasons why the executable file might be covered by the GNU General
   Public License.  */

/**
 * @file
 * @brief C standard library header.
 * @details Allows functions to accept an indefinite number of arguments.
 *
 * @date 13.02.10
 * @author Eldar Abusalimov
 */

#ifndef STDARG_H_
#define STDARG_H_

/* We assume GCC is used. At least for now. -- Eldar */
#ifdef __GNUC__

typedef __builtin_va_list va_list;

#define va_start(v,l) __builtin_va_start(v,l)
#define va_end(v)     __builtin_va_end(v)
#define va_arg(v,l)   __builtin_va_arg(v,l)
#define va_copy(d,s)  __builtin_va_copy(d,s)

#else /* __GNUC__ */

//#define va_list char *
typedef struct
{
 void *stack_ptr;
} __va_list;

typedef __va_list* va_list;

#define __VA_ALIGNED_SIZE(x) ((sizeof(x) + sizeof(void*) - 1) & ~(sizeof(void*) - 1))

#define va_start(ap, last) __va_list __va_list__ = { 0 }; __va_list__.stack_ptr = (void *)(((char *)&(last)) + __VA_ALIGNED_SIZE(last)); (ap) = &__va_list__
#define va_end(ap) ((void*)0)

#define va_arg(ap, type) \
	( ((ap)->stack_ptr = (va_list)((char *)(ap)->stack_ptr + __VA_ALIGNED_SIZE(type))), \
	*(type *)(void *)((char *)(ap)->stack_ptr - __VA_ALIGNED_SIZE(type)) )


#endif /* __GNUC__ */

#endif /* STDARG_H_ */
