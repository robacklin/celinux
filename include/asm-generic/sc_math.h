#ifndef SC_MATH_GENERIC
#define SC_MATH_GENERIC
/*
 * Copyright 2003 MontaVista Software.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
/*
 * These are the generic scaling functions for machines which
 * do not yet have the arch asm versions (or for the 64-bit
 * long systems)
 */
/*
 * Pre scaling defines
 */
#define SC_32(x) ((unsigned long long)(x) << 32)
// #define SC_n(n,x) (((unsigned long long)(x)) << (n))
#define SC_n(n,x) (((long long)(x))<<(n))

#if (BITS_PER_LONG < 64)

#define SCC_SHIFT 16
#define SCC_MASK ((1 << SCC_SHIFT) -1)
/*
 * mpy a long by a long and return a long long
 */

extern inline long long mpy_l_X_l_ll(unsigned long mpy1,unsigned long mpy2)
{
	return mpy1 * mpy2;
}
/*
 * mpy a long long by a long and return a long long
 */

extern inline long long mpy_ll_X_l_ll(unsigned long long mpy1,
				      unsigned long mpy2)
{
	/*
	 * Actually gcc can do this, but this seems to be a few
	 * instructions shorter.
	 */
	long long result = mpy_l_X_l_ll((unsigned long)mpy1, mpy2);
	result +=  (mpy_l_X_l_ll((long)(mpy1 >> 32), mpy2) << 32);
	return result;
}
/*
 * mpy a long by a long and return the low part and a seperate hi part
 */


extern inline unsigned long  mpy_l_X_l_h(unsigned long mpy1,
				unsigned long mpy2,
				unsigned long *hi)
{
	unsigned long long it = mpy_l_X_l_ll(mpy1, mpy2);
	*hi = (unsigned long)(it >> 32);
        return (unsigned long)it;

}
/*
 * This routine preforms the following calculation:
 *
 * X = (a*b)>>32
 * we could, (but don't) also get the part shifted out.
 */
extern inline unsigned long mpy_sc32(unsigned long a,unsigned long b)
{
        return (unsigned long)(mpy_l_X_l_ll(a, b) >> 32);
}
/*
 * X = (a/b)<<32 or more precisely x = (a<<32)/b
 */
#include <asm/div64.h>
#if 0  // maybe one day we will do signed numbers...
/*
 * do_div doesn't handle signed numbers, so:
 */
#define do_div_signed(result, div)					\
({									\
        long rem, flip = 0;						\
	if (result < 0){						\
		result = -result;					\
		flip = 2;                 /* flip rem & result sign*/	\
		if (div < 0){						\
			div = -div;					\
			flip--;          /* oops, just flip rem */	\
		}							\
	}								\
	rem = do_div(result,div);					\
	rem = flip ? -rem : rem;					\
	if ( flip == 2)							\
		result = -result;					\
	rem;								\
})
#endif

extern inline unsigned long div_sc32(unsigned long a, unsigned long b)
{
	unsigned long long result = SC_32(a);
	do_div(result, b);
        return (unsigned long)result;
}
/*
 * X = (a*b)>>24
 * we could, (but don't) also get the part shifted out.
 */

#define mpy_sc24(a,b) mpy_sc_n(24,(a),(b))
/*
 * X = (a/b)<<24 or more precisely x = (a<<24)/b
 */
#define div_sc24(a,b) div_sc_n(24,(a),(b))

/*
 * The routines allow you to do x = ((a<< N)/b) and
 * x=(a*b)>>N for values of N from 1 to 32.
 *
 * These are handy to have to do scaled math.
 * Scaled math has two nice features:
 * A.) A great deal more precision can be maintained by
 *     keeping more signifigant bits.
 * B.) Often an in line div can be replaced with a mpy
 *     which is a LOT faster.
 */

/* x = (aa * bb) >> N */


#define mpy_sc_n(N,aa,bb) ({(unsigned long)(mpy_l_X_l_ll((aa), (bb)) >> (N));})

/* x = (aa << N / bb)  */
#define div_sc_n(N,aa,bb) ({unsigned long long result = SC_n((N), (aa)); \
                            do_div(result, (bb)); \
                            (long)result;})  

  
/*
 * (long)X = ((long long)divs) / (long)div
 * (long)rem = ((long long)divs) % (long)div
 *
 * Warning, this will do an exception if X overflows.
 * Well, it would if done in asm, this code just truncates..
 */
#define div_long_long_rem(a,b,c) div_ll_X_l_rem((a),(b),(c))

/* x = divs / div; *rem = divs % div; */
extern inline unsigned long div_ll_X_l_rem(unsigned long long divs, 
					   unsigned long div, 
					   unsigned long * rem)
{
	unsigned long long it = divs;
	*rem = do_div(it, div);
	return (unsigned long)it;
}
/*
 * same as above, but no remainder
 */
extern inline unsigned long div_ll_X_l(unsigned long long divs, 
				       unsigned long div)
{
	unsigned long long it = divs;
        do_div(it, div);
        return (unsigned long)it;
}
/*
 * (long)X = (((long)divh<<32) | (long)divl) / (long)div
 * (long)rem = (((long)divh<<32) % (long)divl) / (long)div
 *
 * Warning, this will do an exception if X overflows.
 * Well, it would if done in asm, this code just truncates..
 */
extern inline unsigned long div_h_or_l_X_l_rem(unsigned long divh,
					       unsigned long divl, 
					       unsigned long div,
					       unsigned long* rem)
{
	unsigned long long result = SC_32(divh) + (divl);

        return div_ll_X_l_rem(result, (div), (rem));

}
#else
/* The 64-bit long version */

/* 
 * The 64-bit long machine can do most of these in native C.  We assume that 
 * the "long long" of 32-bit machines is typedefed away so the we need only
 * deal with longs.  This code should be tight enought that asm code is not
 * needed.
 */

/*
 * mpy a long by a long and return a long
 */

extern inline unsigned long mpy_l_X_l_ll(unsigned long mpy1, unsigned long mpy2)
{

        return (mpy1) * (mpy2);

}
/*
 * mpy a long by a long and return the low part and a separate hi part
 * This code always returns 32 values... may not be what you want...
 */


extern inline unsigned long  mpy_l_X_l_h(unsigned long mpy1,
					 unsigned long mpy2,
					 unsigned long *hi)
{
	unsigned long it = mpy1 * mpy2;
	*hi = (it >> 32);
        return it & 0xffffffff;

}
/*
 * This routine preforms the following calculation:
 *
 * X = (a*b)>>32
 * we could, (but don't) also get the part shifted out.
 */
extern inline unsigned long mpy_sc32(unsigned long a, unsigned long b)
{
        return (mpy1 * mpy2) >> 32);
}
/*
 * X = (a/b)<<32 or more precisely x = (a<<32)/b
 */

extern inline long div_sc32(long a, long b)
{
	return  SC_32(a) / (b);
}
/*
 * X = (a*b)>>24
 * we could, (but don't) also get the part shifted out.
 */

#define mpy_sc24(a,b) mpy_sc_n(24,a,b)
/*
 * X = (a/b)<<24 or more precisely x = (a<<24)/b
 */
#define div_sc24(a,b) div_sc_n(24,a,b)

/*
 * The routines allow you to do x = ((a<< N)/b) and
 * x=(a*b)>>N for values of N from 1 to 32.
 *
 * These are handy to have to do scaled math.
 * Scaled math has two nice features:
 * A.) A great deal more precision can be maintained by
 *     keeping more signifigant bits.
 * B.) Often an in line div can be replaced with a mpy
 *     which is a LOT faster.
 */

/* x = (aa * bb) >> N */


#define mpy_sc_n(N,aa,bb) ((aa) * (bb)) >> N)

/* x = (aa << N / bb)  */
#define div_sc_n(N,aa,bb) (SC_n((N), (aa)) / (bb))

  
/*
 * (long)X = ((long long)divs) / (long)div
 * (long)rem = ((long long)divs) % (long)div
 *
 * Warning, this will do an exception if X overflows.
 * Well, it would if done in asm, this code just truncates..
 */
#define div_long_long_rem(a,b,c) div_ll_X_l_rem(a, b, c)

/* x = divs / div; *rem = divs % div; */
extern inline unsigned long div_ll_X_l_rem(unsigned long divs, 
					   unsigned long div, 
					   unsigned long * rem)
{
	*rem = divs % div;
	return divs / div;
}
/*
 * same as above, but no remainder
 */
extern inline unsigned long div_ll_X_l(unsigned long divs, 
				       unsigned long div)
{
        return divs / div;
}
/*
 * (long)X = (((long)divh<<32) | (long)divl) / (long)div
 * (long)rem = (((long)divh<<32) % (long)divl) / (long)div
 *
 * Warning, this will do an exception if X overflows.
 * Well, it would if done in asm, this code just truncates..
 */
extern inline unsigned long div_h_or_l_X_l_rem(unsigned long divh,
					       unsigned long divl, 
					       unsigned long div,
					       unsigned long* rem)
{
	long result = SC_32(divh) + divl;

        return div_ll_X_l_rem(result, div, rem);

}
#endif  // else(BITS_PER_LONG < 64)
#endif
