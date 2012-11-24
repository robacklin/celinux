#ifndef SC_MATH
#define SC_MATH
#define MATH_STR(X) #X
#define MATH_NAME(X) X

/*
 * Pre scaling defines
 */
#define SC_32(x) ((long long)x<<32)
#define SC_n(n,x) (((long long)x)<<n)
/*
 * This routine preforms the following calculation:
 *
 * X = (a*b)>>32
 * we could, (but don't) also get the part shifted out.
 */
extern inline long mpy_ex32(long a,long b)
{
        long edx;
	__asm__("imull %2"
		:"=a" (a), "=d" (edx)
		:"rm" (b),
		 "0" (a));
        return edx;
}
/*
 * X = (a/b)<<32 or more precisely x = (a<<32)/b
 */

extern inline long div_ex32(long a, long b)
{
        long dum;
        __asm__("divl %2"
                :"=a" (b), "=d" (dum)
                :"r" (b), "0" (0), "1" (a));
        
        return b;
}
/*
 * X = (a*b)>>24
 * we could, (but don't) also get the part shifted out.
 */

#define mpy_ex24(a,b) mpy_sc_n(24,a,b)
/*
 * X = (a/b)<<24 or more precisely x = (a<<24)/b
 */
#define div_ex24(a,b) div_sc_n(24,a,b)

/*
 * The routines allow you to do x = (a/b) << N and
 * x=(a*b)>>N for values of N from 1 to 32.
 *
 * These are handy to have to do scaled math.
 * Scaled math has two nice features:
 * A.) A great deal more precision can be maintained by
 *     keeping more signifigant bits.
 * B.) Often an in line div can be repaced with a mpy
 *     which is a LOT faster.
 */

#define mpy_sc_n(N,aa,bb) ({long edx,a=aa,b=bb; \
	__asm__("imull %2\n\t" \
                "shldl $(32-"MATH_STR(N)"),%0,%1"    \
		:"=a" (a), "=d" (edx)\
		:"rm" (b),            \
		 "0" (a)); edx;})


#define div_sc_n(N,aa,bb) ({long dum=aa,dum2,b=bb; \
        __asm__("shrdl $(32-"MATH_STR(N)"),%4,%3\n\t"  \
                "sarl $(32-"MATH_STR(N)"),%4\n\t"      \
                "divl %2"              \
                :"=a" (dum2), "=d" (dum)      \
                :"rm" (b), "0" (0), "1" (dum)); dum2;})  

  
/*
 * (long)X = ((long long)divs) / (long)div
 * (long)rem = ((long long)divs) % (long)div
 *
 * Warning, this will do an exception if X overflows.
 */
#define div_long_long_rem(a,b,c) div_ll_X_l_rem(a,b,c)

extern inline long div_ll_X_l_rem(long long divs, long div,long * rem)
{
        long dum2;
        __asm__( "divl %2"
                :"=a" (dum2), "=d" (*rem)
                :"rm" (div), "A" (divs));
        
        return dum2;

}
/*
 * same as above, but no remainder
 */
extern inline long div_ll_X_l(long long divs, long div)
{
        long dum;
        return div_ll_X_l_rem(divs,div,&dum);
}
/*
 * (long)X = (((long)divh<<32) | (long)divl) / (long)div
 * (long)rem = (((long)divh<<32) % (long)divl) / (long)div
 *
 * Warning, this will do an exception if X overflows.
 */
extern inline long div_h_or_l_X_l_rem(long divh,long divl, long div,long* rem)
{
        long dum2;
        __asm__( "divl %2"
                :"=a" (dum2), "=d" (*rem)
                :"rm" (div), "0" (divl),"1" (divh));
        
        return dum2;

}
extern inline long long mpy_l_X_l_ll(long mpy1,long mpy2)
{
        long long eax;
	__asm__("imull %1\n\t"
		:"=A" (eax)
		:"rm" (mpy2),
		 "a" (mpy1));
        
        return eax;

}
extern inline long  mpy_1_X_1_h(long mpy1,long mpy2,long *hi)
{
        long eax;
	__asm__("imull %2\n\t"
		:"=a" (eax),"=d" (*hi)
		:"rm" (mpy2),
		 "0" (mpy1));
        
        return eax;

}

#endif
