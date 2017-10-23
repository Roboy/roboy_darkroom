/*
 * Copyright (c) 2004-2005 The Trustees of Indiana University and Indiana
 *                         University Research and Technology
 *                         Corporation.  All rights reserved.
 * Copyright (c) 2004-2005 The University of Tennessee and The University
 *                         of Tennessee Research Foundation.  All rights
 *                         reserved.
 * Copyright (c) 2004-2005 High Performance Computing Center Stuttgart, 
 *                         University of Stuttgart.  All rights reserved.
 * Copyright (c) 2004-2005 The Regents of the University of California.
 *                         All rights reserved.
 * Copyright (c) 2010      IBM Corporation.  All rights reserved.
 * Copyright (c) 2010      ARM ltd.  All rights reserved.
 * Copyright (c) 2013      Linaro ltd.  All rights reserved.
 * $COPYRIGHT$
 *
 * Additional copyrights may follow
 *
 * $HEADER$
 */

#ifndef OMPI_SYS_ARCH_ATOMIC_H
#define OMPI_SYS_ARCH_ATOMIC_H 1

#if OPAL_WANT_SMP_LOCKS

#define OPAL_HAVE_ATOMIC_MEM_BARRIER 1

#define MB()  __asm__ __volatile__ ("dmb ish" : : : "memory")
#define RMB() __asm__ __volatile__ ("dmb ish" : : : "memory")
#define WMB() __asm__ __volatile__ ("dmb ish" : : : "memory")

#else

#define MB()
#define RMB()
#define WMB()

#endif /* OPAL_WANT_SMP_LOCKS */



/**********************************************************************
 *
 * Memory Barriers
 *
 *********************************************************************/

#if (OPAL_HAVE_ATOMIC_MEM_BARRIER == 1)

static inline
void opal_atomic_mb(void)
{
    MB();
}


static inline
void opal_atomic_rmb(void)
{
    RMB();
}


static inline
void opal_atomic_wmb(void)
{
    WMB();
}

#endif


/**********************************************************************
 *
 * Atomic math operations
 *
 *********************************************************************/

#define OPAL_HAVE_ATOMIC_CMPSET_32 1
static inline int opal_atomic_cmpset_32(volatile int32_t *addr,
                                        int32_t oldval, int32_t newval)
{
  int32_t ret;
  int32_t tmp;

   __asm__ __volatile__ (
                         "1:  ldxr   %w0, [%2]        \n"
                         "    cmp    %w0, %w3          \n"
                         "    bne    2f              \n"
                         "    stxr   %w1, %w4, [%2]    \n"
                         "    cmp    %w1, #0          \n"
                         "    bne    1b              \n"
                         "2:                          \n"

                         : "=&r" (ret), "=&r" (tmp)
                         : "r" (addr), "r" (oldval), "r" (newval)
                         : "cc", "memory");

   return (ret == oldval);
}

static inline int opal_atomic_cmpset_acq_32(volatile int32_t *addr,
                                            int32_t oldval, int32_t newval)
{
    int rc;

    rc = opal_atomic_cmpset_32(addr, oldval, newval);
    opal_atomic_rmb();

    return rc;
}


static inline int opal_atomic_cmpset_rel_32(volatile int32_t *addr,
                                            int32_t oldval, int32_t newval)
{
    opal_atomic_wmb();
    return opal_atomic_cmpset_32(addr, oldval, newval);
}

#define OPAL_HAVE_ATOMIC_CMPSET_64 1
static inline int opal_atomic_cmpset_64(volatile int64_t *addr,
                                        int64_t oldval, int64_t newval)
{
  int64_t ret;
  int32_t tmp;

   __asm__ __volatile__ (
                         "1:  ldxr   %0, [%2]        \n"
                         "    cmp    %0, %3          \n"
                         "    bne    2f              \n"
                         "    stxr   %w1, %4, [%2]   \n"
                         "    cmp    %w1, #0         \n"
                         "    bne    1b              \n"
                         "2:                         \n"

                         : "=&r" (ret), "=&r" (tmp)
                         : "r" (addr), "r" (oldval), "r" (newval)
                         : "cc", "memory");

   return (ret == oldval);
}

/* these two functions aren't inlined in the non-gcc case because then
   there would be two function calls (since neither cmpset_64 nor
   atomic_?mb can be inlined).  Instead, we "inline" them by hand in
   the assembly, meaning there is one function call overhead instead
   of two */
static inline int opal_atomic_cmpset_acq_64(volatile int64_t *addr,
                                            int64_t oldval, int64_t newval)
{
    int rc;

    rc = opal_atomic_cmpset_64(addr, oldval, newval);
    opal_atomic_rmb();

    return rc;
}


static inline int opal_atomic_cmpset_rel_64(volatile int64_t *addr,
                                            int64_t oldval, int64_t newval)
{
    opal_atomic_wmb();
    return opal_atomic_cmpset_64(addr, oldval, newval);
}

#endif /* ! OMPI_SYS_ARCH_ATOMIC_H */
