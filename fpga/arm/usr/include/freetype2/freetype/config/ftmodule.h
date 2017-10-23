#if defined(__linux__)
# if defined(__x86_64__) && defined(__LP64__)
#  include <x86_64-linux-gnu/freetype2/freetype/config/ftmodule.h>
# elif defined(__x86_64__) && defined(__ILP32__)
#  include <x86_64-linux-gnux32/freetype2/freetype/config/ftmodule.h>
# elif defined(__i386__)
#  include <i386-linux-gnu/freetype2/freetype/config/ftmodule.h>
# elif defined(__aarch64__) && defined(__AARCH64EL__)
#  include <aarch64-linux-gnu/freetype2/freetype/config/ftmodule.h>
# elif defined(__alpha__)
#  include <alpha-linux-gnu/freetype2/freetype/config/ftmodule.h>
# elif defined(__ARM_EABI__) && defined(__ARM_PCS_VFP)
#  include <arm-linux-gnueabihf/freetype2/freetype/config/ftmodule.h>
# elif defined(__ARM_EABI__) && !defined(__ARM_PCS_VFP)
#  include <arm-linux-gnueabi/freetype2/freetype/config/ftmodule.h>
# elif defined(__hppa__)
#  include <hppa-linux-gnu/freetype2/freetype/config/ftmodule.h>
# elif defined(__ia64__)
#  include <ia64-linux-gnu/freetype2/freetype/config/ftmodule.h>
# elif defined(__m68k__) && !defined(__mcoldfire__)
#  include <m68k-linux-gnu/freetype2/freetype/config/ftmodule.h>
# elif defined(__mips_hard_float) && defined(_MIPSEL)
#  if _MIPS_SIM == _ABIO32
#   include <mipsel-linux-gnu/freetype2/freetype/config/ftmodule.h>
#  elif _MIPS_SIM == _ABIN32
#   include <mips64el-linux-gnuabin32/freetype2/freetype/config/ftmodule.h>
#  elif _MIPS_SIM == _ABI64
#   include <mips64el-linux-gnuabi64/freetype2/freetype/config/ftmodule.h>
#  else
#   error unknown multiarch location for ftmodule.h
#  endif
# elif defined(__mips_hard_float)
#  if _MIPS_SIM == _ABIO32
#   include <mips-linux-gnu/freetype2/freetype/config/ftmodule.h>
#  elif _MIPS_SIM == _ABIN32
#   include <mips64-linux-gnuabin32/freetype2/freetype/config/ftmodule.h>
#  elif _MIPS_SIM == _ABI64
#   include <mips64-linux-gnuabi64/freetype2/freetype/config/ftmodule.h>
#  else
#   error unknown multiarch location for ftmodule.h
#  endif
# elif defined(__or1k__)
#  include <or1k-linux-gnu/freetype2/freetype/config/ftmodule.h>
# elif defined(__powerpc__) && defined(__SPE__)
#  include <powerpc-linux-gnuspe/freetype2/freetype/config/ftmodule.h>
# elif defined(__powerpc64__)
#  if defined(__LITTLE_ENDIAN__)
#    include <powerpc64le-linux-gnu/freetype2/freetype/config/ftmodule.h>
#  else
#    include <powerpc64-linux-gnu/freetype2/freetype/config/ftmodule.h>
#  endif
# elif defined(__powerpc__)
#  include <powerpc-linux-gnu/freetype2/freetype/config/ftmodule.h>
# elif defined(__s390x__)
#  include <s390x-linux-gnu/freetype2/freetype/config/ftmodule.h>
# elif defined(__s390__)
#  include <s390-linux-gnu/freetype2/freetype/config/ftmodule.h>
# elif defined(__sh__) && defined(__LITTLE_ENDIAN__)
#  include <sh4-linux-gnu/freetype2/freetype/config/ftmodule.h>
# elif defined(__sparc__) && defined(__arch64__)
#  include <sparc64-linux-gnu/freetype2/freetype/config/ftmodule.h>
# elif defined(__sparc__)
#  include <sparc-linux-gnu/freetype2/freetype/config/ftmodule.h>
# else
#   error unknown multiarch location for ftmodule.h
# endif
#elif defined(__FreeBSD_kernel__)
# if defined(__LP64__)
#  include <x86_64-kfreebsd-gnu/freetype2/freetype/config/ftmodule.h>
# elif defined(__i386__)
#  include <i386-kfreebsd-gnu/freetype2/freetype/config/ftmodule.h>
# else
#   error unknown multiarch location for ftmodule.h
# endif
#elif defined(__gnu_hurd__)
# include <i386-gnu/freetype2/freetype/config/ftmodule.h>
#else
# error unknown multiarch location for ftmodule.h
#endif
