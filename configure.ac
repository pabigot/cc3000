# If creating a fresh area from a maintainer checkout, do:
# mkdir m4
# libtoolize --install --copy
# automake --add-missing --copy
# autoreconf

AC_INIT([cc3000],[1.12])
AC_CONFIG_SRCDIR([include/cc3000/cc3000_common.h])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_AUX_DIR([build-aux])
AM_MAINTAINER_MODE
AC_CANONICAL_SYSTEM
AM_INIT_AUTOMAKE([-Wall -Werror foreign no-define no-dist])

# Tell Makefile if we're building for the MSP430.
AM_CONDITIONAL([WITH_MSP430], [test "${host_alias}" = "msp430"])

# Need a C compiler.  We're building a library, and for some targets
# (like msp430) we may not be providing the -mmcu= option that's
# required to be able to do a non-relocatable link.  So don't ask
# AC_PROG_CC to check whether executables can be built.
AC_NO_EXECUTABLES
AC_PROG_CC
AM_CONDITIONAL([WITH_GCC], [test "${GCC}" = "yes"])

# TARGET_CFLAGS is placed in AM_CFLAGS allowing special flags to be
# provided without affecting the toolchain default flags.
AC_ARG_VAR([TARGET_CFLAGS],
           [Target-specific flags added before standard CFLAGS variable])

# Optionally build for multiple memory models for msp430
ENABLE_MEMORY_MODELS=no
AC_ARG_ENABLE([memory-models],
  [AS_HELP_STRING([--enable-memory-models=MODELS],
    [comma-separated list of memory models to build
     @<:@MODELS={none,small,medium,large,huge;all}@:>@])],
  ENABLE_MEMORY_MODELS=$enableval)
if test "${host_alias}" = "msp430" ; then
  AC_MSG_CHECKING(enabled memory models)
  memory_models=
  case ",${ENABLE_MEMORY_MODELS}," in
    ,default,|,yes,|*,all,*)
      memory_models='none,small,medium,large,huge'
      ;;
    ,no,)
      memory_models=none
      ;;
  esac    
  memory_models=`echo ${memory_models} | sed 's/,/ /g'`
  for i in ${memory_models} ; do
    case $i in
      none|small|medium|large|huge)
        ;;
      *)
        AC_MSG_ERROR([invalid --enable-memory-model argument ${i}])
        ;;
    esac
  done
  AC_MSG_RESULT(${memory_models})
  AC_SUBST(MEMORY_MODELS,[${memory_models}])
elif test "${ENABLE_MEMORY_MODELS}" != "no" ; then
  AC_MSG_WARN([--enable-memory-models only available with --host=msp430])
fi

# Use libtool; no shared libraries for embedded platforms
LT_INIT([disable-shared])

# See whether we need to define the standard time structures
AC_CHECK_HEADERS([sys/time.h time.h])

# Put config.h down in the private area.  We need it to determine what
# workaround structures need to be declared locally when libc doesn't
# provide a suitable version.
AC_CONFIG_HEADERS([include/cc3000/config.h])

AC_CONFIG_FILES([Makefile src/Makefile include/cc3000/Makefile])

AC_OUTPUT
