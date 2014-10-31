NO_WERROR=
WERROR=-Wno-error 
KMOD=	ipu3

#SRCS=   device_if.h bus_if.h ofw_bus_if.h
SRCS= opt_syscons.h opt_teken.h opt_splash.h fb_if.h
SRCS+=	ipu3_fb.c

.include <bsd.kmod.mk>
