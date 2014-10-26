NO_WERROR=
WERROR=-Wno-error 
KMOD=	ipu

SRCS=   device_if.h bus_if.h ofw_bus_if.h
SRCS+=	ipuv3.c

.include <bsd.kmod.mk>
