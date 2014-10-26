NO_WERROR=
WERROR=-Wno-error
KMOD=	ipu

SRCS=   device_if.h bus_if.h ofw_bus_if.h
SRCS+= opt_syscons.h opt_teken.h opt_splash.h fb_if.h
SRCS+=	imx51_ipuv3_fbd.c

.include <bsd.kmod.mk>
