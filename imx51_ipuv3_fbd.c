/*-
 * Copyright (c) 2012 Oleksandr Tymoshenko <gonzo@freebsd.org>
 * Copyright (c) 2012, 2013 The FreeBSD Foundation
 * All rights reserved.
 *
 * Portions of this software were developed by Oleksandr Rybalko
 * under sponsorship from the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bio.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/time.h>
#include <sys/timetc.h>
#include <sys/fbio.h>
#include <sys/consio.h>
#include <sys/eventhandler.h>

#include <sys/kdb.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/fdt.h>
#include <machine/resource.h>
#include <machine/frame.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/vt/vt.h>
#include <dev/vt/colors/vt_termcolors.h>

#include <arm/freescale/imx/imx51_ccmvar.h>

// XXX
#include "imx51_ipuv3reg.h"
//#include <arm/freescale/imx/imx51_ipuv3reg.h>

#include "fb_if.h"

struct ipu3sc_softc {
	device_t		dev;
	device_t		sc_fbd;		/* fbd child */
	struct fb_info		sc_info;

	bus_space_tag_t		iot;
	bus_space_handle_t	ioh;
	bus_space_handle_t	cm_ioh;
	bus_space_handle_t	dp_ioh;
	bus_space_handle_t	di0_ioh;
	bus_space_handle_t	di1_ioh;
	bus_space_handle_t	dctmpl_ioh;
	bus_space_handle_t	dc_ioh;
	bus_space_handle_t	dmfc_ioh;
	bus_space_handle_t	idmac_ioh;
	bus_space_handle_t	cpmem_ioh;
};

#define	IPUV3_READ(ipuv3, module, reg)					\
	bus_space_read_4((ipuv3)->iot, (ipuv3)->module##_ioh, (reg))
#define	IPUV3_WRITE(ipuv3, module, reg, val)				\
	bus_space_write_4((ipuv3)->iot, (ipuv3)->module##_ioh, (reg), (val))

#define	CPMEM_CHANNEL_OFFSET(_c)	((_c) * 0x40)
#define	CPMEM_WORD_OFFSET(_w)		((_w) * 0x20)
#define	CPMEM_DP_OFFSET(_d)		((_d) * 0x10000)
#define	IMX_IPU_DP0		0
#define	IMX_IPU_DP1		1
#define	CPMEM_CHANNEL(_dp, _ch, _w)					\
	    (CPMEM_DP_OFFSET(_dp) + CPMEM_CHANNEL_OFFSET(_ch) +		\
		CPMEM_WORD_OFFSET(_w))
#define	CPMEM_OFFSET(_dp, _ch, _w, _o)					\
	    (CPMEM_CHANNEL((_dp), (_ch), (_w)) + (_o))

enum IPUV3_SUBMODULES {
	IPUV3_CM,
	IPUV3_IDMAC,
	IPUV3_DP,
	IPUV3_IC,
	IPUV3_IRT,
	IPUV3_CSI0,
	IPUV3_CSI1,
	IPUV3_DI0,
	IPUV3_DI1,
	IPUV3_SMFC,
	IPUV3_DC,
	IPUV3_DMFC,
	IPUV3_VDI,
	IPUV3_CPMEM,
	IPUV3_LUT,
	IPUV3_SRM,
	IPUV3_TPM,
	IPUV3_DCTMPL,
	IPUV3_SUBMODULES_COUNT
};

struct submodule_info
{
	char *name;
	bus_addr_t addr;
	bus_addr_t size;
	bus_space_handle_t *ioh;
};


static int	ipu3_fb_probe(device_t);
static int	ipu3_fb_attach(device_t);
static int	ipu3_fb_detach(device_t);

/* XXX: read the configuration from u-boot and allocate a similiar fb */
static void
ipu3_fb_init(struct ipu3sc_softc *sc)
{
	uint64_t w0sh96;
	uint32_t w1sh96;

	/* FW W0[137:125] - 96 = [41:29] */
	/* FH W0[149:138] - 96 = [53:42] */
	w0sh96 = IPUV3_READ(sc, cpmem, CPMEM_OFFSET(IMX_IPU_DP1, 23, 0, 16));
	w0sh96 <<= 32;
	w0sh96 |= IPUV3_READ(sc, cpmem, CPMEM_OFFSET(IMX_IPU_DP1, 23, 0, 12));

	sc->sc_info.fb_width = ((w0sh96 >> 29) & 0x1fff) + 1;
	sc->sc_info.fb_height = ((w0sh96 >> 42) & 0x0fff) + 1;

	/* SLY W1[115:102] - 96 = [19:6] */
	w1sh96 = IPUV3_READ(sc, cpmem, CPMEM_OFFSET(IMX_IPU_DP1, 23, 1, 12));
	sc->sc_info.fb_stride = ((w1sh96 >> 6) & 0x3fff) + 1;

	printf("%dx%d [%d]\n", sc->sc_info.fb_width, sc->sc_info.fb_height,
	    sc->sc_info.fb_stride);
	sc->sc_info.fb_size = sc->sc_info.fb_height * sc->sc_info.fb_stride;

	sc->sc_info.fb_vbase = (intptr_t)contigmalloc(sc->sc_info.fb_size,
	    M_DEVBUF, M_ZERO, 0, ~0, PAGE_SIZE, 0);
	sc->sc_info.fb_pbase = (intptr_t)vtophys(sc->sc_info.fb_vbase);

	/* DP1 + config_ch_23 + word_2 */
	IPUV3_WRITE(sc, cpmem, CPMEM_OFFSET(IMX_IPU_DP1, 23, 1, 0),
	    (((uint32_t)sc->sc_info.fb_pbase >> 3) |
	    (((uint32_t)sc->sc_info.fb_pbase >> 3) << 29)) & 0xffffffff);

	IPUV3_WRITE(sc, cpmem, CPMEM_OFFSET(IMX_IPU_DP1, 23, 1, 4),
	    (((uint32_t)sc->sc_info.fb_pbase >> 3) >> 3) & 0xffffffff);

	/* XXX: fetch or set it from/to IPU. */
	sc->sc_info.fb_bpp = sc->sc_info.fb_depth = sc->sc_info.fb_stride /
	    sc->sc_info.fb_width * 8;
}

/* Use own color map, because of different RGB offset. */
static int
ipu3_fb_init_cmap(uint32_t *cmap, int bytespp)
{

	switch (bytespp) {
	case 8:
		return (vt_generate_cons_palette(cmap, COLOR_FORMAT_RGB,
		    0x7, 5, 0x7, 2, 0x3, 0));
	case 15:
		return (vt_generate_cons_palette(cmap, COLOR_FORMAT_RGB,
		    0x1f, 10, 0x1f, 5, 0x1f, 0));
	case 16:
		return (vt_generate_cons_palette(cmap, COLOR_FORMAT_RGB,
		    0x1f, 11, 0x3f, 5, 0x1f, 0));
	case 24:
	case 32: /* Ignore alpha. */
		return (vt_generate_cons_palette(cmap, COLOR_FORMAT_RGB,
		    0xff, 0, 0xff, 8, 0xff, 16));
	default:
		return (1);
	}
}

static int
ipu3_fb_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "fsl,ipu3") &&
		!ofw_bus_is_compatible(dev, "fsl,imx6q-ipu"))
		return (ENXIO);

	device_set_desc(dev, "i.MX Image Processing Unit v3 (FB)");

	return (BUS_PROBE_DEFAULT);
}

static void
ipu3_set_addresses(struct submodule_info *modules, bus_addr_t base,
			 bus_addr_t cm_offset, bus_addr_t cpmem_offset)
{
	bus_addr_t cm_addr;
	bus_addr_t cpmem_addr;

	/*
	 * The base address and offset of the rigisters from the base
	 * depend on the SOC.
	 */
	cm_addr = base + cm_offset;
	modules[IPUV3_CM].addr = cm_addr + IPU_CM_CM_OFFSET;
	modules[IPUV3_IDMAC].addr = cm_addr + IPU_IDMAC_CM_OFFSET;
	modules[IPUV3_DP].addr = cm_addr + IPU_DP_CM_OFFSET;
	modules[IPUV3_IC].addr = cm_addr + IPU_IC_CM_OFFSET;
	modules[IPUV3_IRT].addr = cm_addr + IPU_IRT_CM_OFFSET;
	modules[IPUV3_CSI0].addr = cm_addr + IPU_CSI0_CM_OFFSET;
	modules[IPUV3_CSI1].addr = cm_addr + IPU_CSI1_CM_OFFSET;
	modules[IPUV3_DI0].addr = cm_addr + IPU_DI0_CM_OFFSET;
	modules[IPUV3_DI1].addr = cm_addr + IPU_DI1_CM_OFFSET;
	modules[IPUV3_SMFC].addr = cm_addr + IPU_SMFC_CM_OFFSET;
	modules[IPUV3_DC].addr = cm_addr + IPU_DC_CM_OFFSET;
	modules[IPUV3_DMFC].addr = cm_addr + IPU_DMFC_CM_OFFSET;
	modules[IPUV3_VDI].addr = cm_addr + IPU_VDI_CM_OFFSET;

	/*
	 * cpmem relative, this differs between imx51, imx53 on one hand
	 * the imx6 on the other side.
	 */
	cpmem_addr = base + cpmem_offset;
	modules[IPUV3_CPMEM].addr = cpmem_addr + IPU_CPMEM_CPMEM_OFFSET;
	modules[IPUV3_LUT].addr = cpmem_addr + IPU_LUT_CPMEM_OFFSET;
	modules[IPUV3_SRM].addr = cpmem_addr + IPU_SRM_CPMEM_OFFSET;
	modules[IPUV3_TPM].addr = cpmem_addr + IPU_TPM_CPMEM_OFFSET;
	modules[IPUV3_DCTMPL].addr = cpmem_addr + IPU_DCTMPL_CPMEM_OFFSET;
}

static void
ipu3_dump_submodule_info(struct submodule_info *modules, size_t count)
{
	struct submodule_info *mod = modules;
	size_t i;

	for (i = 0; i < count; i++) {
		uprintf("%10s %lX %lX\n", mod->name, mod->addr, mod->size);
		mod++;
	}

}

// XXX: is there no common type for this?
struct arm32_regs {
	pcell_t phys;
	pcell_t size;
};

static int
ipu3_fb_attach(device_t dev)
{
	struct ipu3sc_softc *sc = device_get_softc(dev);
	phandle_t node;
	struct arm32_regs *regs;
	int err;
	uintptr_t base;
	struct submodule_info *mod;
	size_t i;
	struct submodule_info mods[] = {
		{.name = "CM", .size = IPU_CM_SIZE, .ioh = &sc->cm_ioh},
		{.name = "IDMAC", .size = IPU_DMFC_SIZE, .ioh = &sc->idmac_ioh},
		{.name = "DP", .size = IPU_DP_SIZE, .ioh = &sc->dp_ioh},
		{.name = "IC", .size = IPU_IC_SIZE, .ioh = NULL},
		{.name = "IRT", .size = IPU_IRT_SIZE, .ioh = NULL},
		{.name = "CSIO", .size = IPU_CSI0_SIZE, .ioh = NULL},
		{.name = "CSI1", .size = IPU_CSI1_SIZE, .ioh = NULL},
		{.name = "DIO0", .size = IPU_DI0_SIZE, .ioh = &sc->di0_ioh},
		{.name = "DIO1", .size = IPU_DI1_SIZE, .ioh = &sc->di1_ioh},
		{.name = "SMFC", .size = IPU_SMFC_SIZE, .ioh = NULL},
		{.name = "DC", .size = IPU_DC_SIZE, .ioh = &sc->dc_ioh},
		{.name = "DMFC", .size = IPU_DMFC_SIZE, .ioh = &sc->dmfc_ioh},
		{.name = "VDI", .size = IPU_VDI_SIZE, .ioh = NULL},
		{.name = "CPMEM", .size = IPU_CPMEM_SIZE, .ioh = &sc->cpmem_ioh},
		{.name = "LUT", .size = IPU_LUT_SIZE, .ioh = NULL},
		{.name = "SRM", .size = IPU_SRM_SIZE, .ioh = NULL},
		{.name = "TPM", .size = IPU_TPM_SIZE, .ioh = NULL},
		{.name = "DCTMPL", .size = IPU_DCTMPL_SIZE, .ioh = &sc->dctmpl_ioh},
	};
	int mod_count = sizeof(mods) / sizeof(mods[0]);
	int nregs;

	sc->dev = dev;
	sc->iot = fdtbus_bs_tag;

#if 0
	if (bootverbose)
		device_printf(dev, "clock gate status is %d\n",
		    imx51_get_clk_gating(IMX51CLK_IPU_HSP_CLK_ROOT));
#endif
	uprintf("ipu3_fb_attach\n");

	node = ofw_bus_get_node(dev);
	nregs = OF_getprop_alloc(node, "reg", sizeof(*regs), (void **)&regs);
	if (nregs <= 0) {
		uprintf("obtaining reg failed");
		return (ENXIO);
	}

	if (nregs != 1 && nregs < mod_count) {
		free(regs, M_OFWPROP);
		uprintf("either the base address are all submodule registers must be provided!");
		return (ENXIO);
	}

	if (nregs == 1) {
		uprintf("single reg given, assuming base address\n");
		// XXX: make this SOC specific
		ipu3_set_addresses(mods, fdt32_to_cpu(regs[i].phys), 0x200000, 0x300000);
	} else {
		uprintf("all regs given, parsing\n");
		for (i = 0; i < mod_count; i++) {
			mods[i].addr = fdt32_to_cpu(regs[i].phys);
			mods[i].size = fdt32_to_cpu(regs[i].size);
		}
	}
	free(regs, M_OFWPROP);

	uprintf("count: %d\n", nregs);
	ipu3_dump_submodule_info(mods, mod_count);

	mod = mods;
	for (i = 0; i < mod_count; i++, mod++) {
		if (mod->ioh == NULL)
			continue;

		uprintf("mapping %s\n", mod->name);

		/* map controller registers */
		err = bus_space_map(sc->iot, mod->addr, mod->size, 0,
				    mod->ioh);
		if (err)
			goto fail;
	}

#ifdef notyet
	sc->ih = imx51_ipuv3_intr_establish(IMX51_INT_IPUV3, IPL_BIO,
	    ipuv3intr, sc);
	if (sc->ih == NULL) {
		device_printf(sc->dev,
		    "unable to establish interrupt at irq %d\n",
		    IMX51_INT_IPUV3);
		return (ENXIO);
	}
#endif

	ipu3_fb_init(sc);

	sc->sc_info.fb_name = device_get_nameunit(dev);

	ipu3_fb_init_cmap(sc->sc_info.fb_cmap, sc->sc_info.fb_depth);
	sc->sc_info.fb_cmsize = 16;

	/* Ask newbus to attach framebuffer device to me. */
	sc->sc_fbd = device_add_child(dev, "fbd", device_get_unit(dev));
	if (sc->sc_fbd == NULL)
		device_printf(dev, "Can't attach fbd device\n");

	return (bus_generic_attach(dev));

fail:
	device_printf(sc->dev, "failed to map registers (%s, errno=%d)\n",
		      mod->name, err);

	mod--;
	while (mod != mods)
		if (mod->ioh)
			bus_space_unmap(sc->iot, *mod->ioh, mod->size);

	return (err);
}

static int
ipu3_fb_detach(device_t dev)
{
	return (0);
}

static struct fb_info *
ipu3_fb_getinfo(device_t dev)
{
	struct ipu3sc_softc *sc = device_get_softc(dev);

	return (&sc->sc_info);
}

static device_method_t ipu3_fb_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		ipu3_fb_probe),
	DEVMETHOD(device_attach,	ipu3_fb_attach),
	DEVMETHOD(device_detach,	ipu3_fb_detach),

	/* Framebuffer service methods */
	DEVMETHOD(fb_getinfo,		ipu3_fb_getinfo),
	{ 0, 0 }
};

static devclass_t ipu3_fb_devclass;

static driver_t ipu3_fb_driver = {
	"fb",
	ipu3_fb_methods,
	sizeof(struct ipu3sc_softc),
};

DRIVER_MODULE(fb, simplebus, ipu3_fb_driver, ipu3_fb_devclass, 0, 0);
