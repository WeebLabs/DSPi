/*
 * vc_ids.h: VC_PAGE_<MNEM> page numbers and VC_<MNEM>_<NAME> command IDs,
 * generated from the pages/<mnem>.def files. Safe to include from any firmware file.
 */

#ifndef VC_IDS_H
#define VC_IDS_H

#include "pico.h"   // PICO_RP2350 for conditional .def entries

// Every page in ascending order. Adding a page: a line here, a block below,
// pages/<mnem>.def and vc_page_<mnem>.c.
#define VC_PAGE_LIST(X) \
    X(SYS) X(STORE) X(INPUT) X(OUTPUT) X(ROUTE) X(FILTER) X(DYN) \
    X(FX) X(MEAS) X(CS) X(XPORT) X(CLOCK) X(DIAG)

#define VC_IDS_CAT_(a, b) a##b
#define VC_IDS_CAT(a, b) VC_IDS_CAT_(a, b)
#define VC_IDS_CAT3_(a, b, c) a##b##c
#define VC_IDS_CAT3(a, b, c) VC_IDS_CAT3_(a, b, c)

// VC_MNEM must match the VC_PAGE line of the file included under it. VC_MNEM_OF_*
// lets each page's table check that (vc_page_table.inc pass 5).
#define VC_PAGE(m, num, rev, doc) VC_PAGE_##m = (num), VC_IDS_CAT(VC_MNEM_OF_, VC_MNEM) = (num),
#define VC_CMD(c, name, g, s, ak, gl, smin, smax, fl, a, doc) \
    VC_IDS_CAT3(VC_, VC_MNEM, _##name) = (VC_IDS_CAT(VC_PAGE_, VC_MNEM) << 8) | (c),
#define VC_RESERVED(c, note)

enum {
#define VC_MNEM SYS
#include "pages/sys.def"
#undef VC_MNEM
#define VC_MNEM STORE
#include "pages/store.def"
#undef VC_MNEM
#define VC_MNEM INPUT
#include "pages/input.def"
#undef VC_MNEM
#define VC_MNEM OUTPUT
#include "pages/output.def"
#undef VC_MNEM
#define VC_MNEM ROUTE
#include "pages/route.def"
#undef VC_MNEM
#define VC_MNEM FILTER
#include "pages/filter.def"
#undef VC_MNEM
#define VC_MNEM DYN
#include "pages/dyn.def"
#undef VC_MNEM
#define VC_MNEM FX
#include "pages/fx.def"
#undef VC_MNEM
#define VC_MNEM MEAS
#include "pages/meas.def"
#undef VC_MNEM
#define VC_MNEM CS
#include "pages/cs.def"
#undef VC_MNEM
#define VC_MNEM XPORT
#include "pages/xport.def"
#undef VC_MNEM
#define VC_MNEM CLOCK
#include "pages/clock.def"
#undef VC_MNEM
#define VC_MNEM DIAG
#include "pages/diag.def"
#undef VC_MNEM
};

#undef VC_PAGE
#undef VC_CMD
#undef VC_RESERVED

#endif // VC_IDS_H
