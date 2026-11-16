/*
 * vc_page_table.inc: expands one page's .def into its command table and
 * exports `const VcPage vc_page_<MNEM>`. Include at the END of the page's .c,
 * after `#define VC_PAGE_DEF "pages/<mnem>.def"`, so static handlers are in scope.
 */

#ifndef VC_PAGE_DEF
#error "define VC_PAGE_DEF before including vc_page_table.inc"
#endif

// VC_NONE detection: pasting VC_PROBE_ onto a handler token yields "~, 1" only
// for VC_NONE. VC_NONE and handler names must never be macros.
#define VC_PROBE_VC_NONE ~, 1
#define VC_ARG2_(a, b, ...) b
#define VC_ARG2(...) VC_ARG2_(__VA_ARGS__)
#define VC_IS_NONE_(probe) VC_ARG2(probe, 0, ~)
#define VC_IIF_0(t, f) f
#define VC_IIF_1(t, f) t
#define VC_IIF_(c) VC_IIF_##c
#define VC_IIF(c) VC_IIF_(c)
#define VC_PROTO_(h) VC_IIF(VC_IS_NONE_(VC_PROBE_##h))(, VcResult h(VcCtx *c);)

// Pass 1: page constants and handler prototypes.
#define VC_PAGE(m, num, rev, doc) enum { vc_this_page_ = (num), vc_this_rev_ = (rev) };
#define VC_CMD(c, name, g, s, ak, gl, smin, smax, fl, a, doc) VC_PROTO_(g) VC_PROTO_(s)
#define VC_RESERVED(c, note)
#include VC_PAGE_DEF
#undef VC_PAGE
#undef VC_CMD
#undef VC_RESERVED

// Pass 2: static checks per entry.
#define VC_PAGE(m, num, rev, doc)
#define VC_CMD(c, name, g, s, ak, gl, smin, smax, fl, a, doc)                         \
    _Static_assert((c) >= 0x10 && (c) <= 0xFF, "command " #name ": number outside 0x10-0xFF"); \
    _Static_assert(((gl) == 0) == VC_IS_NONE_(VC_PROBE_##g),                          \
                   "command " #name ": GET_LEN must be 0 exactly when there is no GET"); \
    _Static_assert((gl) == VC_LEN_VAR || (gl) <= VC_REPLY_MAX || ((fl) & VC_F_BULK),  \
                   "command " #name ": fixed GET_LEN above VC_REPLY_MAX needs VC_F_BULK"); \
    _Static_assert(((fl) & VC_F_BULK) || ((smin) <= (smax) && (smax) <= VC_SET_MAX),   \
                   "command " #name ": SET_MIN/SET_MAX out of order or above 64");    \
    _Static_assert(!VC_IS_NONE_(VC_PROBE_##s) || ((smin) == 0 && (smax) == 0),        \
                   "command " #name ": no SET handler needs SET_MIN and SET_MAX of 0"); \
    _Static_assert((a) >= 0 && (a) <= 255, "command " #name ": ARG outside 0-255");
#define VC_RESERVED(c, note) \
    _Static_assert((c) >= 0x10 && (c) <= 0xFF, "reserved number outside 0x10-0xFF");
#include VC_PAGE_DEF
#undef VC_PAGE
#undef VC_CMD
#undef VC_RESERVED

// Pass 3: a duplicate command number (or one also marked reserved) is a duplicate case.
#define VC_PAGE(m, num, rev, doc)
#define VC_CMD(c, name, g, s, ak, gl, smin, smax, fl, a, doc) case (c):
#define VC_RESERVED(c, note) case (c):
__attribute__((unused)) static void vc_page_dup_check_(int n) {
    switch (n) {
#include VC_PAGE_DEF
    default: break;
    }
}
#undef VC_PAGE
#undef VC_CMD
#undef VC_RESERVED

// Pass 4: the table. The trailing zero entry keeps an empty page legal C.
#define VC_PAGE(m, num, rev, doc)
#define VC_CMD(c, name, g, s, ak, gl, smin, smax, fl, a, doc)              \
    { .cmd = (c), .addr_kind = VC_A_##ak, .set_min = (smin), .set_max = (smax), \
      .get_len = (gl), .flags = (fl), .arg = (a),                         \
      .get = VC_IIF(VC_IS_NONE_(VC_PROBE_##g))((VcHandler)0, g),           \
      .set = VC_IIF(VC_IS_NONE_(VC_PROBE_##s))((VcHandler)0, s) },
#define VC_RESERVED(c, note)
static const VcCmd vc_page_cmds_[] = {
#include VC_PAGE_DEF
    { 0 }
};
#undef VC_PAGE
#undef VC_CMD
#undef VC_RESERVED

_Static_assert(sizeof(vc_page_cmds_) / sizeof(vc_page_cmds_[0]) - 1 <= 240,
               "a page holds at most 240 commands");

// Pass 5: the exported page object.
#define VC_PAGE(m, num, rev, doc)                                              \
    const VcPage vc_page_##m = {                                              \
        .page = vc_this_page_, .revision = vc_this_rev_,                      \
        .count = (uint8_t)(sizeof(vc_page_cmds_) / sizeof(vc_page_cmds_[0]) - 1), \
        .cmds = vc_page_cmds_ };                                              \
    _Static_assert((int)VC_MNEM_OF_##m == (int)vc_this_page_, "VC_MNEM block for this page is wrong in vc_ids.h");
#define VC_CMD(c, name, g, s, ak, gl, smin, smax, fl, a, doc)
#define VC_RESERVED(c, note)
#include VC_PAGE_DEF
#undef VC_PAGE
#undef VC_CMD
#undef VC_RESERVED

#undef VC_PAGE_DEF
