/* DSPi RP2350 XIP linker script.
   Derived from pico-sdk 2.2.0 src/rp2_common/pico_crt0/rp2350/memmap_default.ld
   (binary type "default": code executes from flash via the XIP cache).
   Local edits, kept minimal for future SDK-upgrade diffs:
     1. Flash .text EXCLUDE_FILE extended with the hot objects listed below.
     2. Flash .rodata EXCLUDE_FILE extended the same way.
     3. .data gains an explicit RAM code pull list for those objects, bracketed
        by __ram_code_pull_start__/__end__ with a size ASSERT (rename guard).
   Hot RAM residency is otherwise driven by source attributes:
   .time_critical (DSP_TIME_CRITICAL, __time_critical_func) and
   __not_in_flash_func sections land in .data below, exactly as in the SDK
   script.  See scripts/check_ram_placement.py for the enforced symbol list.

   Based on GCC ARM embedded samples.
   Defines the following symbols for use by code:
    __exidx_start
    __exidx_end
    __etext
    __data_start__
    __preinit_array_start
    __preinit_array_end
    __init_array_start
    __init_array_end
    __fini_array_start
    __fini_array_end
    __data_end__
    __bss_start__
    __bss_end__
    __end__
    end
    __HeapLimit
    __StackLimit
    __StackTop
    __stack (== StackTop)
*/

MEMORY
{
    INCLUDE "pico_flash_region.ld"
    RAM(rwx) : ORIGIN =  0x20000000, LENGTH = 512k
    SCRATCH_X(rwx) : ORIGIN = 0x20080000, LENGTH = 4k
    SCRATCH_Y(rwx) : ORIGIN = 0x20081000, LENGTH = 4k
}

ENTRY(_entry_point)

SECTIONS
{
    .flash_begin : {
        __flash_binary_start = .;
    } > FLASH

    /* The bootrom will enter the image at the point indicated in your
       IMAGE_DEF, which is usually the reset handler of your vector table.

       The debugger will use the ELF entry point, which is the _entry_point
       symbol, and in our case is *different from the bootrom's entry point.*
       This is used to go back through the bootrom on debugger launches only,
       to perform the same initial flash setup that would be performed on a
       cold boot.
    */

    .text : {
        __logical_binary_start = .;
        KEEP (*(.vectors))
        KEEP (*(.binary_info_header))
        __binary_info_header_end = .;
        KEEP (*(.embedded_block))
        __embedded_block_end = .;
        KEEP (*(.reset))
        /* TODO revisit this now memset/memcpy/float in ROM */
        /* bit of a hack right now to exclude all floating point and time critical (e.g. memset, memcpy) code from
         * FLASH ... we will include any thing excluded here in .data below by default */
        *(.init)
        *libgcc.a:cmse_nonsecure_call.o
        /* DSPi: also exclude the hot USB/audio-support objects (RAM code pull
           list in .data below): TinyUSB device core + tusb.c helpers run per
           ISO transfer and inside the USB ISR, tusb_fifo backs the OSAL event
           queue, float_math hosts the leveller's per-block powf/log10f
           wrappers, and the alarm pool (pico_time/pheap/hardware_timer) is
           cancelled and re-armed from the SPDIF RX DMA ISR on every block.
           spdif_rx FIFO accessors and clocks (servo path) plus the notify
           ring (drained from the USB ISR) are steady-state hot as well.
           Paths are anchored so unrelated same-named objects cannot match.
           The extra newlib patterns cover Arm GNU toolchains that link
           libg.a with libc_a-*.o member names; the SDK pattern misses them
           and mem* would silently land in flash. */
        *(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:
            *libg.a:*lib_a-mem*.o *libc.a:*libc_a-mem*.o *libg.a:*libc_a-mem*.o
            *tinyusb/src/device/usbd.c.o *tinyusb/src/device/usbd_control.c.o
            *tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c.o
            *tinyusb/src/portable/raspberrypi/rp2040/rp2040_usb.c.o
            *tinyusb/src/common/tusb_fifo.c.o *tinyusb/src/tusb.c.o
            *pico_float/float_math.c.o *pico_time/time.c.o
            *pico_util/pheap.c.o *hardware_timer/timer.c.o
            *hardware_clocks/clocks.c.o *DSPi.dir/notify.c.o
            *pico_spdif_rx/spdif_rx.c.o) .text*)
        *(.fini)
        /* Pull all c'tors into .text */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)
        /* Followed by destructors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(SORT(.preinit_array.*)))
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        *(SORT(.fini_array.*))
        *(.fini_array)
        PROVIDE_HIDDEN (__fini_array_end = .);

        *(.eh_frame*)
        . = ALIGN(4);
    } > FLASH

    /* Note the boot2 section is optional, and should be discarded if there is
       no reference to it *inside* the binary, as it is not called by the
       bootrom. (The bootrom performs a simple best-effort XIP setup and
       leaves it to the binary to do anything more sophisticated.) However
       there is still a size limit of 256 bytes, to ensure the boot2 can be
       stored in boot RAM.

       Really this is a "XIP setup function" -- the name boot2 is historic and
       refers to its dual-purpose on RP2040, where it also handled vectoring
       from the bootrom into the user image.
    */

    .boot2 : {
        __boot2_start__ = .;
        *(.boot2)
        __boot2_end__ = .;
    } > FLASH

    ASSERT(__boot2_end__ - __boot2_start__ <= 256,
        "ERROR: Pico second stage bootloader must be no more than 256 bytes in size")

    .rodata : {
        /* DSPi: same exclusions as .text above, plus spdif_rx rodata (the
           sample-rate tables are read per block inside the RX DMA ISR). */
        *(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:
            *libg.a:*lib_a-mem*.o *libc.a:*libc_a-mem*.o *libg.a:*libc_a-mem*.o
            *tinyusb/src/device/usbd.c.o *tinyusb/src/device/usbd_control.c.o
            *tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c.o
            *tinyusb/src/portable/raspberrypi/rp2040/rp2040_usb.c.o
            *tinyusb/src/common/tusb_fifo.c.o *tinyusb/src/tusb.c.o
            *pico_float/float_math.c.o *pico_time/time.c.o
            *pico_util/pheap.c.o *hardware_timer/timer.c.o
            *hardware_clocks/clocks.c.o *DSPi.dir/notify.c.o
            *pico_spdif_rx/spdif_rx.c.o) .rodata*)
        *(.srodata*)
        . = ALIGN(4);
        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.flashdata*)))
        . = ALIGN(4);
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    __exidx_end = .;

    /* Machine inspectable binary information */
    . = ALIGN(4);
    __binary_info_start = .;
    .binary_info :
    {
        KEEP(*(.binary_info.keep.*))
        *(.binary_info.*)
    } > FLASH
    __binary_info_end = .;
    . = ALIGN(4);

    .ram_vector_table (NOLOAD): {
        *(.ram_vector_table)
    } > RAM

    .uninitialized_data (NOLOAD): {
        . = ALIGN(4);
        *(.uninitialized_data*)
    } > RAM

    .data : {
        __data_start__ = .;
        *(vtable)

        *(.time_critical*)

        /* DSPi: RAM code pull list, mirrors the flash EXCLUDE_FILE entries.
           These objects run in the USB ISR / per audio block and must not
           take XIP cache misses.  The bracket symbols exist so the rename
           guard ASSERT below can measure the region. */
        __ram_code_pull_start__ = .;
        *tinyusb/src/device/usbd.c.o(.text* .rodata*)
        *tinyusb/src/device/usbd_control.c.o(.text* .rodata*)
        *tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c.o(.text* .rodata*)
        *tinyusb/src/portable/raspberrypi/rp2040/rp2040_usb.c.o(.text* .rodata*)
        *tinyusb/src/common/tusb_fifo.c.o(.text* .rodata*)
        *tinyusb/src/tusb.c.o(.text* .rodata*)
        *pico_float/float_math.c.o(.text* .rodata*)
        *pico_time/time.c.o(.text* .rodata*)
        *pico_util/pheap.c.o(.text* .rodata*)
        *hardware_timer/timer.c.o(.text* .rodata*)
        *hardware_clocks/clocks.c.o(.text* .rodata*)
        *DSPi.dir/notify.c.o(.text* .rodata*)
        *pico_spdif_rx/spdif_rx.c.o(.text* .rodata*)
        __ram_code_pull_end__ = .;

        /* remaining .text and .rodata; i.e. stuff we exclude above because we want it in RAM */
        *(.text*)
        . = ALIGN(4);
        *(.rodata*)
        . = ALIGN(4);

        *(.data*)
        *(.sdata*)

        . = ALIGN(4);
        *(.after_data.*)
        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__mutex_array_start = .);
        KEEP(*(SORT(.mutex_array.*)))
        KEEP(*(.mutex_array))
        PROVIDE_HIDDEN (__mutex_array_end = .);

        *(.jcr)
        . = ALIGN(4);
    } > RAM AT> FLASH

    /* An unmatched object wildcard is silent; if a listed file is renamed on
       an SDK or TinyUSB update, its code moves to flash and the USB ISR path
       starts taking XIP misses unnoticed.  Coarse tripwire only (the region
       holds ~5 KB; the hottest parts of these objects land in .time_critical
       above); per-symbol enforcement is scripts/check_ram_placement.py. */
    ASSERT(__ram_code_pull_end__ - __ram_code_pull_start__ > 3K,
        "DSPi: RAM code pull list lost objects (renamed file after SDK/TinyUSB update?); update memmap_dspi_rp2350_xip.ld")

    .tdata : {
        . = ALIGN(4);
		*(.tdata .tdata.* .gnu.linkonce.td.*)
        /* All data end */
        __tdata_end = .;
    } > RAM AT> FLASH
    PROVIDE(__data_end__ = .);

    /* __etext is (for backwards compatibility) the name of the .data init source pointer (...) */
    __etext = LOADADDR(.data);

    .tbss (NOLOAD) : {
        . = ALIGN(4);
        __bss_start__ = .;
        __tls_base = .;
        *(.tbss .tbss.* .gnu.linkonce.tb.*)
        *(.tcommon)

        __tls_end = .;
    } > RAM

    .bss (NOLOAD) : {
        . = ALIGN(4);
        __tbss_end = .;

        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.bss*)))
        *(COMMON)
        PROVIDE(__global_pointer$ = . + 2K);
        *(.sbss*)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    .heap (NOLOAD):
    {
        __end__ = .;
        end = __end__;
        KEEP(*(.heap*))
    } > RAM
    /* Output slot rings (usb_audio.c slot_ring_store) at the top of RAM: DMA ring
       wrap needs ring-size alignment, which costs no padding here. NOLOAD and not
       zeroed by crt0; every slot setup stamps its whole ring. */
    SLOT_RINGS_BYTES = 0x8000;
    .slot_rings (ORIGIN(RAM) + LENGTH(RAM) - SLOT_RINGS_BYTES) (NOLOAD) :
    {
        KEEP(*(.slot_rings))
    } > RAM
    ASSERT(SIZEOF(.slot_rings) == SLOT_RINGS_BYTES, "slot_ring_store size must match SLOT_RINGS_BYTES")

    /* historically on GCC sbrk was growing past __HeapLimit to __StackLimit, however
       to be more compatible, we now set __HeapLimit explicitly to where the end of the heap is */
    __HeapLimit = ADDR(.slot_rings);

    /* Start and end symbols must be word-aligned */
    .scratch_x : {
        __scratch_x_start__ = .;
        *(.scratch_x.*)
        . = ALIGN(4);
        __scratch_x_end__ = .;
    } > SCRATCH_X AT > FLASH
    __scratch_x_source__ = LOADADDR(.scratch_x);

    .scratch_y : {
        __scratch_y_start__ = .;
        *(.scratch_y.*)
        . = ALIGN(4);
        __scratch_y_end__ = .;
    } > SCRATCH_Y AT > FLASH
    __scratch_y_source__ = LOADADDR(.scratch_y);

    /* .stack*_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later
     *
     * stack1 section may be empty/missing if platform_launch_core1 is not used */

    /* by default we put core 0 stack at the end of scratch Y, so that if core 1
     * stack is not used then all of SCRATCH_X is free.
     */
    .stack1_dummy (NOLOAD):
    {
        *(.stack1*)
    } > SCRATCH_X
    .stack_dummy (NOLOAD):
    {
        KEEP(*(.stack*))
    } > SCRATCH_Y

    .flash_end : {
        KEEP(*(.embedded_end_block*))
        PROVIDE(__flash_binary_end = .);
    } > FLASH =0xaa

    /* stack limit is poorly named, but historically is maximum heap ptr: the SDK's
       _sbrk and pico_malloc bound the heap by THIS symbol, not __HeapLimit, so it
       must stop below the slot rings or malloc could grow into live DMA rings. */
    __StackLimit = ADDR(.slot_rings);
    __StackOneTop = ORIGIN(SCRATCH_X) + LENGTH(SCRATCH_X);
    __StackTop = ORIGIN(SCRATCH_Y) + LENGTH(SCRATCH_Y);
    __StackOneBottom = __StackOneTop - SIZEOF(.stack1_dummy);
    __StackBottom = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);

    /* picolibc and LLVM */
    PROVIDE (__heap_start = __end__);
    PROVIDE (__heap_end = __HeapLimit);
    PROVIDE( __tls_align = MAX(ALIGNOF(.tdata), ALIGNOF(.tbss)) );
    PROVIDE( __tls_size_align = (__tls_size + __tls_align - 1) & ~(__tls_align - 1));
    PROVIDE( __arm32_tls_tcb_offset = MAX(8, __tls_align) );

    /* llvm-libc */
    PROVIDE (_end = __end__);
    PROVIDE (__llvm_libc_heap_limit = __HeapLimit);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed")

    ASSERT( __binary_info_header_end - __logical_binary_start <= 1024, "Binary info must be in first 1024 bytes of the binary")
    ASSERT( __embedded_block_end - __logical_binary_start <= 4096, "Embedded block must be in first 4096 bytes of the binary")

    /* todo assert on extra code */
}
