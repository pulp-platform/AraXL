/* This file is used to generate link.ld, Ara's linker script,
   which depends on the number of lanes of the current configuration */

OUTPUT_ARCH("riscv")
ENTRY(_start)

/* One L2 bank holds L2NumWords x (AxiDataWidth/8) bytes. L2NumWords is 2^20
   (ara_soc.sv's parameter, also mirrored in ara_tb.sv); AxiDataWidth/8 equals
   ALIGNMENT, substituted by align_sections.sh along with NRL2BANKS.
   The top 2 KB is reserved: tc_sram decodes only $clog2(L2NumWords) index
   bits, so the non-cacheable window at 0xBFFFF800 aliases onto exactly those
   bytes and no section may be placed there. */
MEMORY {
  L2 : ORIGIN = 0x80000000, LENGTH = NRL2BANKS * 1048576 * ALIGNMENT - 2048
}

/*
  Ara's TB works only if the sections are aligned to a AxiWideBeWidth boundary
*/
SECTIONS {
  .text : {
    *(.text.init)
    *(.text)
    *(.text.*)
  } > L2

  .data : ALIGN(ALIGNMENT) {
    *(.data)
    *(.data.*)
  } > L2

  .rodata  : ALIGN(ALIGNMENT) { *(.rodata .rodata.* .gnu.linkonce.r.*) } > L2
  .rodata1 : ALIGN(ALIGNMENT) { *(.rodata1) } > L2
  .sdata2  : ALIGN(ALIGNMENT) {
    *(.sdata2 .sdata2.* .gnu.linkonce.s2.*)
  } > L2

  .sdata   : ALIGN(ALIGNMENT) {
    __global_pointer$ = . + 0x800;
    *(.srodata.cst16) *(.srodata.cst8) *(.srodata.cst4) *(.srodata.cst2) *(.srodata .srodata.*)
    *(.sdata .sdata.* .gnu.linkonce.s.*)
  } > L2

  .bss : ALIGN(ALIGNMENT) {
    __bss_start = .;
    *(.bss)
    *(.sbss .sbss.* .sbss2 .sbss2.* .gnu.linkonce.sb2.*);
    __bss_end = .;
  } > L2

  .l2 : ALIGN(ALIGNMENT) {
    *(.l2)
    timer = .;
    . = . + 0x8;
    l2_alloc_base = ALIGN(ALIGNMENT);
  } > L2

  .comment : ALIGN(ALIGNMENT) { *(.comment) } > L2

  /* In non-cacheable region of main memory */
  sync_flag              = 0xBFFFF800;
  sync_generation        = 0xBFFFF804;
  /* to later use l2 alloc base within applications to allocate the following atomic pointers */
  canneal_keep_going_global_flag        = 0xBFFFF808;
  streamcluster_pspeedy_cond_generation = 0xBFFFF810;
  streamcluster_pspeedy_i               = 0xBFFFF814;
  printf_lock                           = 0xBFFFF818;
  mutex_lock                            = 0xBFFFF840;
  l2_atomic_alloc_base                  = 0xBFFFF848;

  eoc_address_reg        = 0xD0000000;
  dram_start_address_reg = 0xD0000008;
  dram_end_address_reg   = 0xD0000010;
  event_trigger          = 0xD0000018;
  hw_cnt_en_reg          = 0xD0000020;

  fake_uart              = 0xC0000000;
}
