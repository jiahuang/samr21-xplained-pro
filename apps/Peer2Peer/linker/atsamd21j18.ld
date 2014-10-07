/**/
MEMORY
{
  flash (rx) : ORIGIN = 0x00000000, LENGTH = 0x40000 /* 256k */
  ram  (rwx) : ORIGIN = 0x20000000, LENGTH = 0x8000 /* 32k */
}

__top_flash = 0x40000;
__top_ram = 0x20000000 + 0x8000;

ENTRY(HAL_IrqHandlerReset)

SECTIONS
{
  .text : ALIGN(4)
  {
    FILL(0xff)
    KEEP(*(.vectors))
    *(.text*)
    *(.rodata)
    *(.rodata.*)
    . = ALIGN(4);
  } > flash

  . = ALIGN(4);
  _etext = .;

  .uninit_RESERVED : ALIGN(4)
  {
    KEEP(*(.bss.$RESERVED*))
  } > ram

  .data : ALIGN(4)
  {
    FILL(0xff)
    _data = .;
    *(vtable)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > ram AT > flash

  .bss : ALIGN(4)
  {
    _bss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    PROVIDE(_end = .);
  } > ram

  PROVIDE(_stack_top = __top_ram - 0);
}

