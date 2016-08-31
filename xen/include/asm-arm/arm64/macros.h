#ifndef __ASM_ARM_ARM64_MACROS_H
#define __ASM_ARM_ARM64_MACROS_H

/*
 * Emit a 64-bit absolute little endian symbol reference in a way that
 * ensures that it will be resolved at build time, even when building a
 * PIE binary. This requires cooperation from the linker script, which
 * must emit the lo32/hi32 halves individually.
 */
    .macro	le64sym, sym
    .long	\sym\()_lo32
    .long	\sym\()_hi32
    .endm

#endif /* __ASM_ARM_ARM32_MACROS_H */
