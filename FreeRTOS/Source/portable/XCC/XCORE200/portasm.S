// Copyright (c) 2019, XMOS Ltd, All rights reserved

#include "rtos_support_rtos_config.h"

.globl kexcept
.align 128              /* align the kernel section to 128 bytes */
.type  kexcept,@function
.issue_mode dual
.cc_top kexcept.function, kexcept
kexcept:
  ldc r11, 0x0004
  shl r11, r11, 16
  ldc r9, 0x0080
  or  r11, r11, r9
  bau r11 //_TrapHandler is at 0x00040080. TODO: Is it always? Why can't I access the symbol _TrapHandler?

.align 64
kcall:
  /* start saving the thread's context */
  extsp RTOS_SUPPORT_INTERRUPT_STACK_GROWTH
  stw    r10,    sp[18]
  stw    r11,    sp[19]

  /* kcall sets SPC to the instruction of the kcall rather than the next instruction */
  /* so we need to adjust the SPC value that we save to the stack: */
  stw    spc,    sp[1]   /* save the saved program counter onto the stack... */
  ldw    r10,    sp[1]   /* so that we can load it into r10 (which we have already saved). */
  add    r10, r10, 4     /* Add 4 to the spc to make it point to the instruction after the kcall */
 {stw    r10,    sp[1]   /* Now save it to the stack */

  /* KCALLs use the same common function as interrupt callbacks. */
  /* we want it to call pxKernelCallHandler as the root function. */
  ldap   r11,    pxKernelCallHandler}
 {mov    r10,     r11

  /* jump into _freertos_interrupt_callback_common but just past */
  /* the instruction that saves the SPC since it was saved above */
  bu _freertos_kcall_save_ctx_continue}

.globl rtos_interrupt_callback_common
rtos_interrupt_callback_common:
  /* This is the body of the RTOS _xcore_c_interrupt_callback_XXX functions. */
  /* r10 = interrupt_callback_t function */

  /* Save the thread's context onto the thread's stack. */
  /* The stack was extended for this by the wrapper function. */
  stw    spc,    sp[1]
_freertos_kcall_save_ctx_continue:
  stw    ssr,    sp[2]
  stw    sed,    sp[3]
  stw    et,     sp[4]
  stw    dp,     sp[5]
  stw    cp,     sp[6]
  stw    lr,     sp[7]
  stw    r0,     sp[8]
  stw    r1,     sp[9]
  stw    r2,     sp[10]
  stw    r3,     sp[11]
  stw    r4,     sp[12]
  stw    r5,     sp[13]
  stw    r6,     sp[14]
  stw    r7,     sp[15]
  stw    r8,     sp[16]
 {stw    r9,     sp[17]
/*stw    r10,    sp[18]     already saved by the wrapper function. */
/*stw    r11,    sp[19]     already saved by the wrapper function. */

  ldaw   r1,     sp[0]}  /* Get value of current stackpointer... */
  ldw    r0,     dp[pxCurrentTCB]
  stw    r1,     r0[0x0] /* and save it to the first word (top of stack) in the current TCB */

 {kentsp 0               /* switch to the kernel stack. */
                         /* The value 0 is safe to use since we don't need the SP */
                         /* saved to KSP[0] as it will be saved by the callback. */

  get    r11,    ed}     /* Get the event data... */
 {mov    r0,     r11     /* into the first argument for the callback function... */
  bla    r10}            /* and call the callback function. */

  ldap   r11,    vPortSwitchContext
  bla    r11

//krestsp 0              /* actually unnecessary since KSP is already set and the SP */
                         /* is being restored next from the current TCB. */

.globl _freertos_restore_ctx
_freertos_restore_ctx:

  ldw    r0,     dp[pxCurrentTCB]
  ldw    r0,     r0[0x0] /* Get the top of the stack from the current TCB... */
  set    sp,     r0;     /* into the stack pointer register. */

  /* Restore the state */
  ldw    spc,    sp[1]
  ldw    ssr,    sp[2]
  ldw    sed,    sp[3]
  ldw    et,     sp[4]
  ldw    dp,     sp[5]
  ldw    cp,     sp[6]
  ldw    lr,     sp[7]
  ldw    r0,     sp[8]
  ldw    r1,     sp[9]
  ldw    r2,     sp[10]
  ldw    r3,     sp[11]
  ldw    r4,     sp[12]
  ldw    r5,     sp[13]
  ldw    r6,     sp[14]
  ldw    r7,     sp[15]
  ldw    r8,     sp[16]
  ldw    r9,     sp[17]
  ldw    r10,    sp[18]
 {ldw    r11,    sp[19]

  /* shrink the stack by the size of the context just restored */
  ldaw   sp,     sp[RTOS_SUPPORT_INTERRUPT_STACK_GROWTH]}

  kret                   /* exit kernel mode and return to the thread */

.cc_bottom kexcept.function

