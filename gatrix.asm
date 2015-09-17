; Gatrix 0.1 alpha

#include "8515def.inc"

; device ant timing settings
#define DEVICE_FREQ_HZ 4000000
#define TICKS_HZ 1000 

#define TIMER_COMPARE_CLOCK_CYCLES 400
;.equ TIMER_COMPARE_CLOCK_CYCLES = DEVICE_FREQ_HZ/TICK_HZ

#define CONTEXT_SIZE 33
#define THREAD_STACK_SIZE 64
#define SP_SIZE 2
#define MAX_THREADS 5
#define TS_SIZE 1
#define K_TH_S_UNUSED_MIN_SIZE 32

#define TH_SP_AREA_SIZE SP_SIZE * MAX_THREADS
#define TS_AREA_SIZE MAX_THREADS * TS_SIZE

#define CURRENT_THREAD_ADDR RAMEND
#define NUM_RUNNING_THREAD_ADDR RAMEND - TS_SIZE
#define TH_SP_AREA_ADDR RAMEND - TS_SIZE - TH_SP_AREA_SIZE
#define TS_AREA_ADDR RAMEND - TS_SIZE - TH_SP_AREA_SIZE  - TS_AREA_SIZE

#if (TS_AREA_SIZE+TS_SIZE+TH_SP_AREA_SIZE + TS_AREA_SIZE) > (THREAD_STACK_SIZE -  K_TH_S_UNUSED_MIN_SIZE )
#error "Bad settings: not enouth memor to kernel stack"
#endif

#define THREAD_STATUS_RUNNING 1
#define THREAD_STATUS_NONE  0

; macros
.macro mSave_Context
	push r0
	in r0, SREG
	push r0
	push r1
	push r2
	push r3
	push r4
	push r5
	push r6
	push r7
	push r8
	push r9
	push r10
	push r11
	push r12
	push r13
	push r14
	push r15
	push r16
	push r17
	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
	push r24
	push r25
	push r26
	push r27
	push r28
	push r29
	push r30
	push r31
.endmacro	

.macro mRestore_Context
	pop r31
	pop r30
	pop r29
	pop r28
	pop r27
	pop r26
	pop r25
	pop r24
	pop r23
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
	pop r15
	pop r14
	pop r13
	pop r12
	pop r11
	pop r10
	pop r9
	pop r8
	pop r7
	pop r6
	pop r5
	pop r4
	pop r3
	pop r2
	pop r1
	pop r0
	out SREG, r0
	pop r0
.endmacro


.macro mSet_X_to_TS_AREA
	ldi XL, low(TS_AREA_ADDR)
	ldi XH, high(TS_AREA_ADDR)
.endmacro

.macro mSet_X_to_TH_SP_AREA
	ldi XL, low(TH_SP_AREA_ADDR)
	ldi XH, high(TH_SP_AREA_ADDR)
.endmacro

.macro mSet_X_to_TH_NUM
	ldi XL, low(NUM_RUNNING_THREAD_ADDR)
	ldi XH, high(NUM_RUNNING_THREAD_ADDR)
.endmacro

.macro mSet_X_to_CURRENT_THREAD
	ldi XL, low(CURRENT_THREAD_ADDR)
	ldi XH, high(CURRENT_THREAD_ADDR)
.endmacro

.org 0x0
	rjmp init

.org OC1Aaddr
	rjmp OC1A_vec

init:
;set kernel thread SP
	ldi r16, low(TS_AREA_ADDR-1)
	out SPL, r16
	ldi r16, high(TS_AREA_ADDR-1)
	out SPH, r16
	
	rjmp kernel


OC1A_vec:
	nop
	msave_context
	rcall determine_next_task
	sts CURRENT_THREAD_ADDR, r16
	out SPL, r17
	out SPH, r18
	mrestore_context

;	pop ZL
;	pop ZH
;	ijmp

	reti

determine_next_task:
	; get current running task
	; increment it (if end of task reached - start from begining)
	; check status of this
	; if next task isn't running increment task
	; loop until running task reached
	; then store SPH:SPL in r17:r18 and next task to r16
	; return to interupt vector

	mSet_X_to_CURRENT_THREAD

	ld r16, X
	clr r17
	ldi r19, THREAD_STATUS_RUNNING

next_task_loop:
	inc r16
	cpi r16, MAX_THREADS
	breq first_task

	mSet_X_to_TS_AREA

	; calculate next task status address
	add XL, r16 
	adc XH, r17

	; getting next status
	ld r18, X
	cp r18, r19
	brne next_task_loop

	;found next task! geting SP
	; r18 now contain next thread number
	mSet_X_to_TH_SP_AREA

;multiply twice	
	add XL, r16
	adc XH, r17

	add XL, r16
	adc XH, r17

	; setting SP to registers
	ld r17, X+
	ld r18, X
	
	; return to interrupt vector
	ret

first_task:
	; hack!!! inc(0xff) = 0!
	ser r16
	rjmp next_task_loop

	
run_thread:
; get number of running threads
; check if max isn't reached
; if reached - do nothing
; else inrement number of running threads,
; alocate space to stack, set TS slot to running
; place entry point ant zeroed SREG on top stack
; decrement stack pointer by context size
; status returned in r0 register


;get number of current threads
	mSet_X_to_TH_NUM
	ld r16, X
	cpi r16, MAX_THREADS
	breq too_match_tasks
	
	inc r16
; r16 now contain num running threads
	st X, r16

; search for free slot in thread table
	mSet_X_to_TS_AREA
	ser r17

_run_thread_search_next:
	ld r16, X+
	inc r17
	cpi r16, THREAD_STATUS_NONE
	brne _run_thread_search_next

; store status "running" to the slot
	ldi r16, THREAD_STATUS_RUNNING
	st -X, r16
	
;calculate SP
; r17 - now contains task number
; SP = RAMEND-((TASK_NUM+1)*STACK_SIZE) -1
	mov r18, r17

	ldi r20, THREAD_STACK_SIZE
	ldi r21, 0

	ldi XL, low(RAMEND)
	ldi XH, high(RAMEND)

	inc r18
	inc r18
		
_run_thread_calc_SP:
	subi XL, THREAD_STACK_SIZE
	sbc XH, r21
	dec r18
	tst r18
	brne _run_thread_calc_SP

	clr r16
	st -X, ZL
	st -X, ZH
	st -X, r16

_run_thread_calc_SP_end:
; campute true stack position
	subi XL, CONTEXT_SIZE
	sbc XH, r21

; save X to Y
	mov YL, XL
	mov YH, XH

	mSet_X_to_TH_SP_AREA

	mov r18, r17
	ldi r19, SP_SIZE

_run_thread_calc_SP_PL_off:
	tst r18
	breq _run_thread_calc_SP_PL_off_end
	add XL, r19
	adc XH, r21
	dec r18
	rjmp _run_thread_calc_SP_PL_off

_run_thread_calc_SP_PL_off_end: 
; store SP to SP table
	st X+, YL
	st X, YH

	clr r0
	inc r0
	rjmp end_creating_tasks

too_match_tasks:
	clr r0

end_creating_tasks:
	ret


thread_yield:
; disable interrupts
	sei
	rjmp OC1A_vec
	; never reached

kernel:
; clear last 255 SRAM bytes to 0xff
; only for debuging purposes
;	ser r16
;	ser r17
;	ldi XL, low(RAMEND)
;	ldi XH, high(RAMEND)
;_kernel_fill_mem_loop:
;	st -X, r17
;	dec r16
;	tst r16
;	brne _kernel_fill_mem_loop

; preparing form TS AREA initialization 
	mSet_X_to_TS_AREA
	ldi r17, THREAD_STATUS_NONE
	ldi r16, MAX_THREADS


; loop until fill all TS AREA
_kernel_TS_INITIALIZE_loop:
	st X+, r17
	dec r16
	tst r16
	brne _kernel_TS_INITIALIZE_loop

;set current running threads to 0
	mSet_X_to_TH_NUM
	clr r16
	st X, r16

; in this consepts - task must be registered before 
; multhreading will be started

; registering task to start
	ldi ZL, low(thread0)
	ldi ZH, high(thread0)
	rcall run_thread

; registering task to start
	ldi ZL, low(thread1)
	ldi ZH, high(thread1)
	rcall run_thread
	
start_multithreading:

;set tick count
	ldi r16, low(TIMER_COMPARE_CLOCK_CYCLES)
	out OCR1AL, r16
	ldi r16, high(TIMER_COMPARE_CLOCK_CYCLES)
	out OCR1AH, r16

	ldi r16, (1 << COM1A1)
	out TCCR1A, r16
	ldi r16, (1 << CS10) + (1 << CTC1)
	out TCCR1B, r16

	ldi r16, (1 << OCIE1A)
	out TIMSK, r16

; launching the ROCKet!
; BOOOOOM!
	sei

_kernel_master_loop:
; loop loop loop and one more time - LOOOOOOP!
	rcall thread_yield
	rjmp _kernel_master_loop

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; threads

thread0:
	clr r0
	clr r1
	clr r2
	clr r3
	clr r4
	clr r5
	clr r6
	clr r7
	clr r8
	clr r9
	clr r10
	clr r11
	clr r12
	clr r13
	clr r14
	clr r15
	clr r16
	clr r17
	clr r18
	clr r19
	clr r20
	clr r21
	clr r22
	clr r23
	clr r24
	clr r25
	clr r26
	clr r27
	clr r29
	clr r30
	clr r31
_thread0_loop0:
	inc r0
	inc r1
	inc r2
	inc r3
	inc r4
	inc r5
	inc r6
	inc r7
	inc r8
	inc r9
	inc r10
	inc r11
	inc r12
	inc r13
	inc r14
	inc r15
	inc r16
	inc r17
	inc r18
	inc r19
	inc r20
	inc r21
	inc r22
	inc r23
	inc r24
	inc r25
	inc r26
	inc r27
	inc r28
	inc r29
	inc r30
	inc r31
	rjmp _thread0_loop0


thread1:
	ser r16
	ser r17
	ser r18
	ser r19
	ser r20
	ser r21
	ser r22
	ser r23
	ser r24
	ser r25
	ser r26
	ser r27
	ser r29
	ser r30
	ser r31
	mov r31, r0
	mov r31, r1
	mov r31, r2
	mov r31, r3
	mov r31, r4
	mov r31, r5
	mov r31, r6
	mov r31, r7
	mov r31, r8
	mov r31, r9
	mov r31, r10
	mov r31, r11
	mov r31, r12
	mov r31, r13
	mov r31, r14
	mov r31, r15
_thread1_loop0:
	inc r0
	inc r1
	inc r2
	inc r3
	inc r4
	inc r5
	inc r6
	inc r7
	inc r8
	inc r9
	inc r10
	inc r11
	inc r12
	inc r13
	inc r14
	inc r15
	inc r16
	inc r17
	inc r18
	inc r19
	inc r20
	inc r21
	inc r22
	inc r23
	inc r24
	inc r25
	inc r26
	inc r27
	inc r28
	inc r29
	inc r30
	inc r31
	rjmp _thread1_loop0

;; end of kernel


	