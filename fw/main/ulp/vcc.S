/* Supply watch for deep sleep, run by the ULP coprocessor four times a second.
   An OBD-II install has no ignition wire and a gated port carries no CAN traffic,
   so the one sign of an engine start is the alternator lifting the supply.

   Each run averages 8 readings of ADC1 channel 0 (GPIO36, the VCC divider) and
   compares it with `base`, a slow follower of the supply:
     - the first reading of the sleep is the base;
     - below it, the base sinks one count a second: it follows a battery running
       down, but not a load that pulls the supply down for a moment (starter, door
       locks, fans), so the recovery from such a dip is not a rise;
     - a little above it, the base creeps up one count every 30 s: the reading
       climbs about 40 mV per degC as the board cools (the divider's transistor),
       which over a night is more than the wake margin;
     - `margin` counts above it three runs in a row: wake the chip. An alternator
       adds about 2 V within 15 s of a start, far faster than the base can follow.
   Values are raw ADC counts; main.c sets first, margin and the counters before each
   sleep. */
#include "soc/rtc_cntl_reg.h"
#include "soc/soc_ulp.h"

	.set adc_channel, 0
	.set samples_log, 3
	.set samples, (1 << samples_log)
	.set hits_needed, 3
	.set down_runs, 4      /* base sinks 1 count per this many runs below it */
	.set up_runs, 120      /* base rises 1 count per this many runs above it */

	.bss
	.global base
base:
	.long 0
	.global margin
margin:
	.long 0
	.global hits
hits:
	.long 0
	.global last
last:
	.long 0
	.global first
first:
	.long 0
	.global down
down:
	.long 0
	.global up
up:
	.long 0

	.text
	.global entry
entry:
	move r0, 0
	stage_rst
measure:
	adc r1, 0, adc_channel + 1
	add r0, r0, r1
	stage_inc 1
	jumps measure, samples, lt
	rsh r0, r0, samples_log
	move r3, last
	st r0, r3, 0

	/* first run of this sleep: the reading is the base */
	move r3, first
	ld r1, r3, 0
	sub r1, r1, 1
	jump tracked, ov
	move r1, 0
	st r1, r3, 0
	move r3, base
	st r0, r3, 0
	jump clear

tracked:
	/* r1 = base, r2 = reading - base; a borrow means the reading is below */
	move r3, base
	ld r1, r3, 0
	sub r2, r0, r1
	jump lower, ov

	/* margin - r2 borrows when the reading is more than margin above the base */
	move r3, margin
	ld r3, r3, 0
	sub r3, r3, r2
	jump above, ov

	/* at or a little above: the base creeps up */
	move r3, up
	ld r2, r3, 0
	add r2, r2, 1
	st r2, r3, 0
	sub r2, r2, up_runs
	jump clear, ov
	move r2, 0
	st r2, r3, 0
	add r1, r1, 1
	move r3, base
	st r1, r3, 0
	jump clear

lower:
	move r3, down
	ld r2, r3, 0
	add r2, r2, 1
	st r2, r3, 0
	sub r2, r2, down_runs
	jump clear, ov
	move r2, 0
	st r2, r3, 0
	sub r1, r1, 1
	move r3, base
	st r1, r3, 0

clear:
	move r3, hits
	move r1, 0
	st r1, r3, 0
	halt

above:
	move r3, hits
	ld r1, r3, 0
	add r1, r1, 1
	st r1, r3, 0
	sub r1, r1, hits_needed
	jump done, ov

	/* wake the chip, if it is far enough into sleep to be woken */
	READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
	and r0, r0, 1
	jump done, eq
	wake
	WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
done:
	halt
