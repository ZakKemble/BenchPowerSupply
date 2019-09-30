/*
 * Project: Fan Controller for Bench Power Supply
 * Author: Zak Kemble, contact@zakkemble.net
 * Copyright: (C) 2019 by Zak Kemble
 * License: 
 * Web: https://blog.zakkemble.net/laptop-bench-power-supply/
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define FAN_OVERRIDE_NONE	0
#define FAN_OVERRIDE_ON		1
#define FAN_OVERRIDE_OFF	2

#define FAN_COOL_TIME		125 // 125 * 64ms = 8s // Keep fan on for this much longer after COOL_VAL has been reached
#define TEMP_MEASURE_INTERVAL	32 // 32 * 64ms = 2s // How often to measure temperature

// Temperature sensor resistance is 10k @ 22C
// Sensor resistance goes down as temperature goes up
// Pullup resistance is around 40k
// Lower value = hotter
#define HOT_VAL				23 // Sense: 4k @ ??C
#define COOL_VAL			27

#define WDT_INT_RESET()		(WDTCSR |= _BV(WDIE)|_BV(WDE)) // NOTE: Setting WDIE also enables global interrupts
#define WDT_TIMEDOUT() (!(WDTCSR & _BV(WDIE)))

#define FAN_ON() (PORTB |= _BV(PORTB0))
#define FAN_OFF() (PORTB &= ~_BV(PORTB0))
#define BTN_ISPRESSED() (!(PINB & _BV(PINB1)))

// PB0 = Fan out
// PB1 = Switch in
// PB2 = Temp sense ADC

static uint8_t rstflr_mirror __attribute__((section(".noinit,\"aw\",@nobits;"))); // BUG: https://github.com/qmk/qmk_firmware/issues/3657

void get_rstflr(void) __attribute__((naked, used, section(".init3")));
void get_rstflr()
{
	rstflr_mirror = RSTFLR;
	RSTFLR = 0;

	//wdt_disable(); 
	wdt_enable(WDTO_60MS);
}

int main(void)
{
	clock_prescale_set(CPU_DIV);

	DDRB |= _BV(DDB0);
	PUEB |= _BV(PUEB1);

	ACSR = _BV(ACD); // Power off analogue comparator

	// Setup ADC
	ADMUX = _BV(MUX1);
	ADCSRA = _BV(ADIE)|_BV(ADPS2)|_BV(ADPS0);
	DIDR0 = _BV(ADC2D);

	power_all_disable(); // Power off everything else

	uint8_t now = 0;
	uint8_t lastMeasureTemp = 0;
	uint8_t lastHotTime = (0 - FAN_COOL_TIME) + 31; // Turn fan on for 2 seconds at power on
	uint8_t hot = 0;
	uint8_t btnIsPressed = 0;
	uint8_t fanOverride = FAN_OVERRIDE_NONE;
	
	sei();

	// Was reset by the watchdog (mainly for debugging)
	if(rstflr_mirror & _BV(WDRF))
	{
		while(1)
		{
			// _delay_ms() is giving some asm error (avr-libc bug? GCC 8.3.0)
			for(uint16_t i=0;i<20000;i++)
			{
				WDT_INT_RESET();
				_delay_us(100);
			}
			//_delay_ms(2000);
		
			FAN_ON();

			// _delay_ms() is giving some asm error (avr-libc bug? GCC 8.3.0)
			for(uint16_t i=0;i<5000;i++)
			{
				WDT_INT_RESET();
				_delay_us(100);
			}
			//_delay_ms(500);
		
			FAN_OFF();
		}
	}

	WDT_INT_RESET();

	while(1)
	{
		// Timer stuff, increments every 64ms from the WDT
		// Also turns the WDT back on
		if(WDT_TIMEDOUT())
		{
			WDT_INT_RESET();
			now++;
		}

		if((uint8_t)(now - lastMeasureTemp) >= TEMP_MEASURE_INTERVAL)
		{
			// Pullup enable
			PUEB |= _BV(PUEB2);

			lastMeasureTemp = now;

			// Wait for a bit for voltage to stabilize
			// _delay_ms() is giving some asm error (avr-libc bug? GCC 8.3.0)
			for(uint8_t i=0;i<10;i++)
				_delay_us(100);
			//_delay_ms(1);

			// Do an ADC convertion
			power_adc_enable();
			ADCSRA |= _BV(ADEN)|_BV(ADSC);
			set_sleep_mode(SLEEP_MODE_ADC);
			sleep_mode();
			loop_until_bit_is_clear(ADCSRA, ADSC); // In case we wakeup from another interrupt before the convertion completes
			uint8_t val = ADCL;
			ADCSRA &= ~_BV(ADEN);
			power_adc_disable();

			// Pullup disable
			PUEB &= ~_BV(PUEB2);

			if(val > COOL_VAL)
				hot = 0;
			else if(val < HOT_VAL)
				hot = 1;
			// else we're between the hot and cool values (hysteresis)

			if(hot)
				lastHotTime = now;
		}

		// Don't need to bother with switch debouncing here since we only loop every ~64ms from the WDT
		if(BTN_ISPRESSED() && !btnIsPressed)
		{
			btnIsPressed = 1;

			if(fanOverride == FAN_OVERRIDE_NONE)
			{
				fanOverride = FAN_OVERRIDE_ON;
				FAN_ON();
			}
			else if(fanOverride == FAN_OVERRIDE_ON)
			{
				fanOverride = FAN_OVERRIDE_OFF;
				FAN_OFF();
			}
			else
			{
				fanOverride = FAN_OVERRIDE_NONE;
				if(!hot)
					lastHotTime = now - FAN_COOL_TIME;
			}
		}
		else if(!BTN_ISPRESSED() && btnIsPressed)
			btnIsPressed = 0;

		if(fanOverride == FAN_OVERRIDE_NONE)
		{
			if(hot || (uint8_t)(now - lastHotTime) < FAN_COOL_TIME)
				FAN_ON();
			else
			{
				FAN_OFF();
				lastHotTime = now - FAN_COOL_TIME;
			}
		}

		// Sleep if nothing to do
		cli();
		if(!WDT_TIMEDOUT())
		{
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			sleep_enable();
			//sleep_bod_disable();
			sei();
			sleep_cpu();
			sleep_disable();
		}
		sei();
	}
}

EMPTY_INTERRUPT(WDT_vect);
EMPTY_INTERRUPT(ADC_vect);
