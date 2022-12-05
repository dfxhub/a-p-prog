#ifndef __ASMPROCS_H__
#define __ASMPROCS_H__

#ifndef F_CPU
#warning DEFINE F_CPU
#define F_CPU 16000000L
#endif

#define CYCLES_PER_US (F_CPU/1000000)
#define C4PUS (CYCLES_PER_US/4)
#define DVUS(x) (C4PUS*x)

extern uint16_t swap16(uint16_t value);
extern void _delay_us_asm(uint16_t us);
extern void as_i2c_start(void);

extern uint8_t _rand_lfsr(uint8_t sr);
extern uint8_t _rand_marv(uint8_t sr);
extern uint8_t _rand_galo(uint8_t sr);

#define _delay_us(x) _delay_us_asm(DVUS(x))

#endif