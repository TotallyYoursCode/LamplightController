#ifndef MY_MACROSES_H_
#define MY_MACROSES_H_

//-----------------------------------MACROSES----------------------------------------
#define PRAGMA(x) _Pragma(#x)
#define INLINE static inline __attribute__ ((always_inline)) 
// Generally, functions are not inlined unless optimization is specified. For functions declared inline, this attribute inlines the function even if no optimization level is specified. 

#define EnableInterrupts() sei()

#define ATOMIC_SECTION_ENTER   { uint8_t __atomic = SREG; cli();
#define ATOMIC_SECTION_LEAVE   SREG = __atomic; }

#define invbit(p,n) (p=p^(1<<(n)))				// инвертирование бита   PORTD = PORTD^(1<<PIND1); исключающее или
#define setbit(p,n) (p|=(1<<(n)))				// ”становить бит
#define clrbit(p,n) (p&=~(1<<(n)))				// —бросить   бит

// Typed memory access macro
#define MMIO_REG(mem_addr, type) (*(volatile type *)(mem_addr))
//------------------------------------------------------------------------------------

//--------------------------------GPIO------------------------------------------------
#define GPIO_PIN(name, port, bit) \
INLINE void    GPIO_##name##_in()       { DDR##port &= ~(1 << bit); PORT##port &= ~(1 << bit); } \
INLINE void    GPIO_##name##_pullup()   { PORT##port |= (1 << bit); } \
INLINE void    GPIO_##name##_out()      { DDR##port |= (1 << bit); } \
INLINE void    GPIO_##name##_set()      { PORT##port |= (1 << bit); } \
INLINE void    GPIO_##name##_clr()      { PORT##port &= ~(1 << bit); } \
INLINE void    GPIO_##name##_toggle()   { PORT##port ^= (1 << bit); } \
INLINE uint8_t GPIO_##name##_read()     { return (PIN##port & (1 << bit)) != 0; } \
INLINE uint8_t GPIO_##name##_state()    { return (DDR##port & (1 << bit)) != 0; }
//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------



#endif /* MY_MACROSES_H_ */