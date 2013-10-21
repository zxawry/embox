/**
 * @file
 *
 * @date 21.10.2013
 * @author Eldar Abusalimov
 */

#ifndef UTIL_BITMAP_H_
#define UTIL_BITMAP_H_

#include <limits.h>  /* LONG_BIT */

#define BITMAP_DECL(name, nbits) \
	unsigned long name[BITMAP_SIZE(nbits)];

#define BITMAP_SIZE(nbits)  (((nbits) + LONG_BIT - 1) / LONG_BIT)
#define BITMAP_OFFSET(bit)  ((bit) / LONG_BIT)
#define BITMAP_SHIFT(bit)   ((bit) % LONG_BIT)
#define BITMAP_ROUND(bit)   ((bit) - BITMAP_SHIFT(bit))
#define BITMAP_MASK(bit)    (0x1ul << BITMAP_SHIFT(bit))

static inline void bitmap_set_bit(unsigned long *bitmap, unsigned int bit)
{
	bitmap[BITMAP_OFFSET(bit)] |= BITMAP_MASK(bit);
}

static inline void bitmap_clear_bit(unsigned long *bitmap, unsigned int bit)
{
	bitmap[BITMAP_OFFSET(bit)] &= ~BITMAP_MASK(bit);
}

static inline void bitmap_toggle_bit(unsigned long *bitmap, unsigned int bit)
{
	bitmap[BITMAP_OFFSET(bit)] ^= BITMAP_MASK(bit);
}

static inline int bitmap_test_bit(const unsigned long *bitmap, unsigned int bit)
{
	return 0x1ul & (bitmap[BITMAP_OFFSET(bit)] >> BITMAP_SHIFT(bit));
}

extern unsigned int bitmap_find_set_bit(const unsigned long *,
		unsigned int nbits, unsigned int offset);
extern unsigned int bitmap_find_zero_bit(const unsigned long *,
		unsigned int nbits, unsigned int offset);


#endif /* UTIL_BITMAP_H_ */

