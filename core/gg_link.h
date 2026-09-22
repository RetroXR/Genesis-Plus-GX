/*
 * Hooks through which a frontend carries the Game Gear's EXT port (the
 * Gear-to-Gear cable). All NULL unless a frontend installs them, so a build
 * without a link driver runs exactly as before.
 */
#ifndef _GG_LINK_H_
#define _GG_LINK_H_

/* Start of each scanline of a Master System / Game Gear frame, with the frame's
 * master-cycle count so far. May block: it is the rendezvous with the far end. */
extern void (*gg_link_line)(unsigned int cycles);

/* End of the frame, with the master cycles it ran. */
extern void (*gg_link_frame)(unsigned int cycles);

/* After a write to Game Gear port $00-$06 has landed in io_reg. */
extern void (*gg_link_write)(unsigned int offset, unsigned int data);

/* A read of Game Gear port $00-$06: given what the core would return, returns
 * what the far end makes of it. */
extern unsigned int (*gg_link_read)(unsigned int offset, unsigned int value);

#endif
