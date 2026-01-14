#ifndef FONT5X7_H
#define FONT5X7_H

/*
 * 5 x 7 ASCII font.
 *
 * Each byte here has one "column" of the character
 * This minimizes the number of wasted bits (rather
 * than 7 bytes of 5 bit 'lines' it is 5 bytes of
 * 7 bit 'columns'.
 */

/* Initially downloaded form the repository: https://github.com/ChuckM/bb-lcd/blob/master/font-5x7.c */

#define FONT_CHAR_HEIGHT 7
#define FONT_CHAR_WIDTH 5

/* Generic to compute font bit on/off for this font */
#define FONT_BIT(c, column, row) \
  ((font_data[(c)+(column)] & 1<<(7-(row))) != 0)


extern const unsigned char font_data[1275u];

#endif // FONT5X7_H
