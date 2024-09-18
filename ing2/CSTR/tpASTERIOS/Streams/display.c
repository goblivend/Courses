#include <display.h>
#include <stdio.h>
#include <stdarg.h>

/*
 * Simple logger function implemented using printf.
 * The call to fflush() forces the message to be printed right away.
 */
void log_message(const char *format, ...)
{
  va_list args;
  va_start (args, format);
  vprintf(format, args);
  fflush(stdout);
  va_end (args);
}
