#include <display.h>
#include <stdio.h>
#include <stdarg.h>

void display_msg(t_ast_clock_tick date, const t_decoded * data)
{
  log_message("[%4u] %s \n", (unsigned int) date, data->values);
}

void display_error(t_ast_clock_tick date)
{
  log_message("[%4u] ******** ERROR\n", (unsigned int) date);
}

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
