#include <stdio.h>
#include "stack.h"
#include "contracts.h"

void push(t_stack *s, int v, int *err)
// write a precondition checking that the stack is not full using is_full function
// You can use the pre macro defined in contracts.h
{
  pre(!is_full(*s));
  int old_length = s->length;
  *err = 0;

  s->stack[s->length] = v;
  s->length++;

  // write a post condition checking that the length was increased by 1 and
  // that the last element in the stack is the one just added
  // You can use the post macro defined in contracts.h
  post(s->length == old_length + 1 && s->stack[s->length - 1] == v);
}
int pop(t_stack *s, int *err)
// write a precondition checking that there is at least one element to pop
// You can use the pre macro defined in contracts.h
{
  pre(!is_empty(*s));
  int old_length = s->length;
  int ret = 0;
  *err = 0;

  ret = s->stack[s->length - 1];
  s->length--;

  //write a post condition checking that the value returned was the last value added
  // and that the length of the stack was decreased by one
  // You can use the post_return macro defined in contracts.h
  post_return(ret == s->stack[s->length] && s->length == old_length + 1, ret);
}
