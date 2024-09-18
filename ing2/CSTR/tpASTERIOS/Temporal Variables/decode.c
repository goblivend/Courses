#include <decode.h>

static char decode_4_bits(const t_input_data *const input,
                         unsigned int index)
{
  unsigned int raw = ((*input) >> (32 - (index * 4)));
  raw = (raw & 0xf);

  return (raw == 0xf ? (char)0 : (char)(raw + 97));
}

void decode_msg(const t_input_data *const input, t_decoded *const output)
{
  unsigned int i;
  for (i = 0u; i < 8u; ++i)
  {
    output->values[i] = decode_4_bits(input, i + 1);
  }
  output->raw_value = *input;
}
