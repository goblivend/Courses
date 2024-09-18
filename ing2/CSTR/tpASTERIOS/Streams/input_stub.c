#include <common.h>

static const char raw_text[] =
  "Lorem ipsum dolor sit amet, consectetur adipiscing elit.\nDonec non "
  "ultricies nisi.\nSuspendisse vel nunc mollis, tempor orci a, fermentum "
  "nisi.\nFusce mollis nisl sed urna viverra, ut tempus neque dapibus.\n"
  "Proin elementum aliquet mi, vel luctus mi tincidunt in.\nFusce in tempus "
  "tortor, ac fringilla orci.\nPraesent feugiat vitae sapien ut dapibus.\n"
  "Curabitur sed faucibus velit.\nIn eu dapibus risus.\nPellentesque luctus "
  "semper libero in scelerisque.\nInteger quis elit sed ipsum sollicitudin "
  "malesuada.\nDonec vitae erat sem.\n";


static void _fill_info (t_input_data info,
                        const unsigned int cursor,
                        const unsigned int limit)
{
  unsigned int i;
  if (cursor >= limit)
  {
    info[0] = '\0';
    return;
  }

  for (i = 0u; i < MSG_SIZE; ++i)
  {
    if (cursor + i >= limit)
    {
      info[i] = '\0';
      return;
    }
    info[i] = raw_text[cursor + i];
  }
}

void get_part0_msg(t_input_data info)
{
  static unsigned int cursor0 = 0u;
  _fill_info(info, cursor0, (DISPLAY_MSG_NB/2)*8);
  cursor0 += 8;
}

void get_part1_msg(t_input_data info)
{
  static unsigned int cursor1 = (DISPLAY_MSG_NB/2) * 8;
  _fill_info(info, cursor1, TOTAL_SIZE);
  cursor1 += 8;
}
