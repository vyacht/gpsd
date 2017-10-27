#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

int main(int argc, char * argv[]) {

  char * buf = argv[1];
  char csum[3] = { '0', '0', '0' };
  unsigned int n = 1, crc;

  for (n = 1; buf[n] != '\0'; n++)
    crc ^= buf[n];
  (void)snprintf(csum, sizeof(csum), "%02X", crc);

  printf("%s*%s\n", buf, csum);

  return 0;

}
