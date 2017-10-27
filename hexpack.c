/*
  mini prg to write a command to write and read the vynmea chips
 */
#include <stdio.h>      // standard input / output functions
#include <stdint.h>
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions

#define LEN 4096

int main(int argc, char * argv[]) {

    char n2kSequence[LEN];
    uint8_t bin[LEN];
  
    uint8_t opts = 0;

    opterr = 0;
    char c = 0;

    while ((c = getopt(argc, argv, "b:")) != -1) {

        switch (c) {
        case 'b': 
            strncpy(n2kSequence, optarg, LEN);
            printf("n2k %s\n", n2kSequence);
            opts++;
            break;
        default:
            abort ();
        }
    }
  
    int msgll = 
        gpsd_hexpack(n2kSequence, bin, LEN);
  
    int width=16; //8;
    unsigned char line[9] = "\0";
    int toscreen = 1;
    
    int pos = 0;
    unsigned char ch = ' ';

    for (pos=0; pos < msgll; pos++) {
        ch = bin[pos];
        line[pos%width] = ((ch > 31) && (ch < 128)) ? ch : '.';
        line[(pos%width)+1] = '\0';
        if (toscreen) printf("%02x ", ch);
        if (pos%width > (width-2)) {
            if (toscreen) printf("  %s\n", line);
        }
    }
    if (pos%width != 0) {
        int p;
        for (p=0; p<width-(pos%width); p++) printf("   ");	//padding
        if (toscreen) printf("  %s\n\n", line);
    }

    return 0;

}
