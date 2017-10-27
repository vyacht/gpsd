#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <netdb.h>

#include <errno.h>


int main(int argc, char * argv[]) {

    uint8_t buf[1024];
    uint8_t buf2[1024];
    int yes = 1;
    int n = 0;

    int port = 2000;
    char bcast[] = "127.0.0.1";
    char filename[255];

    int sock, status, sinlen;
    struct sockaddr_in sock_in;


    uint8_t opts = 0;
    char c = 0;

    while ((c = getopt(argc, argv, "f:")) != -1) {

        switch (c) {
        case 'f':
            if(strlen(optarg) > 3) {
                strcpy(filename, optarg);
            } 
            break;
        default:
            break;
        }
    }


    sinlen = sizeof(struct sockaddr_in);
    memset(&sock_in, 0, sinlen);

    sock = socket (PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    sock_in.sin_addr.s_addr = htonl(INADDR_ANY);
    sock_in.sin_port = htons(0);
    sock_in.sin_family = PF_INET;

    sock_in.sin_addr.s_addr=inet_addr(bcast); //htonl(-1); /* send message to 255.255.255.255 */
    sock_in.sin_port = htons(port); /* port number */

    sock_in.sin_family = PF_INET;

    status = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(int) );

    //FILE * f = fopen("/home/bo/prg/tmp/gpsd/log.bin", "r");
    //FILE * f = fopen("test/log-20160720-2.bin", "r");
    FILE * f = fopen(filename, "r");

    if(f == NULL) {
        printf("Couldn't open file \n");
        exit(1);
    } else 
        printf("opening file %s\n", filename);

    memset(buf, 0, 1024);
    memset(buf2, 0, 1024);
    size_t read = -1;
    size_t nlen = 411;
    int count = 0, i = 0;
    uint8_t t[] = {0x7e, 0x02, 0x13, 0x73, 0x74, 0x61, 0x74};

    while(((read= fread(buf, nlen, 1, f)) > 0) && (n < 20)) {
//    while(((read= fread(buf, nlen, 1, f)) > 0)) {

        printf("read %lu bytes\n", read);

        if(sendto(sock, buf, nlen, 0, (struct sockaddr *)&sock_in, sinlen) < 0) {
            printf("sendto Status = %s\n", strerror(errno));
        }

        for(i = 0; i < nlen; i++) {
            uint8_t ch = buf[i];
            if(((ch < 31) | (ch > 127)) & (ch != '\n')) {
            printf(".");
			buf2[i] = '.';
            } else {
                printf("%c", ch);
                buf2[i] = ch;
            }
        }

        printf("\n");
    	const char *tmp = buf2;
    	while(tmp = strstr(tmp, "VHW")){
            count++; tmp++;
    	}
        printf("VHW = %d\n", count);
        
    	tmp = buf2;
    	while(tmp = strstr(tmp, "statn2k")){
            //if(((tmp-2)[0] == 0x02))
            printf("STAT %02x\n", tmp[0]);
            tmp++;
    	}
        
        n++;
    	memset(buf, 0, 1024);
    	memset(buf2, 0, 1024);
    }

    fclose(f);
    printf("read %lu bytes\n", read);
    
}
