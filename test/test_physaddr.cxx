#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>
int main(int argc, char *argv[]) {
    if (argc < 3) {
        printf("Usage: %s <phys_addr> <offset>\n", argv[0]);
        return 0;
    }
    for (int i = 0; i < argc; ++i) {
      printf("Arg %d : raw : [%s], atoi :[%d]\n",i,argv[i],atoi(argv[i]));
    }
    off_t offset = atoi(argv[1]);
    size_t len = atoi(argv[2]);
    
    printf("Address 0x%lx (%ld) Offset 0x%lx (%lu)\n",offset,offset, len, len);


    int fd = open("/dev/mem", O_SYNC);
    if (fd == -1) {
      fprintf(stderr,"Failed to open /dev/mem. Error code %d : %s\n",errno,strerror(errno));
      printf("Failed to open /dev/mem. Error code %d : %s\n",errno,strerror(errno));

      return 1;
    }
    printf("Passed.\n");

    unsigned char *mem = (unsigned char*)mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_PRIVATE, fd, offset);
    if (mem == NULL) {
        printf("Can't map memory\n");
        return -1;
    }

    int i;
    for (i = 0; i < len; ++i)
        printf("%02x ", (int)mem[i]);

    printf("All printed.\n");
}

