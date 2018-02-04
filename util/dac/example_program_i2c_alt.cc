/*
A user-space C++ program to deal with a I2C device.
*/

extern "C" {
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
};
#include <cstdint>
#include <cstdlib>
#include <ctime>

namespace i2c {

////////////////////////////////////////////////
// I2C structures for content manipulation
////////////////////////////////////////////////

// -- Channel masks

#define CH0 0x0
#define CH1 0x1
#define CH2 0x2
#define CH3 0x3
#define CH4 0x4
#define CH5 0x5
#define CH6 0x6
#define CH7 0x7
#define ALL_CH 0xF


// -- Commands

/// Define a couple of commands
// not sure of what it does
#define I2C_WR_IN_N     0x0 
// Read one channel
#define I2C_RD_N        0x1
// Write all channels
#define I2C_WR_UPD_ALL  0X2
// Write channel N (mask)
#define I2C_WR_UPD_N    0x3
// Power on/off
#define I2C_POWER       0x4
// Clear 
#define I2C_CLEAR       0x5
// Load registers?
#define I2C_LOAD        0x6
// Reset
#define I2C_RESET       0x7

static const uint32_t dac_nbytes = 3;

typedef struct device {
    int32_t     fd;
    uint16_t    addr;
    uint16_t    channel;
    std::string devname;
} device;

typedef struct command_t {
    uint8_t channel : 4;
    uint8_t command : 4;
} command_t;

typedef union command {
    command_t   cmd;
    uint8_t         val;
} command;

typedef struct level_t {
    uint16_t lsdb : 8;
    uint16_t msdb : 8;
} level_t;

typedef union level {
    level_t  level;
    uint16_t     val;
} level;


const uint16_t ndacs = 3;
uint16_t DAC[ndacs] = {0x48,0x4A,0x4C};


// #include <errno.h>
// #include <string.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <linux/i2c-dev.h>
// #include <sys/ioctl.h>
// #include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>
 


int32_t i2c_init(const device &dac, bool debug = true) {
    int32_t file;
    if (debug) {
        printf("==================================================================\n");
        printf("i2c_init : Initializing I2C DAC %u:\n",dac.channel);
        printf(" * device : %s [fd %i] [DAC %u 0x%X]\n",dac.devname,
                                                        dac.fd,
                                                        dac.channel,dac.addr);
        printf("==================================================================\n");

    }
    if (file = open(dac.devname.c_str(),O_RDWR) < 0) {
        printf("i2c_init : Failed to open the bus.\n");
        return 0;
    }
    if (ioctl(file,I2C_SLAVE,dac.addr) < 0) {
        printf("i2c_init : Failed to acquire bus access and/or talk to the slave.\n");
        close(file);
        return 0;
    }
    dac.fd = file;
    return file+1;
}

void i2c_exit(const device &dac, bool debug = true) {
    int32_t file;
    if (debug) {
        printf("==================================================================\n");
        printf("i2c_exit : Terminating I2C DAC %u:\n",dac.channel);
        printf(" * device : %s [fd %i] [DAC %u 0x%X]\n",dac.devname,
                                                        dac.fd,
                                                        dac.channel,dac.addr);
        printf("==================================================================\n");

    }
    close(dac.fd);
}


int32_t i2c_write(const device& dac,uint8_t*buffer, const size_t& bytes, bool debug = true) {
    ssize_t wr_bytes;
    size_t idx = 0;
    if (debug) {
        printf("==================================================================\n");
        printf("i2c_write : Writing contents to I2C DAC %u:\n",dac.channel);
        printf(" * device : %s [fd %i] [DAC %u 0x%X]\n",dac.devname,
                                                        dac.fd,
                                                        dac.channel,dac.addr);
        printf(" * Full buffer :\n");
        for (idx = 0; idx < bytes; idx++) {
            printf(" Byte %u : 0x%X\n",buffer[idx]);
        }
        printf("==================================================================\n");
    }
    wr_bytes = write(dac.fd,buffer,bytes);
    if (wr_bytes != bytes) {
        printf("i2c_write : Failed to write the message to DAC %i. Received %i (expected %u).\n",dac.channel,wr_bytes,bytes);
        return 0;
    }
    return 1;
}

int32_t exec_dac_cmd(const device& dac, const command &cmd, bool debug = false) {
    uint8_t buffer[dac_nbytes];
    buffer[0] = cmd.val;

    if (debug) {
        printf("==================================================================\n");
        printf("exec_dac_cmd : Writing contents to I2C DAC %u:\n",dac.channel);
        printf(" * device : %s [fd %i] [DAC %u 0x%X]\n",dac.devname,
                                                        dac.fd,
                                                        dac.channel,dac.addr);
        printf(" * command : [cmd addr 0x%X] [ch 0x%X (%u)] [cmd 0x%X (%u)]\n",
               cmd.val, cmd.cmd.channel,cmd.cmd.channel,
               cmd.cmd.command,cmd.cmd.command);

        printf(" * Full buffer : [0x%X]\n",buff[0]);
        printf("==================================================================\n");

    }
    if (!i2c_write(dac,buffer,1)) {
        printf("exec_dac_cmd : Failed to program DAC %u (0x%X)\n",dac.channel,dac.addr);
    }
    return 1;
}

int32_t set_dac_level(const device& dac, const command &cmd, const level &lvl, bool debug = false) {
    uint8_t buffer[dac_nbytes];
    buffer[0] = cmd.val;
    buffer[1] = lvl.level.msdb;
    buffer[2] = lvl.level.lsdb;

    if (debug) {
        printf("==================================================================\n");
        printf("set_dac_level : Writing contents to I2C DAC %u:\n",dac.channel);
        printf(" * device : %s [fd %i] [DAC %u 0x%X]\n",dac.devname,
                                                        dac.fd,
                                                        dac.channel,dac.addr);
        printf(" * command : [cmd addr 0x%X] [ch 0x%X (%u)] [cmd 0x%X (%u)]\n",
               cmd.val, cmd.cmd.channel,cmd.cmd.channel,
               cmd.cmd.command,cmd.cmd.command);

        printf(" * Values : [lvl value 0x%X (%u)] [msdb 0x%X (%u)] [msdb 0x%X (%u)]\n",
               lvl.val,lvl.val,lvl.level.msdb,lvl.level.msdb,lvl.level.lsdb,lvl.level.lsdb);
        printf(" * Full buffer : [0x%X][0x%X][0x%X]\n",buff[0],buff[1],buff[2]);
        printf("==================================================================\n");

    }
    if (!i2c_write(dac,buffer,dac_nbytes)) {
        printf("set_dac_level : Failed to program DAC %u (0x%X)\n",dac.channel,dac.addr);
    }
    return 1;
}

int32_t get_dac_level(const device& dac, const command &c, level &lvl,bool debug = true) {
    ssize_t rd_bytes;
    uint8_t buffer[nbytes];

    // -- TO read back it is a bit more interesting, since we 
    // need to define the adress of where to look
    // -- So we have to first write the address
    // And then read the bytes

    buffer[0] = c.val;

    if (debug) {
        printf("get_dac_level : Writing register to read to I2C DAC %u:\n",dac.channel);
        printf(" * device : %s [fd %i] [DAC %u 0x%X]\n",dac.devname,
                                                        dac.fd,
                                                        dac.channel,dac.addr);
        printf(" * command : [cmd addr 0x%X] [ch 0x%X (%u)] [cmd 0x%X (%u)]\n",
               c.val, c.cmd.channel,c.cmd.channel,
               c.cmd.command,c.cmd.command);
    }
    if (!i2c_write(dac,buffer,1)) {
        printf("get_dac_level : Failed to set pointer to read DAC %i. Received %i (expected %u).\n",dac.channel,rd_bytes,1);
        return 0;
    } else {
        rd_bytes = read(dac.fd,buffer,2);
        if (rd_bytes != 2) {
            printf("get_dac_level : Failed to read DAC %i. Received %i (expected %u).\n",dac.channel,rd_bytes,2);
            return 0;
        } else {
            lvl.level.msdb = buffer[0];
            lvl.level.lsdb = buffer[1];
            if (debug) {
                printf("get_dac_level : Contents read: 0x%X (%u) : [0x%X,0x%X]\n",
                       lvl.val,lvl.val,lvl.level.msdb,lvl.level.lsdb);
            }
        }
    }
    return 1;
}

};


int main() {
    const size_t ndacs = 3;
    const size_t nchannels = 8; // 8 channels per DAC
    size_t i = 0, j = 0;
    i2c::device dev[3];


    printf("Initializing rando number generator...\n");
    std::srand((int)std::time(0));

    printf("Initializing...\n");
    for (i = 0; i < ndacs; i++) {
        dev[i].channel = i;
        dev[i].addr = DAC[i];
        dev[i].devname = "/dev/i2c-0";
        i2c_init(dev[i],true);
    }
    printf("Init done.\n");

    // -- First let's try a reset
    i2c::command com;
    i2c::level lvl;

    printf("Resetting the DACs...\n");
    com.cmd.command = I2C_RESET;
    com.cmd.channel = ALL_CH; // This actually does not matter. Only the command is relevant

    for (i = 0; i < ndacs; i++) {
        printf("...DAC %u\n",i);
        exec_dac_cmd(dev[i],com,true);
    }
    printf("Resetting done...\n");

    sleep(2);

    // -- Try to read the contents of each DAQ
    printf("Reading current channel levels...\n");

    com.cmd.command = I2C_RD_N;
    // loop over device and then over channel
    for (i=0; i < ndacs; i++) {
        printf("...DAC %u :\n",i);
        for (j = 0; j < nchannels; j++) {
            printf("...Read ch %u\n",j);
            com.cmd.channel = j;
            // -- Now go read
            get_dac_level(dev[i],com,lvl,true);
            printf("DAC level [%u][%u] : %u\n",i,j,lvl.val);
            sleep(1);
        }
    }
    printf("Done reading channel levels...\n");

    sleep(2);
    printf("Writing random channel levels...\n");
    // -- This is going to be a neat trick. Write random values and then make sense that we can 
    // read them back
    uint16_t level_table[ndacs][nchannels];

    com.cmd.command = I2C_WR_UPD_N;
    // loop over device and then over channel
    for (i=0; i < ndacs; i++) {
        printf("...DAC %u :\n",i);
        for (j = 0; j < nchannels; j++) {
            level_table[i][j] = (uint16_t) (rand() % 255) + 1;
            printf("...Write ch %u [%u]\n",j,level_table[i][j]);

            com.cmd.channel = j;
            lvl.val = level_table[i][j];
            // -- Now go write
            set_dac_level(dev[i],com,lvl,true);
        }
    }
    printf("Done writing channel levels...\n");

    sleep(3);

    // -- Finally read them back again anc check that they have what we are expecting
    // -- Try to read the contents of each DAQ
    printf("Reading current channel levels...\n");

    com.cmd.command = I2C_RD_N;
    // loop over device and then over channel
    for (i=0; i < ndacs; i++) {
        printf("...DAC %u :\n",i);
        for (j = 0; j < nchannels; j++) {
            printf("...Read ch %u\n",j);
            com.cmd.channel = j;
            // -- Now go read
            get_dac_level(dev[i],com,lvl,true);
            if (lvl.val == level_table[i][j]) {
                printf("DAC level [%u][%u] : %u (OK)\n",i,j,lvl.val);
            } else {
                printf("DAC level [%u][%u] : %u (FAIL)\n",i,j,lvl.val);
            }
        }
    }
    printf("Done reading channel levels...\n");

    printf("All tests complete...\n");
    for (i = 0; i < ndacs; i++) {
        i2c_exit(dev[i],true);
    }

    return 0;
}
