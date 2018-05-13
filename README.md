# ptbreader
Penn Trigger Board Reader

This package implements a threaded linux application that is responsible for communicating with Penn Trigger Board.
The data transfer is performed through UIO interface for the DMA data transmission, and through the memory mapped registers for the register setting. 

The communication with the DAQ backend is performed through 2 TCP socket connections: a control stream and a data stream. The control stream is responsible to receive the configuration and pass that to the manager that sets the registers. The data stream is a pure data transmission connection, directly from the DMA mapped memory.

Several updates were added over the time and the system is now being adjusted to the protoDUNE CTB mkii. 
There are still a couple of edges to clean, but overall the software has been shown to work as intended.

# TODO list

* Adjust configuration as it becomes more clear on the CTB
* Implement extra triggers for ProtoDUNE
* Do a throughput test to evaluate the input rates the firmware is able to handle (as of May 9 2018 rates > 1MHz were causing no troubles)