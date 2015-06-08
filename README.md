# ptbreader
Penn Trigger Board Reader

This package implements a threaded linux application that is responsible for communicating with Penn Trigger Board.
The data transfer is performed through UIO interface for the DMA data transmission, and through the GPIO interface for the register setting. 

The communication with the DAQ backend is performed through 2 TCP socket connections: a control stream and a data stream. The control stream is responsible to receive the configuration and pass that to the manager that sets the registers. The data stream is a pure data transmission connection, directly from the DMA mapped memory.

TODO List:

* Finish implementation of the data stream (PTBReader)
* Implementation of a monitoring channel accessible from the board reader.

