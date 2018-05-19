# Standalone CTB test

This example implements a simple communication interface with the CTB. 
The application sends a (minimalistic) configuration and then creates a socket to listen for the sent data.

## How to run

The file `standalone_run.cpp` contains a couple of references for the IP and port of the CTB, and the IP and port where the data should be sent. Usually the latter should be the IP of the machine where the test is being executed and the some non-privileged port of your choice. 

After adjusting those parameters to your preferences, just need to type `make` to build and then `stadalone_run` to run.

The application implements an oversimplistic menu with 4 options:

```bash
### Select command :
 1 : Init
 2 : Start Run
 3 : Stop Run
 4 : Quit
```

Since the example actually runs over 2 threads, one can at any moment just type the number and `<enter>` to execute a stage. The stages are meant to follow the following pattern: 1->2->3->2->3 ... ->4

## Dependencies

The example is self contained and depends only on a C++11 capable compiler. 
The header `content.h` is copied from the `ptbreader` source and is meant only as a means of simplifying the parsing of the received data. However, one does not strictly have to use it. 

## Known issues

Due to issues with the TCP socket library being used, the example terminates abnormally. This socket library is flawed by the use of static objects, which causes the receiver thread (in this example) to never becoming joinable.