#!/usr/bin/env python

import socket
import threading
import time
import signal

# The parameters of the connection to the PTB
#TCP_IP = '192.168.1.1'
TCP_IP = 'localhost'
TCP_PORT = 8991
TCP_PORT_READER = 8992
BUFFER_SIZE = 1024
run_client = True
run_server = True
keep_reading = True
t1 = None
t2 = None

def load_config():
    # The message is taken from the configuration file
    print "Loading configuration file."
    f = open('config.xml','r')
    config = f.read()
    f.close()
    return config

def dump_packet(data):
    print "Dumping"
    size = len(data)
    barray = bytearray(data)
    print type
    header = int(data >> size*8=3,2)
    
    print "Dumping [",data,"]"

def control_thread():
    # Now start the server
    s = None
    conf = load_config()
    while True:
        try: 
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            #s.settimeout(5.0)
            s.connect((TCP_IP, TCP_PORT))
        except socket.error, (value,message):
            if s:
                s.close()
                #print "Could not open socket: " + message   
            time.sleep(3)
            continue
        break
    print "Sending the configuration"
    size = len(conf)
    pos = 0
    while pos < size:
        s.send(conf[pos:pos+1023])
        pos += 1023
    time.sleep(5) # delays for 5 seconds to let the configuration kick in
    data = s.recv(BUFFER_SIZE)
    print "Received answer ", data

    # Now start the run
    print "Starting the run"
    s.send('<command>StartRun</command>');
    time.sleep(5)
    data = s.recv(BUFFER_SIZE)
    print "Received answer ", data
    
    time.sleep(5)
    print "Stopping the run"
    s.send('<command>StopRun</command>')
    print "Closing the connection"
    s.close()

def client_thread():
    print "Starting client thread"
    # Now start the server
    s = None
    while True:
        try: 
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print "Binding to ", TCP_IP
            s.bind((TCP_IP, TCP_PORT_READER))
            s.listen(1)
            #s.settimeout(5.0) # 5 second timeout
            print "Configured"
        except socket.error, (value,message):
            print "Inside the exception"
            if s:
                s.close()
            print "Could not client socket: " + message
            print "Going to retry in 3 s"
            time.sleep(3)
            continue
        break
    # Receive the data
    print "Waiting for connection"
    while run_server:
        try: 
            client,address = s.accept() # Accept a connection or keep trying
            print "Received connection"
            while keep_reading:
                data = client.recv(BUFFER_SIZE)
                if data:
                    dump_packet(data)
        except  socket.error, (value,message):
            continue
        print "Lost the connection"
    print "Leaving the server"
    s.close()

def Exit_gracefully():
    run_client = False
    keep_reading = False
    run_server = False



def main():
    print "Starting to run"
    # Start a thread for the control and another 
    t1 = threading.Thread(target=client_thread)
    ###t1.daemon = True
    t1.start()
    t2 = threading.Thread(target=control_thread)
    ###t2.daemon = True
    t2.start()
    t1.join()
    t2.join()
    print "All done. Leaving."

if __name__ == "__main__":
    signal.signal(signal.SIGINT, Exit_gracefully)
    main()
