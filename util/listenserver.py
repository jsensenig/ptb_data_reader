import socket
import sys

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bind the socket to the port
server_address = ('localhost', 8992)
print >>sys.stderr, 'starting up on %s port %s' % server_address
print "Your configuration should be : "
print "<config><DataBuffer><DaqHost>localhost</DaqHost><DaqPort>8992</DaqPort></DataBuffer><RolloverClocks>25000000</RolloverClocks></config>"
print "And the running command should be :"
print "<command>StartRun</command>"
print "And eventually the stopping command:"
print "<command>StopRun</command>"
sock.bind(server_address)
# Listen for incoming connections
sock.listen(1)

while True:
    # Wait for a connection
    print >>sys.stderr, 'waiting for a connection'
    connection, client_address = sock.accept()
    try:
        print >>sys.stderr, 'connection from', client_address

        # Receive the data in small chunks and retransmit it
        while True:
            data = connection.recv(4096)
            if data:
                #print >>sys.stderr, 'received "%s"' % data
                print >>sys.stderr, " ".join("{:02x}".format(ord(c)) for c in data)
                
            #if data:
            #    print >>sys.stderr, 'sending data back to the client'
            #    connection.sendall(data)
            #else:
            #    print >>sys.stderr, 'no more data from', client_address
            #    break
            
    finally:
        # Clean up the connection
        connection.close()

