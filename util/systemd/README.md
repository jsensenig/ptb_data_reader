# SystemD startup service for the PTB/CTB

This configuration starts the PTB/CTB software interface in a daemon.

## How to install

Copy the `ptb.service` into the systemd setup directory (eg. `/lib/systemd/system`).

## TODO List

* Introduce the listening port as a configuration parameter
* Implement internal voltages and temperature monitoring using the XADC block and add an option to activate the feature, serving the results in a port passed as a parameter.
