This directory contains the udev rules required to run the LinuxCNC modules with the correct permissions.

On a Debian system, copy these rules to /etc/udev/rules.d using 'sudo cp *.rules /etc/udev/rules.d'. On other Linux flavours, consult system documentation 

You probably also need to reload the rules by executing 'sudo udevadm control --reload-rules' and 'sudo udevadm trigger'