# hl7\_bridges
This package creates a configurable bridge between ROS 2 and HL7 v2.x

There are two scripts: `hl7_mllp_client` and `hl7_mllp_server`, so that
data can bridge from HL7 -> ROS 2 or from ROS 2 -> HL7. The intent is
that multiple instances of each of those scripts will be launched, with
parameters specifying the respective MLLP ports, hostnames, and ROS 2
topic names of the resulting bridged connections.

# License
This software is released under the Apache License, version 2.0.
