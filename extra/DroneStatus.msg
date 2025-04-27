#########################################
# @file DroneStatus.msg 
# @author Chase E. Stewart
#
# @brief Message for conveying quadcopter status for task allocation in distributed system
# tested by creating a tutorial ROS2 package and compiling
#
# Guiding questions: 
#   1. what would a 'dispatcher' want to know
#   2. what comes free with RoS2 
#   3. what are we optimizing for (task emphasizes optimization and network bandwidth)
#
#########################################

# Assumptions made in this development:
#    1. This is a "low frequency" message, human timescale
#    2. Proximity is critical for task selection 
#          (i.e. tasks are short duration, majority of time is in transit)
#    3. There is no benefit to "load balancing", unequal distribution of tasks is fine
#          (if we felt otherwise, could offer utilization in message)
#    4. There is a possiblity of wanting to field platforms with different configurations and/or versions
#          (and thus the api_version)

# header with high level info 
uint8 node_id  # which quadcopter out of 4 is reporting, useful for status_bitfield
uint8 api_version  # what version of this message API is in use, omit if user will never field mixed versions/ HW

# RE: below, I read a trick from source https://www.aapelivuorinen.com/blog/2023/06/30/floats-for-coordinates/
# that states storing lat/lon as (-180+360*(u/2^32)) gives millimeter precision in 32 bits
# at the cost of translation. If you absolutely hate this idea, pretend I made this message float64,
# as float32 has 1.7M granularity; I am under the assumption then that float32 is insufficient

# position
int32 latitude_wgs84  # latitude, requires translation -180+360*(u/2^32) 
int32 longitude_wgs84  # longitude, requires translation -180+360*(u/2^32)
int32 elevation_wgs84  # elevation, requires translation -180+360*(u/2^32)

# status_bitfield bits
uint64 status_bitfield  # see block comment below

# We need to ensure a consistent interface with source code
# However, declaring a constant would add waste to the message
# As ROS2 Interfaces states constants are like fields with defaults 
# that are read-only; so then, I would use something like python's 
# beautifulSoup library, or a perl script to generate both this comment
# and a C++ header in the source code, preferably from a wiki or excel ICD
#
# You could use a union with extra bits in status bitfield to create more types as needed
# in an extremely tight space, just be careful and do lots of validation
# 
# Regardless I am writing it in the constant format here in this block comment
# uint64 STATUS_IDLE=0x00
# uint64 STATUS_WORKING=0x01
# uint64 STATUS_HW_FAULT=0x02
# uint64 STATUS_SW_FAULT=0x04
# uint64 STATUS_LOW_BAT=0x08
# uint64 STATUS_RECHARGING=0x10
# uint64 STATUS_LOST=0x20
# uint64 STATUS_RESERVED_START=0x40 # NOTE having several reserved bits allows us to make minor edits to API that only enhance
# uint64 STATUS_RESERVED_END=0x80

# footer 
uint8 crc  # Calculated CRC integrity check in case link is sketchy
# if desired/necessary, ensure data integrity by performing CRC. 
# cut this field if ROS2 handles it; It seemed to me it does not for messages (vs. actions possibly with QoS)
