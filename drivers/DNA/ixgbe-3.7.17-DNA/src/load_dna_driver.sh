#!/bin/bash

# Configure here the network interfaces to activate
IF[0]=dna0
IF[1]=dna1
IF[2]=
IF[3]=

#service udev start

# Remove old modules (if loaded)
rmmod ixgbe
rmmod pf_ring

# We assume that you have compiled PF_RING
insmod ../../../../kernel/pf_ring.ko

# Default (as many queues as the number of processors)
insmod ./ixgbe.ko RSS=1,1,1,1

# Disable multiqueue
#insmod ./ixgbe.ko RSS=0,0,0,0

# Configure the number of TX slots
#insmod ./ixgbe.ko RSS=0,0,0,0 num_tx_slots=4096

# Enable 8 queues
#insmod ./ixgbe.ko MQ=1,1 RSS=8,8 FdirMode=0,0

# Enable hw filters
#insmod ./ixgbe.ko RSS=0,0,0,0 FdirMode=2,2,2,2 FdirPballoc=3,3,3,3

# Set a large MTU (jumbo frame)
#insmod ./ixgbe.ko RSS=0,0,0,0 mtu=9000

sleep 1

killall irqbalance 

for index in 0 1 2 3
do
  if [ -z ${IF[index]} ]; then
    continue
  fi
  printf "Configuring %s\n" "${IF[index]}"
  ifconfig ${IF[index]} up
  sleep 1
  bash ../scripts/set_irq_affinity.sh ${IF[index]}
#  ethtool -A ${IF[index]} autoneg off
#  ethtool -A ${IF[index]} rx off
#  ethtool -A ${IF[index]} tx off
#  ethtool -s ${IF[index]} speed 10000
done
