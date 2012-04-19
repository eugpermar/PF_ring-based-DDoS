E1000=e1000/e1000-8.0.35
E1000E=e1000e/e1000e-1.6.3
IGB=igb/igb-3.2.10
IXGBE=ixgbe/ixgbe-3.7.17

all:
	cd $(E1000)/src; make
	cd $(E1000E)/src; make
	cd $(IGB)/src; make
	cd $(IXGBE)/src; make

clean:
	cd $(E1000)/src; make clean
	cd $(E1000E)/src; make clean
	cd $(IGB)/src; make clean
	cd $(IXGBE)/src; make clean
