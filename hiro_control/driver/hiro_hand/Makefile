all:stubs

IDL_BASE = vnet3Finger

TOP=../
include $(TOP)Make.rules

OB_IDL_PYTHON_FLAGS     = -bpython

stubs:../corba/$(IDL_BASE).idl
	$(OB_IDL_C) $(OB_IDL_PYTHON_FLAGS) $<

clean:
	-$(RM) -rf *~ *.pyc $(IDL_BASE)_idl.py VnetHand VnetHand__POA
