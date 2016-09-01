make=make --no-print-directory

FF=gfortran
FFLAGS=-Wall
PROGRAM=Simulation.exe

SRCS=Simulation.f90
OBJS=Simulation.o 
S_DIR=./ModuleFolder
COBJS=$(S_DIR)/Standard_Collection.o $(S_DIR)/atmos76.o $(S_DIR)/Environment.o $(S_DIR)/Rocket_Class.o $(S_DIR)/Rocket_Dynamics.o

default:sub prime

sub:
	$(make) -C ModuleFolder

prime:$(OBJS) $(COBJS)
	$(FF) $(FFLAGS) -o $(PROGRAM) -I$(S_DIR) $(OBJS) $(COBJS)
	rm -r $(OBJS)

$(OBJS):$(SRCS)
	$(FF) -c -I$(S_DIR) $<