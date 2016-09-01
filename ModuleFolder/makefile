FF=gfortran
SRCS=Standard_Collection.f90 atmos76.f90 Environment.f90 Rocket_Class.f90 Rocket_Dynamics.f90
COBJS=Standard_Collection.o atmos76.o Environment.o Rocket_Class.o Rocket_Dynamics.o


all:$(SRCS)
	$(FF) -c $(SRCS) -Wall

%.o:%.f90
	$(FF) -c $<
	
%.mod:%.f90 %.o
	$(FF) -c $<
	
clean:rm -r 

