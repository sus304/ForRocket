FF=gfortran
SRCS=Standard_Collection.f90 atmos76.f90 Environment.f90 Airframe.f90 Translation.f90 Rotation.f90
COBJS=Standard_Collection.o atmos76.o Environment.o Airframe.o Translation.o Rotation.o


all:$(SRCS)
	$(FF) -c $(SRCS) -Wall

%.o:%.f90
	$(FF) -c $<
	
%.mod:%.f90 %.o
	$(FF) -c $<
	
clean:rm -r 

