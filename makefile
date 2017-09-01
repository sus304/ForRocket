FC=gfortran

TARGET=ForRocket_v4.exe
TARGETPOST=ForRocketPost.exe
FFLAGS=-Wall -O2
TARGETDIR=./bin
OBJDIR=./obj
SRCDIR=./src

SRCS=$(SRCDIR)/Standard_Collection.f90 $(SRCDIR)/atmos76.f90 $(SRCDIR)/Environment.f90 $(SRCDIR)/Rocket_Dynamics.f90 $(SRCDIR)/Simulation.f90
OBJS=$(OBJDIR)/Standard_Collection.o $(OBJDIR)/atmos76.o $(OBJDIR)/Environment.o $(OBJDIR)/Rocket_Dynamics.o $(OBJDIR)/Simulation.o
MODS=Standard_Collection.mod atmos76.mod Environment.mod Rocket_Dynamics.mod Simulation.mod
SRCSPOST=$(SRCDIR)/Post.f90
OBJSPOST=$(OBJDIR)/Post.o

.SUFFIXES: .f90

$(TARGETDIR)/$(TARGET): $(OBJS)
	$(FC) -o $@ $^ $(FFLAGS)
	rm -f $(OBJS) ./*.mod
	$(FC) -o $(TARGETDIR)/$(TARGETPOST) $(SRCSPOST) $(FFLAGS)

$(OBJDIR)/%.o $(OBJDIR)/%.mod: $(SRCDIR)/%.f90
	@[ -d $(OBJDIR) ]
	$(FC) -o $@ -c $<

all: $(TARGET)

clean:
	rm -f $(OBJS) ./*.mod