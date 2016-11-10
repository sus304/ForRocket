FF=ifort
LINKER=link
RM=del

TARGET=ForRocket.exe
OUTDIR=./bin
OBJDIR=./obj
SRCDIR=./src

SRCS=$(SRCDIR)/Rocket_Class.f90 $(SRCDIR)/Standard_Collection.f90 $(SRCDIR)/atmos76.f90 $(SRCDIR)/Environment.f90 $(SRCDIR)/Rocket_Dynamics.f90 $(SRCDIR)/Simulation.f90
OBJS=$(OBJDIR)/Rocket_Class.obj $(OBJDIR)/Standard_Collection.obj $(OBJDIR)/atmos76.obj $(OBJDIR)/Environment.obj $(OBJDIR)/Rocket_Dynamics.obj $(OBJDIR)/Simulation.obj

FFLAGS=/nologo /c /Fo"$(OBJDIR)\\" /Fd"$(OBJDIR)\\" /O2 /module:"$(OBJDIR)"
LFLAGS=/NOLOGO /SUBSYSTEM:CONSOLE  

all:clean $(OBJDIR)\$(OBJS) $(OUTDIR)\$(TARGET)
	
clean:  
	-@if not exist $(OUTDIR) md $(OUTDIR)
	-@if not exist $(OBJDIR) md $(OBJDIR)

$(OUTDIR)\$(TARGET): $(OBJS)  
	$(LINKER) $(LFLAGS) $(OBJS) /OUT:"$(OUTDIR)\$(TARGET)" $**  
  
$(OBJDIR)\$(OBJS): $(SRCS)  
	$(FF) $(FFLAGS) $(SRCS)