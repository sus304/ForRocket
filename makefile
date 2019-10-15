CXX = g++
CXXFLAGS = -Wall -O2 -std=gnu++11

TARGET = ForRocket
TARGETDIR = ./bin
OBJDIR = ./obj
SRCDIR = ./src
INCLUDES = -I ./lib

ifeq ($(OS),Windows_NT)
    TARGET = ForRocket.exe
endif

SRCS = $(SRCDIR)/main.cpp
SRCS += $(SRCDIR)/DynamicsSolver.cpp
SRCS += $(SRCDIR)/Environment.cpp
SRCS += $(SRCDIR)/FlightController.cpp
SRCS += $(SRCDIR)/FlightDataRecorder.cpp
SRCS += $(SRCDIR)/LibDataIO.cpp
SRCS += $(SRCDIR)/Payload.cpp
SRCS += $(SRCDIR)/Rocket.cpp
SRCS += $(SRCDIR)/SOEHandler.cpp
SRCS += $(SRCDIR)/Stage.cpp

OBJS := $(SRCS:.cpp=.o)

.PHONY: all clean
.SUFFIXES: .c .cpp .o

all: $(TARGETDIR)/$(TARGET)

clean:
	$(RM) $(OBJS)

$(TARGETDIR)/$(TARGET): $(OBJS)
	$(CXX) -o $@ $(OBJS)
	$(RM) $(OBJS)

.cpp.o:
	$(CXX) $(INCLUDES) $(CXXFLAGS) -o $@ -c $^
.cpp:
	$(CXX) $(INCLUDES) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
