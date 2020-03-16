## Compiler Command
####################################
CXX=g++

## Compiler Option
####################################
CXXFLAGS=-Wall -std=gnu++11
RELEASE_FLAGS=-O2

TEST_FLAGS=-O0 -g -DDEBUG

## Target
####################################
TARGETDIR=./bin
TARGET=ForRocket
ifeq ($(OS),Windows_NT)
    TARGET = ForRocket.exe
endif

## Source File Directory
####################################
SRCDIR=./src

TEST_SRCDIR=./test

## Include Directory
####################################
INCLUDES=-I./lib
INCLUDES+=-I./src

TEST_INCLUDES=-I./test -I./lib/googletest/include -Lgoogletest -lgtest -lgtest_main -lpthread

## Source File
####################################
## コンパイル順序は依存関係を満たすように
## 上から順にコンパイル
SRCS +=$(SRCDIR)/degrad.cpp
SRCS +=$(SRCDIR)/commandline_option.cpp
SRCS +=$(SRCDIR)/fileio.cpp
SRCS +=$(SRCDIR)/json_control.cpp
SRCS +=$(SRCDIR)/interpolate.cpp
SRCS +=$(SRCDIR)/environment/coordinate.cpp
SRCS +=$(SRCDIR)/environment/datetime.cpp
SRCS +=$(SRCDIR)/environment/sequence_clock.cpp
SRCS +=$(SRCDIR)/environment/satmo1976.cpp
SRCS +=$(SRCDIR)/environment/air.cpp
SRCS +=$(SRCDIR)/environment/gravity.cpp
SRCS +=$(SRCDIR)/environment/wgs84.cpp
SRCS +=$(SRCDIR)/environment/wind.cpp
SRCS +=$(SRCDIR)/environment/vincenty.cpp

SRCS +=$(SRCDIR)/rocket/parameter/acceleration.cpp
SRCS +=$(SRCDIR)/rocket/parameter/attitude.cpp
SRCS +=$(SRCDIR)/rocket/parameter/force.cpp
SRCS +=$(SRCDIR)/rocket/parameter/interpolate_parameter.cpp
SRCS +=$(SRCDIR)/rocket/parameter/mass.cpp
SRCS +=$(SRCDIR)/rocket/parameter/moment.cpp
SRCS +=$(SRCDIR)/rocket/parameter/position.cpp
SRCS +=$(SRCDIR)/rocket/parameter/velocity.cpp
SRCS +=$(SRCDIR)/rocket/engine.cpp
SRCS +=$(SRCDIR)/rocket/flight_data_recorder.cpp
SRCS +=$(SRCDIR)/rocket/rocket.cpp

SRCS +=$(SRCDIR)/dynamics/dynamics_base.cpp
SRCS +=$(SRCDIR)/dynamics/dynamics_3dof_onlauncher.cpp
SRCS +=$(SRCDIR)/dynamics/dynamics_3dof_parachute.cpp
SRCS +=$(SRCDIR)/dynamics/dynamics_6dof_aero.cpp
SRCS +=$(SRCDIR)/dynamics/dynamics_6dof_programrate.cpp
SRCS +=$(SRCDIR)/dynamics/noniterative_iip.cpp

SRCS +=$(SRCDIR)/factory/engine_factory.cpp
SRCS +=$(SRCDIR)/factory/rocket_factory.cpp
SRCS +=$(SRCDIR)/factory/rocket_stage_factory.cpp

SRCS +=$(SRCDIR)/solver/rocket_stage.cpp
SRCS +=$(SRCDIR)/solver/trajectory_solver.cpp

OBJS=$(SRCS:.cpp=.o)  # srcsの.cppを.oに変換して登録


RELEASE_SRCS +=$(SRCDIR)/ForRocket.cpp
RELEASE_OBJS=$(RELEASE_SRCS:.cpp=.o) 

TEST_SRCS=$(TEST_SRCDIR)/common4test.cpp
TEST_SRCS+=$(TEST_SRCDIR)/test_interpolate.cpp
TEST_SRCS+=$(TEST_SRCDIR)/test_interpolateparameter.cpp
TEST_SRCS+=$(TEST_SRCDIR)/test_clock.cpp
TEST_OBJS=$(TEST_SRCS:.cpp=.o) 


## Suffixes
####################################
.SUFFIXES: .c .cpp .o


## Make Rule
####################################
all:debug

.PHONY: release
release: CXXFLAGS+=$(RELEASE_FLAGS)
release: build
release: clean

.PHONY: debug
debug: CXXFLAGS+=$(TEST_FLAGS)
debug: build
# debug: clean

.PHONY: build
build: $(OBJS) $(RELEASE_OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGETDIR)/$(TARGET) $^

.PHONY: test
test: CXXFLAGS+=$(TEST_FLAGS)
test: INCLUDES+=$(TEST_INCLUDES)
test: $(OBJS) $(TEST_OBJS) ./lib/googletest/libgtest.a ./lib/googletest/libgtest_main.a
	$(CXX) $(CXXFLAGS) -o $(TARGETDIR)/$(TARGET) $^
	
	
.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCLUDES) $< -o $@


clean:
	rm -f $(OBJS) $(TEST_OBJS) $(RELEASE_OBJS)

disclean:
	rm -f $(OBJS) $(TEST_OBJS) $(RELEASE_OBJS)
	rm -f $(TARGETDIR)/$(TARGET)


### Memo #############
# -o	直後の名前でコンパイルorリンク
# -c	コンパイルのみ行う
# -g	デバッグ情報を付加

# $@	ターゲットファイル名
# $%	ターゲットがアーカイブメンバだったときのターゲットメンバ名
# $<	最初の依存するファイルの名前
# $?	ターゲットより新しいすべての依存するファイル名
# $^	すべての依存するファイルの名前
# $+	Makefileと同じ順番の依存するファイルの名前
# $*	サフィックスを除いたターゲットの名前
