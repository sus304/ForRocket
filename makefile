## Compiler Command
####################################
CXX=g++

## Compiler Option
####################################
CXXFLAGS=-Wall -std=gnu++11
RELEASE_FLAGS=-O2

TEST_FLAGS=-O0 -g

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

TEST_INCLUDES=-I./lib/googletest/include -Lgoogletest -lgtest -lgtest_main -lpthread

## Source File
####################################
## コンパイル順序は依存関係を満たすように
## 上から順にコンパイル
SRCS =$(SRCDIR)/.cpp
SRCS +=$(SRCDIR)/main.cpp

OBJS=$(SRCS:.cpp=.o)  # srcsの.cppを.oに変換して登録


RELEASE_SRCS +=$(SRCDIR)/log_main.cpp
RELEASE_OBJS=$(RELEASE_SRCS:.cpp=.o) 

TEST_SRCS=$(TEST_SRCDIR)/test_FlowTest_LogIntegrate.cpp
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

.PHONY: test
test: CXXFLAGS+=$(TEST_FLAGS)
test: INCLUDES+=$(TEST_INCLUDES)
test: test_build
# test: clean

.PHONY: build
build: $(OBJS) $(RELEASE_OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGETDIR)/$(TARGET) $^

.PHONY: test_build
test_build: $(OBJS) $(TEST_OBJS) googletest/libgtest.a googletest/libgtest_main.a
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
