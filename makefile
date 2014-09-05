TARGETS=test

CC=g++
#OPTIMISE=-O2
OPTIMISE=-O0
DEBUG=-ggdb3
C11FLAGS= -fno-strict-aliasing -DUNIV_LINUX \
	-std=c++0x -std=gnu++0x \
	-Werror -Wall -Wextra $(OPTIMISE) $(DEBUG)
LIBS=

RAW_COMMONOBJECTS=util.o ConfigReader.o
RAW_TARGETOBJECTS=$(TARGETS:%=%.o)

COMMONOBJECTS=$(RAW_COMMONOBJECTS:%=build/%)
TARGETOBJECTS=$(RAW_TARGETOBJECTS:%=build/%)

ALLOBJECTS=$(COMMONOBJECTS) $(TARGETOBJECTS)
OBJSRC=$(COMMONOBJECTS:.o=.cpp)

#recurse:
	#@make -j$(shell getconf _NPROCESSORS_ONLN) all

all: $(TARGETS)

build:
	mkdir -p build

-include $(ALLOBJECTS:.o=.d)

clean:
	rm -f $(TARGETS)
	rm -f build/*

build/%.o: %.cpp | build
	$(CC) $(C11FLAGS) -c $*.cpp -o build/$*.o
	$(CC) $(C11FLAGS) -MM $*.cpp > build/$*.d
	sed -i '1s/^/build\//' build/$*.d

test: build/test.o $(COMMONOBJECTS)
	$(CC) build/test.o $(COMMONOBJECTS) -o test $(LIBS)

run: $(TARGETS)
	./test
