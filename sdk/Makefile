CXX = g++
CXXFLAGS = -std=c++11 -Wall -I./include -fPIC
AR = ar
ARFLAGS = rcs

SDK_SOURCES = $(wildcard src/*.cpp)
SDK_OBJECTS = $(SDK_SOURCES:.cpp=.o)
SDK_LIB = libsl_lidar_sdk.a

all: $(SDK_LIB)

$(SDK_LIB): $(SDK_OBJECTS)
	$(AR) $(ARFLAGS) $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(SDK_OBJECTS) $(SDK_LIB) 