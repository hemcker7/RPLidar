CXX = g++
CXXFLAGS = -std=c++11 -Wall -I./sdk/include
LDFLAGS = -L./sdk -lsl_lidar_sdk -lpthread -lrt
GLFW_LIBS = -lglfw -lGL -lGLEW

all: sdk data_logger visual_logger

sdk:
	cd sdk && $(MAKE)

data_logger: sdk
	$(CXX) $(CXXFLAGS) app/data_logger/main.cpp -o app/data_logger/data_logger $(LDFLAGS)

visual_logger: sdk
	$(CXX) $(CXXFLAGS) app/visual_logger/main.cpp -o app/visual_logger/visual_logger $(LDFLAGS) $(GLFW_LIBS)

clean:
	cd sdk && $(MAKE) clean
	rm -f app/data_logger/data_logger app/visual_logger/visual_logger 