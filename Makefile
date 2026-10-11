# Makefile for High-Performance BBAI-64 Web Teleoperation C++ Server
CXX = g++
CXXFLAGS = -O3 -std=c++17 -Wall -Wextra -pthread
INCLUDES = -I./include $(shell pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv 2>/dev/null)
LIBS = -pthread $(shell pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv 2>/dev/null)

TARGET = web_teleop_server
SRCS = src/web_teleop_server.cpp

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(TARGET) $(SRCS) $(LIBS)
	@echo "=========================================================="
	@echo " Build Complete: ./$(TARGET)"
	@echo " Run with: ./$(TARGET) [port]"
	@echo "=========================================================="

clean:
	rm -f $(TARGET)

.PHONY: all clean
