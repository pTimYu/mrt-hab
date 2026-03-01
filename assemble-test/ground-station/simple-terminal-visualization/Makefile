# Variables
CXX = g++
CXXFLAGS = -Wall -I.
SRCS = main.cpp data-receive-and-decomposition/serialib.cpp data-receive-and-decomposition/data_receive_and_decompose.cpp
TARGET = build/arduino_app

# The build rule
all:
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

# Clean rule to remove the executable
clean:
	rm -f $(TARGET)

run:
	./$(TARGET)