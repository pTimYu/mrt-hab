# Variables
CXX = g++
CXXFLAGS = -Wall -I.
SRCS = main.cpp \
       data-receive/serialib.cpp \
       data-receive/data_receive.cpp
TARGET = build/arduino_app

# The build rule
all:
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

clean:
	rm -f $(TARGET)

run:
	./$(TARGET)
