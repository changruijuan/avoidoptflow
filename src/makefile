CC = gcc
XX = g++
CFLAGS = -Wall -O -g
TARGET = avoidOptflow
OPENCV_FLAG = `pkg-config --cflags opencv`
OPENCV_LIB = `pkg-config --libs opencv`

%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@ $(OPENCV_FLAG)
%.o : %.cpp
	$(XX) $(CFLAGS) -c $< -o $@ $(OPENCV_FLAG)

SOURCES = $(wildcard *.c *.cpp)
OBJS = $(patsubst %.c, %.o, $(patsubst %.cpp, %.o, $(SOURCES)))

$(TARGET) : $(OBJS)
	$(XX) $(OBJS) -o $(TARGET) $(OPENCV_LIB)
	chmod a+x $(TARGET)

clean:
	rm -rf *.o avoidOptflow
