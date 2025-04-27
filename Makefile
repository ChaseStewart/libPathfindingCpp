##############################################
# Makefile for main executable to exercise libpathfinding
# Written by Chase E. Stewart
# 4/24/2025
##############################################

CC = g++
CFLAGS =  -g -std=c++20 -Wall -Werror 
CFLAGS += -Werror=unused-variable -Werror=unused-function -Werror=unused-but-set-variable 
CFLAGS += -Wl,-R./libpathfinding
INCLUDES = -I./libpathfinding
SRCS = main.cpp
OBJS = $(SRCS:.c=.o)
TARGET = main
LDFLAGS = -L./libpathfinding
LDLIBS = -lpathfinding

.PHONY: clean

main: $(OBJS)
	make -C ./libpathfinding
	$(CC) $(CFLAGS) $(INCLUDES) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $(TARGET) $(LDFLAGS) $(LDLIBS)

$(OBJS):
	$(CC) $(CFLAGS) $(INCLUDES) -cpp $< -o $@ $(LDFLAGS) $(LDLIBS)

clean:
	make clean -C ./libpathfinding
	$(RM) *.o $(TARGET)
