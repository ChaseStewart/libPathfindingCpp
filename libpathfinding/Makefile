##############################################
# Makefile for libpathfinding shared library
# Written by Chase E. Stewart
# 4/24/2025
##############################################

CC = g++
CPPFLAGS = -g -std=c++20 -Wall -shared -fPIC 
INCLUDES = -I.
SRCS = pathfinding.cpp

TARGET = libpathfinding.so

.PHONY: clean

all:    $(TARGET)
	@echo  libpathfinding compiled

$(TARGET):
	$(CC) $(CPPFLAGS) $(INCLUDES) -o $(TARGET) $(SRCS) 

clean:
	$(RM) -f *.o $(TARGET)
