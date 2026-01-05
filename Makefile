
CC := gcc
CFLAGS := -Wall -Wextra -I.

SRCS := $(wildcard src/*.c)
OBJS := $(SRCS:.c=.o
)
TARGET := netdr

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
