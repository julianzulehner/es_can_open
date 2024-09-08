TARGET_NAME = mainx64
# Compiler
CC = gcc

# Compiler flags
CFLAGS = -Wall -Wextra -std=c11

# Target executable
TARGET = $(TARGET_NAME).exe

# Source files
SRC = $(TARGET_NAME).c 

# Object files
OBJ = $(SRC:.c=.o)

# Default target
all: $(TARGET) run

# Build target
$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJ)

# Compile source files into object files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Run target
run:
	./$(TARGET)

# Clean target
clean:
	del $(TARGET) $(OBJ)


