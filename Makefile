
CC=arm-none-linux-gnueabi-gcc


TARGET = newcodes600

BUILD_DIR=build

SRC_DIR = sourcefile					 # src/cantool
SRC_DIR += bussc
SRC_DIR += caculateProceTime
#SRC_DIR += bussc/sourcefile

INC_DIR = include 
INC_DIR += bussc
INC_DIR += caculateProceTime
#INC_DIR += bussc/sourcefile

CFLAGS=$(patsubst %,-I %,$(INC_DIR))
CFLAGS += -g -Wall -O1 -msoft-float -I../include         # 不加这条编译成的文件只有400k+，程序不能执行。加了这条才有625837 

INCLUDES=$(foreach dir,$(INC_DIR),$(wildcard $(dir)/*.h))

SOURCE=$(foreach dir,$(SRC_DIR),$(wildcard $(dir)/*.c))

OBJS=$(patsubst %.c,$(BUILD_DIR)/%.o,$(notdir $(SOURCE)))

LIBS = -lc -lm -lnosys 

LIBS :=  -lpthread 
INCL = -I ../include-inter -I../include -I ../pub-link/link-status-lib 
LIBSDIR = -L../pub -L../pub-inter -L../libs

VPATH=$(SRC_DIR)

# 下面的 -lm 是链接数学math库，不加 -lm会报错：.text+0x2a818): undefined reference to `sqrt'
$(BUILD_DIR)/$(TARGET):$(OBJS)

	$(CC) $(^) -lm -o $(@) $(LIBS)   #$(CC) -lm -o $(TARGET) -g $(OBJS) $(LIBS)  

 	
$(BUILD_DIR)/%.o:%.c $(INCLUDE) | create_build

	$(CC) -c $< -o $@ $(CFLAGS)

.PHONY:clean create_build

clean:

	rm -r $(BUILD_DIR)

create_build:

	mkdir -p $(BUILD_DIR)
