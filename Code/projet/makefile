
#This is a template to build your own project with the e-puck2_main-processor folder as a library.
#Simply adapt the lines below to be able to compile

# Define project name here
PROJECT = projet

DEF = -DSTM32F407xx

#Define path to the e-puck2_main-processor folder
GLOBAL_PATH = ../../lib/e-puck2_main-processor

#Source files to include
CSRC += ./usr_interface.c \
		./detect_obj.c \
		./led_anim.c \
		./capture_color.c \
		./coordinate_motor.c \
		./main.c \
				
#Header folders to include
INCDIR += 

#Jump to the main Makefile
include $(GLOBAL_PATH)/Makefile