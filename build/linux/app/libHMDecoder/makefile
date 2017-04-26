# the SOURCE definiton lets you move your makefile to another position
CONFIG 				= LIBRARY

# set directories to your wanted values
SRC_DIR				= ../../../../source/App/libHMDecoder
INC_DIR				= ../../../../source/Lib
LIB_DIR				= ../../../../lib
BIN_DIR				= ../../../../bin

SRC_DIR1		=
SRC_DIR2		=
SRC_DIR3		=
SRC_DIR4		=

USER_INC_DIRS	= -I$(SRC_DIR) 
USER_LIB_DIRS	=

ifeq ($(HIGHBITDEPTH), 1)
HBD=HighBitDepth
else
HBD=
endif

# intermediate directory for object files
OBJ_DIR				= ./objects$(HBD)

# set executable name
PRJ_NAME			= libHMDecoder$(HBD)

# defines to set
DEFS				= -DMSYS_LINUX -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -DMSYS_UNIX_LARGEFILE

# set objects
OBJS          		= 	\
					$(OBJ_DIR)/libHMDecoder.o \

# set libs to link with
LIBS				= -ldl

DEBUG_LIBS			=
RELEASE_LIBS		=

STAT_LIBS			= -lpthread
DYN_LIBS			=


DYN_DEBUG_LIBS		= -lTLibDecoder$(HBD)d -lTLibCommon$(HBD)d
DYN_DEBUG_PREREQS	= $(LIB_DIR)/libTLibDecoder$(HBD)d.a $(LIB_DIR)/libTLibCommon$(HBD)d.a
STAT_DEBUG_LIBS		= -lTLibDecoder$(HBD)Staticd -lTLibCommon$(HBD)Staticd
STAT_DEBUG_PREREQS	= $(LIB_DIR)/libTLibDecoder$(HBD)Staticd.a $(LIB_DIR)/libTLibCommon$(HBD)Staticd.a

DYN_RELEASE_LIBS	= -lTLibDecoder$(HBD) -lTLibCommon$(HBD)
DYN_RELEASE_PREREQS	= $(LIB_DIR)/libTLibDecoder$(HBD).a $(LIB_DIR)/libTLibCommon$(HBD).a
STAT_RELEASE_LIBS	= -lTLibDecoder$(HBD)Static -lTLibCommon$(HBD)Static
STAT_RELEASE_PREREQS = $(LIB_DIR)/libTLibDecoder$(HBD)Static.a $(LIB_DIR)/libTLibCommon$(HBD)Static.a 

# name of the base makefile
MAKE_FILE_NAME		= ../../common/makefile.base

# include the base makefile
include $(MAKE_FILE_NAME)
