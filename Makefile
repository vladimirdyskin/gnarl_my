PROJECT_NAME := gnarl

CPPFLAGS += -I $(abspath include)

include $(IDF_PATH)/make/project.mk
