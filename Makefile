#--------------------- Basic Settings -----------------------------------------#
PROGRAM_NAME  := des
BINARY_BASE   := bin
BUILD_BASE    := bld
INCLUDE_BASE  := inc
SOURCE_BASE   := src

#--------------------- External Libraries -------------------------------------#
HEADER_DIRS   := ../libprim/inc
STATIC_LIBS   := ../libprim/bld/libprim.a

#--------------------- Cpp Lint -----------------------------------------------#
LINT          := $(HOME)/.makeccpp/cpplint/cpplint.py
LINT_FLAGS    :=

#--------------------- Unit Tests ---------------------------------------------#
TEST_SUFFIX   := _TEST
GTEST_BASE    := $(HOME)/.makeccpp/gtest

#--------------------- Compilation and Linking --------------------------------#
CXX           := g++
AR            := gcc-ar
SRC_EXTS      := .cc
HDR_EXTS      := .h .tcc
CXX_FLAGS     := -std=c++11 -Wall -Wextra -pedantic -Wfatal-errors 
CXX_FLAGS     += -march=native -g -O3 -flto
CXX_FLAGS     += -pthread
#CXX_FLAGS     += -DNDEBUGLOG
LINK_FLAGS    := -lz -lpthread -Wl,--no-as-needed

#--------------------- Auto Makefile ------------------------------------------#
include ~/.makeccpp/auto_lib.mk
