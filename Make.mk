#
# Copyright (c) 2018, Theodore A. Roth <troth@openavr.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of OpenAVR nor the names of its contributors may be used
#    to endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# Get the PATH for directory containing 'Make.mk' allowing the user to name this
# directory anything they want (e.g. 'Build', 'BuildSystem', 'Make', etc) and it
# avoids having hard coded pathes relative to TOPDIR in this makefile fragment.
MAKE_TOOLS_DIR :=  $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

# TOPDIR variable is set to the top directory of the application. There
# may be multiple applications in a project which share a common MAKE_TOOLS_DIR
# directory, but each will have a different TOPDIR.
TOPDIR := $(abspath $(CURDIR))
export TOPDIR

# For debugging
ifeq ($(DEBUG_MK),y)
  $(info "MAKE_TOOLS_DIR: $(MAKE_TOOLS_DIR))
  $(info "TOPDIR: $(TOPDIR)")
endif

include $(MAKE_TOOLS_DIR)/arches/$(TGT_ARCH).mk

# TODO: Move to $(TGT_ARCH).mk
TOOLCHAIN   ?= arm-none-eabi-

ADDR2LINE   := $(TOOLCHAIN)addr2line
AR          := $(TOOLCHAIN)ar
AS          := $(TOOLCHAIN)as
CPP         := $(TOOLCHAIN)cpp
CXX         := $(TOOLCHAIN)g++
CC          := $(TOOLCHAIN)gcc
LD          := $(TOOLCHAIN)ld
NM          := $(TOOLCHAIN)nm
OBJCOPY     := $(TOOLCHAIN)objcopy
OBJDUMP     := $(TOOLCHAIN)objdump
RANLIB      := $(TOOLCHAIN)ranlib
READELF     := $(TOOLCHAIN)readelf
STRIP       := $(TOOLCHAIN)strip
SIZE        := $(TOOLCHAIN)size

ifeq ($(SHOW_TOOLCHAIN),y)
  $(info "ADDR2LINE: $(ADDR2LINE)")
  $(info "AR:        $(AR)")
  $(info "AS:        $(AS)")
  $(info "CPP:       $(CPP)")
  $(info "CXX:       $(CXX)")
  $(info "CC:        $(CC)")
  $(info "LD:        $(LD)")
  $(info "NM:        $(NM)")
  $(info "OBJCOPY:   $(OBJCOPY)")
  $(info "OBJDUMP:   $(OBJDUMP)")
  $(info "RANLIB:    $(RANLIB)")
  $(info "READELF:   $(READELF)")
  $(info "STRIP:     $(STRIP)")
  $(info "SIZE:      $(SIZE)")
endif

## Warnings for C, C++ and ASM
WARNINGS  = -Wall
WARNINGS += -Werror
WARNINGS += -Wextra
#WARNINGS += -Wredundant-decls
WARNINGS += -Wundef
WARNINGS += -Wshadow
WARNINGS += -Wfloat-conversion
WARNINGS += $(ARCH_WARNINGS)

## Warnings for C
C_WARNINGS  = $(WARNINGS)
C_WARNINGS += -Wimplicit-function-declaration
C_WARNINGS += -Wmissing-prototypes
C_WARNINGS += -Wstrict-prototypes

## Warnings for C
CXX_WARNINGS  = $(WARNINGS)

## Warnings for C
AS_WARNINGS  = $(WARNINGS)

## Defines for C, C++ and ASM
DEFS  = $(ARCH_DEFS)
DEFS += $(TGT_DEFS)

## Defines for ASM
AS_DEFS  = $(DEFS)
AS_DEFS += $(ARCH_AS_DEFS)

## Flags for C, C++ and ASM
C_CXX_AS_FLAGS  = -Os  # Optimize for size.
C_CXX_AS_FLAGS += -Og  # Optimize for debugging experience (ok to combine with -Os).
C_CXX_AS_FLAGS += -g3
C_CXX_AS_FLAGS += -ggdb
C_CXX_AS_FLAGS += -ffast-math
C_CXX_AS_FLAGS += -fno-finite-math-only
C_CXX_AS_FLAGS += -fmessage-length=100
C_CXX_AS_FLAGS += -flto
C_CXX_AS_FLAGS += -ffunction-sections
C_CXX_AS_FLAGS += -fdata-sections
C_CXX_AS_FLAGS += $(INC_DIRS)
C_CXX_AS_FLAGS += $(MCPU)
C_CXX_AS_FLAGS += $(MARCH)

## Flags for C and C++
C_CXX_FLAGS  = $(C_CXX_AS_FLAGS)
C_CXX_FLAGS += $(DEFS)

## Flags for C
CFLAGS = $(C_CXX_FLAGS)
CFLAGS += $(C_WARNINGS)
CFLAGS += -fno-strict-aliasing

## Flags for C++
CXXFLAGS  = $(C_CXX_FLAGS)
CXXFLAGS += $(CXX_WARNINGS)

## Flags for ASM
ASFLAGS  = $(C_CXX_AS_FLAGS)
ASFLAGS += $(AS_WARNINGS)
ASFLAGS += $(AS_DEFS)
ASFLAGS += -x assembler-with-cpp
ASFLAGS += -Wa,-ggdb

LIB_DIRS  = -L./obj

## Flags for linking
LDFLAGS  = -Wl,-Map,$@.map
LDFLAGS += -Wl,-T$(LD_SCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -nostartfiles
LDFLLGS += -nodefaultlibs
#LDFLAGS += -nostdlib
LDFLAGS += $(LIB_DIRS)
LDFLAGS += --specs=nano.specs
LDFLAGS += --specs=nosys.specs
LDFLAGS += -flto

ARCHIVES ?=
INC_DIRS ?=
SRC_DIRS ?=

include $(TOPDIR)/Sources.mk

LIBS += -lgcc

V ?= 0

ifeq ($(V),0)
  QQ = @
  Q  = @
else
  ifeq ($(V),1)
    QQ = @
    Q  =
  else
    QQ =
    Q  =
  endif
endif

OBJ_SECTIONS += -j .isr_vector
OBJ_SECTIONS += -j .text
OBJ_SECTIONS += -j .rodata
OBJ_SECTIONS += -j .data
OBJ_SECTIONS += -j .preinit_array
OBJ_SECTIONS += -j .init_array
OBJ_SECTIONS += -j .fini_array
OBJ_SECTIONS += -j .ARM.extab
OBJ_SECTIONS += -j .ARM.exidx

GEN_DEPS = -Wp,-M,-MP,-MT,obj/$(*F).o,-MF,obj/.deps/$(@F).d

space := $(empty) $(empty)

VPATH = $(subst $(space),:,$(SRC_DIRS))

OBJS  = $(filter %.o,$(SRC:%.s=obj/%.o))
OBJS += $(filter %.o,$(SRC:%.S=obj/%.o))
OBJS += $(filter %.o,$(SRC:%.c=obj/%.o))
OBJS += $(filter %.o,$(SRC:%.cpp=obj/%.o))
OBJS += $(filter %.o,$(SRC:%.cc=obj/%.o))

DEP_LIBS = $(ARCHIVES:%=obj/lib%.a)
LIBS += $(ARCHIVES:%=-l%)

all: obj/$(PRG).lst obj/$(PRG).bin obj/$(PRG).hex

clean:
	rm -rf obj

.PHONY: all clean

include $(MAKE_TOOLS_DIR)/Version.mk

obj/%.o: %.c
	-@echo "CC $@"
	-$(QQ)$(CC) $(CFLAGS) -E $(GEN_DEPS) -o /dev/null $<
	$(Q)$(CC) $(CFLAGS) -c -o $@ $<

obj/%.o: %.cpp
	-@echo "CXX $@"
	-$(QQ)$(CXX) $(CXXFLAGS) -E $(GEN_DEPS) -o /dev/null $<
	$(Q)$(CXX) $(CXXFLAGS) -c -o $@ $<

obj/%.o: %.cc
	-@echo "CXX $@"
	-$(QQ)$(CXX) $(CXXFLAGS) -E $(GEN_DEPS) -o /dev/null $<
	$(Q)$(CXX) $(CXXFLAGS) -c -o $@ $<

obj/%.o: %.s
	-@echo "AS $@"
	-$(QQ)$(CC) $(ASFLAGS) -E $(GEN_DEPS) -o /dev/null $<
	$(Q)$(CC) $(ASFLAGS) -c -o $@ $<

obj/%.o: %.S
	-@echo "AS $@"
	-$(QQ)$(CC) $(ASFLAGS) -E $(GEN_DEPS) -o /dev/null $<
	$(Q)$(CC) $(ASFLAGS) -c -o $@ $<

obj/%.o: %.asm
	-@echo "AS $@"
	-$(QQ)$(CC) $(ASFLAGS) -E $(GEN_DEPS) -o /dev/null $<
	$(Q)$(CC) $(ASFLAGS) -c -o $@ $<

obj/$(PRG).elf: $(OBJS) $(DEP_LIBS)
	-@echo "LD $@"
	$(Q)$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)
	$(Q)$(SIZE) -B $@

obj/%.lst: obj/%.elf
	-@echo "OBJDUMP $@"
	$(Q)$(OBJDUMP) -dSst $< > $@

obj/%.bin: obj/%.elf
	-@echo "OBJCOPY $@"
	$(Q)$(OBJCOPY) $(OBJ_SECTIONS) -O binary -S $< $@

obj/%.hex: obj/%.elf
	-@echo "OBJCOPY $@"
	$(Q)$(OBJCOPY) $(OBJ_SECTIONS) -O ihex -S $< $@

#
# Automate the generation of rules for building libraries from the
# source files. This template function gets run for each entry in the
# ARCHIVES variable (if it is defined).
#
# The source for each library, <foo>, should be defined in the
# following variables:
#
#   LIB_SRC_<foo>      # C, C++ and ASM source files
#
define GEN_ARCHIVE_RULES_TEMPLATE
  # For debugging: uncomment to get output when template is called
  #$$(info DEBUG - GEN_ARCHIVE_RULES_TEMPLATE: $(1))

  -l$(1): lib$(1).a

  OBJS_$(1)  = $(filter %.o,$(LIB_SRC_$(1):%.s=obj/%.o))
  OBJS_$(1) += $(filter %.o,$(LIB_SRC_$(1):%.S=obj/%.o))
  OBJS_$(1) += $(filter %.o,$(LIB_SRC_$(1):%.c=obj/%.o))
  OBJS_$(1) += $(filter %.o,$(LIB_SRC_$(1):%.cpp=obj/%.o))
  OBJS_$(1) += $(filter %.o,$(LIB_SRC_$(1):%.cc=obj/%.o))

  obj/lib$(1).a: $$(OBJS_$(1))
	-@echo "AR $$@"
	-@rm -f $$@
	$$(Q)$$(AR) $$(ARFLAGS) $$@ $$(OBJS_$(1))
endef
$(foreach src,$(ARCHIVES),$(eval $(call GEN_ARCHIVE_RULES_TEMPLATE,$(src))))

-include $(shell mkdir -p obj/.deps 2>/dev/null) $(wildcard obj/.deps/*)
