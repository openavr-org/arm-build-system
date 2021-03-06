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

ARCH_WARNINGS   +=

ARCH_DEFS       += -D__CORTEX_M0_PLUS
ARCH_DEFS       += -DTARGET_M0_PLUS
ARCH_DEFS       += -DTARGET_CORTEX_M
ARCH_DEFS       += -DTARGET_CORTEX_M0_PLUS
ARCH_DEFS       += -DARM_MATH_CM0_PLUS
ARCH_DEFS       += -DTOOLCHAIN_GCC_ARM
ARCH_DEFS       += -DTOOLCHAIN_GCC

ARCH_INC_DIRS   +=
ARCH_SRC_DIRS   +=

MCPU            += -mthumb
MCPU            += -mcpu=cortex-m0plus
#MCPU            += -mcpu=cortex-m0plus.small-mulitply
