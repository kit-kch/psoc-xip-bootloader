#################################################################################################
# << NEORV32 - Application Makefile >>                                                          #
# ********************************************************************************************* #
# Make sure to add the RISC-V GCC compiler's bin folder to your PATH environment variable.      #
# ********************************************************************************************* #
# BSD 3-Clause License                                                                          #
#                                                                                               #
# Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
#                                                                                               #
# Redistribution and use in source and binary forms, with or without modification, are          #
# permitted provided that the following conditions are met:                                     #
#                                                                                               #
# 1. Redistributions of source code must retain the above copyright notice, this list of        #
#    conditions and the following disclaimer.                                                   #
#                                                                                               #
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
#    conditions and the following disclaimer in the documentation and/or other materials        #
#    provided with the distribution.                                                            #
#                                                                                               #
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
#    endorse or promote products derived from this software without specific prior written      #
#    permission.                                                                                #
#                                                                                               #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
# OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
# OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
# ********************************************************************************************* #
# The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
#################################################################################################

# Modify this variable to fit your NEORV32 setup (neorv32 home folder)
NEORV32_HOME ?= ../../neorv32

# Minimal RISC-V ISA (E!) only
MARCH = rv32e_zicsr_zifencei
MABI = ilp32e

# Optimize for minimal size
EFFORT = -Os

# Adjust "rom" memory size and base for BOOTROM
# Just use a minimal "ram" size that should be available on any platform configuration.
# Define MAKE_BOOTLOADER for SW library (reduces footprint)
# Enable link-time-optimization
USER_FLAGS += \
-Wl,--defsym,__neorv32_rom_size=4k \
-Wl,--defsym,__neorv32_rom_base=0xFFE00000 \
-Wl,--defsym,__neorv32_ram_size=256 \
-DMAKE_BOOTLOADER \
-flto

include $(NEORV32_HOME)/sw/common/common.mk
