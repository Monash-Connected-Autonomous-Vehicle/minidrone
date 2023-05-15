# Include the nanopb provided Makefile rules
# include ../../extra/nanopb.mk
include nanopb.mk

# Compiler flags to enable all warnings & debug info
CFLAGS = -Wall -Werror -g -O0
CFLAGS += "-I$(NANOPB_DIR)"

# C source code files that are required
# CSRC  = simple.c                   # The main program
CSRC += minidrone.pb.c                # The compiled protocol definition
CSRC += pb_encode.c  # The nanopb encoder
CSRC += pb_decode.c  # The nanopb decoder
CSRC += pb_common.c  # The nanopb common parts

# Build rule for the main program
minidrone: $(CSRC)
	$(CC) $(CFLAGS) -ominidrone $(CSRC)

# Build rule for the protocol
minidrone.pb.c: minidrone.proto
	$(PROTOC) $(PROTOC_OPTS) --nanopb_out=. minidrone.proto
