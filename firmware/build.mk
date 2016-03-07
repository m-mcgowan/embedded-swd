
CSRC += $(call target_files,,*.c)
CPPSRC += examples/Norwegian_Blue.cpp
GLOBAL_DEFINES += PARTICLE_LOCAL_BUILD=1
INCLUDE_DIRS += $(APPDIR)
