# makefile for jennic

CFLAGS = -DJN514x=5140 -DJN5148=5148 -msibcall -mno-entri -mno-multi -mno-setcc
CFLAGS += -mno-cmov -mno-carry -mno-subb -mno-sext -mno-ror -mno-hard-div
CFLAGS += -mhard-mul -mbranch-cost=3 -msimple-mul -march=ba2 -mredzone-size=4
CFLAGS += -ffixed-r16 -ffixed-r17 -ffixed-r18 -ffixed-r19 -ffixed-r20
CFLAGS += -ffixed-r21 -ffixed-r22 -ffixed-r23 -ffixed-r24 -ffixed-r25
CFLAGS += -ffixed-r26 -ffixed-r27 -ffixed-r28 -ffixed-r29 -ffixed-r30
CFLAGS += -ffixed-r31 -fomit-frame-pointer -Os -Wall -Wunreachable-code
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -DWATCHDOG_ENABLED -DJENNIC_HW_BBC_RXINCCA=0 -DJENNIC_HW_BBC_DMA=0
CFLAGS += -DJENNIC_HW_BBC_ISA=1 -DJENNIC_SW_EXTERNAL_FLASH=1
CFLAGS += -DJENNIC_CHIP=JN5148 -DJENNIC_CHIP_FAMILY=JN514x

JENLIB = /home/share/tool/Jennic
CFLAGS += -I$(JENLIB)/Platform/Common/Include
CFLAGS += -I$(JENLIB)/Platform/DK2/Include
CFLAGS += -I$(JENLIB)/Components/Common/Include
CFLAGS += -I$(JENLIB)/Components/JennicLogo/Include
CFLAGS += -I$(JENLIB)/Components/Utilities/Include
CFLAGS += -I$(JENLIB)/Components/AppQueueApi/Include
CFLAGS += -I$(JENLIB)/Components/JenNet/Include
CFLAGS += -I$(JENLIB)/Components/Jenie/Include
CFLAGS += -I$(JENLIB)/Components/AppApi/Include
CFLAGS += -I$(JENLIB)/Components/HardwareApi/Include
CFLAGS += -I$(JENLIB)/Components/MAC/Include
CFLAGS += -I$(JENLIB)/Components/TimerServer/Include
CFLAGS += -I$(JENLIB)/Components/Random/Include

LFLAGS = -Wl,--gc-sections -Wl,-u_AppColdStart -Wl,-u_AppWarmStart
LFLAGS += -TApp_Stack_Size.ld -nostartfiles -mba2_elf -march=ba2
LFLAGS += -L$(JENLIB)/Chip/JN5148/Build
LFLAGS += -L$(JENLIB)/Chip/JN5148/Library
LFLAGS += -L$(JENLIB)/Platform/DK2/Library
LFLAGS += -L$(JENLIB)/Components/Library
LFLAGS += -L$(JENLIB)/Stack/JenNet/Build
LFLAGS += -TApp_Stack_Size.ld -TAppBuild_JN5148.ld -TAppBuild_JN5148_End.ld

# coordinator link
LFLAGC = -Wl,--start-group -lJenieCR_JN514x -lMacPatch_JN5148 -lHWPatch_JN5148
LFLAGC += -lRomStdLib_JN5148 -lPDM_JN514x -lBoardLib_JN514x -Wl,--end-group

# router link
LFLAGR = -Wl,--start-group -lJenieCR_JN514x -lMacPatch_JN5148 -lHWPatch_JN5148
LFLAGR += -lRomStdLib_JN5148 -lPDM_JN514x -lBoardLib_JN514x -Wl,--end-group

# end device link
LFLAGN = -Wl,--start-group -lJenieED_JN514x -lMacPatch_JN5148 -lHWPatch_JN5148
LFLAGN += -lRomStdLib_JN5148 -lPDM_JN514x -lBoardLib_JN514x -Wl,--end-group

OFLAGS = -j .flashheader -j .oad -j .mac -j .heap_location -j .rtc_clt
OFLAGS += -j .rodata -j .data -j .text -j .bss -j .heap -j .stack -S -O binary

PATH := /home/share/tool/ba2-elf/bin:$(PATH)
CC = ba-elf-gcc
OC = ba-elf-objcopy

.PHONY: base rotr node

base.elf: LFLAG2 = $(LFLAGC)
rotr.elf: LFLAG2 = $(LFLAGR)
node.elf: LFLAG2 = $(LFLAGN)

all: base rotr node

new: clean all

clean:
	rm -rf *.o *.elf *.bin *.map

base: base.bin

rotr: rotr.bin

node: node.bin

%.bin: %.elf
	$(OC) $(OFLAGS) $< $@

%.elf: %.o Utils.o
	$(CC) -o $@ $(CFLAGS) $+ $(LFLAGS) $(LFLAG2)

%.o: %.c %.h
	$(CC) -c -o $@ $(CFLAGS) $<

%.o: %.c
	$(CC) -c -o $@ $(CFLAGS) $<
