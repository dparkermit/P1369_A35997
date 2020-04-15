#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=cof
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/A35997.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/A35997.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../Main.c ../A35997.c ../ETM_BUFFER_BYTE_64.c ../ETM_DSP_FUNCTIONS.s ../ETM_IO_PORTS.s ../ETM_SPI.c ../faults_A35997.c ../LTC2656.c ../serial_A35997.c ../A35997_assembly_functions.s

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1472/Main.o ${OBJECTDIR}/_ext/1472/A35997.o ${OBJECTDIR}/_ext/1472/ETM_BUFFER_BYTE_64.o ${OBJECTDIR}/_ext/1472/ETM_DSP_FUNCTIONS.o ${OBJECTDIR}/_ext/1472/ETM_IO_PORTS.o ${OBJECTDIR}/_ext/1472/ETM_SPI.o ${OBJECTDIR}/_ext/1472/faults_A35997.o ${OBJECTDIR}/_ext/1472/LTC2656.o ${OBJECTDIR}/_ext/1472/serial_A35997.o ${OBJECTDIR}/_ext/1472/A35997_assembly_functions.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1472/Main.o.d ${OBJECTDIR}/_ext/1472/A35997.o.d ${OBJECTDIR}/_ext/1472/ETM_BUFFER_BYTE_64.o.d ${OBJECTDIR}/_ext/1472/ETM_DSP_FUNCTIONS.o.d ${OBJECTDIR}/_ext/1472/ETM_IO_PORTS.o.d ${OBJECTDIR}/_ext/1472/ETM_SPI.o.d ${OBJECTDIR}/_ext/1472/faults_A35997.o.d ${OBJECTDIR}/_ext/1472/LTC2656.o.d ${OBJECTDIR}/_ext/1472/serial_A35997.o.d ${OBJECTDIR}/_ext/1472/A35997_assembly_functions.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1472/Main.o ${OBJECTDIR}/_ext/1472/A35997.o ${OBJECTDIR}/_ext/1472/ETM_BUFFER_BYTE_64.o ${OBJECTDIR}/_ext/1472/ETM_DSP_FUNCTIONS.o ${OBJECTDIR}/_ext/1472/ETM_IO_PORTS.o ${OBJECTDIR}/_ext/1472/ETM_SPI.o ${OBJECTDIR}/_ext/1472/faults_A35997.o ${OBJECTDIR}/_ext/1472/LTC2656.o ${OBJECTDIR}/_ext/1472/serial_A35997.o ${OBJECTDIR}/_ext/1472/A35997_assembly_functions.o

# Source Files
SOURCEFILES=../Main.c ../A35997.c ../ETM_BUFFER_BYTE_64.c ../ETM_DSP_FUNCTIONS.s ../ETM_IO_PORTS.s ../ETM_SPI.c ../faults_A35997.c ../LTC2656.c ../serial_A35997.c ../A35997_assembly_functions.s


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/A35997.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=30F6014A
MP_LINKER_FILE_OPTION=,--script=p30F6014A.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1472/Main.o: ../Main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/Main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/Main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Main.c  -o ${OBJECTDIR}/_ext/1472/Main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/Main.o.d"      -g -D__DEBUG     -omf=coff -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/Main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/A35997.o: ../A35997.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/A35997.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/A35997.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../A35997.c  -o ${OBJECTDIR}/_ext/1472/A35997.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/A35997.o.d"      -g -D__DEBUG     -omf=coff -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/A35997.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/ETM_BUFFER_BYTE_64.o: ../ETM_BUFFER_BYTE_64.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/ETM_BUFFER_BYTE_64.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ETM_BUFFER_BYTE_64.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../ETM_BUFFER_BYTE_64.c  -o ${OBJECTDIR}/_ext/1472/ETM_BUFFER_BYTE_64.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/ETM_BUFFER_BYTE_64.o.d"      -g -D__DEBUG     -omf=coff -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ETM_BUFFER_BYTE_64.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/ETM_SPI.o: ../ETM_SPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/ETM_SPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ETM_SPI.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../ETM_SPI.c  -o ${OBJECTDIR}/_ext/1472/ETM_SPI.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/ETM_SPI.o.d"      -g -D__DEBUG     -omf=coff -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ETM_SPI.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/faults_A35997.o: ../faults_A35997.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/faults_A35997.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/faults_A35997.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../faults_A35997.c  -o ${OBJECTDIR}/_ext/1472/faults_A35997.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/faults_A35997.o.d"      -g -D__DEBUG     -omf=coff -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/faults_A35997.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/LTC2656.o: ../LTC2656.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/LTC2656.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/LTC2656.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../LTC2656.c  -o ${OBJECTDIR}/_ext/1472/LTC2656.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/LTC2656.o.d"      -g -D__DEBUG     -omf=coff -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/LTC2656.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/serial_A35997.o: ../serial_A35997.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/serial_A35997.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/serial_A35997.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../serial_A35997.c  -o ${OBJECTDIR}/_ext/1472/serial_A35997.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/serial_A35997.o.d"      -g -D__DEBUG     -omf=coff -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/serial_A35997.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/1472/Main.o: ../Main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/Main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/Main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Main.c  -o ${OBJECTDIR}/_ext/1472/Main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/Main.o.d"        -g -omf=coff -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/Main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/A35997.o: ../A35997.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/A35997.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/A35997.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../A35997.c  -o ${OBJECTDIR}/_ext/1472/A35997.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/A35997.o.d"        -g -omf=coff -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/A35997.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/ETM_BUFFER_BYTE_64.o: ../ETM_BUFFER_BYTE_64.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/ETM_BUFFER_BYTE_64.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ETM_BUFFER_BYTE_64.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../ETM_BUFFER_BYTE_64.c  -o ${OBJECTDIR}/_ext/1472/ETM_BUFFER_BYTE_64.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/ETM_BUFFER_BYTE_64.o.d"        -g -omf=coff -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ETM_BUFFER_BYTE_64.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/ETM_SPI.o: ../ETM_SPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/ETM_SPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ETM_SPI.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../ETM_SPI.c  -o ${OBJECTDIR}/_ext/1472/ETM_SPI.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/ETM_SPI.o.d"        -g -omf=coff -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ETM_SPI.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/faults_A35997.o: ../faults_A35997.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/faults_A35997.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/faults_A35997.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../faults_A35997.c  -o ${OBJECTDIR}/_ext/1472/faults_A35997.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/faults_A35997.o.d"        -g -omf=coff -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/faults_A35997.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/LTC2656.o: ../LTC2656.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/LTC2656.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/LTC2656.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../LTC2656.c  -o ${OBJECTDIR}/_ext/1472/LTC2656.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/LTC2656.o.d"        -g -omf=coff -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/LTC2656.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/serial_A35997.o: ../serial_A35997.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/serial_A35997.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/serial_A35997.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../serial_A35997.c  -o ${OBJECTDIR}/_ext/1472/serial_A35997.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/serial_A35997.o.d"        -g -omf=coff -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/serial_A35997.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1472/ETM_DSP_FUNCTIONS.o: ../ETM_DSP_FUNCTIONS.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/ETM_DSP_FUNCTIONS.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ETM_DSP_FUNCTIONS.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../ETM_DSP_FUNCTIONS.s  -o ${OBJECTDIR}/_ext/1472/ETM_DSP_FUNCTIONS.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG   -omf=coff -DXPRJ_default=$(CND_CONF)    -I".." -I"." -Wa,-MD,"${OBJECTDIR}/_ext/1472/ETM_DSP_FUNCTIONS.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,,-g,--no-relax,-g$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ETM_DSP_FUNCTIONS.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1472/ETM_IO_PORTS.o: ../ETM_IO_PORTS.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/ETM_IO_PORTS.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ETM_IO_PORTS.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../ETM_IO_PORTS.s  -o ${OBJECTDIR}/_ext/1472/ETM_IO_PORTS.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG   -omf=coff -DXPRJ_default=$(CND_CONF)    -I".." -I"." -Wa,-MD,"${OBJECTDIR}/_ext/1472/ETM_IO_PORTS.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,,-g,--no-relax,-g$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ETM_IO_PORTS.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1472/A35997_assembly_functions.o: ../A35997_assembly_functions.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/A35997_assembly_functions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/A35997_assembly_functions.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../A35997_assembly_functions.s  -o ${OBJECTDIR}/_ext/1472/A35997_assembly_functions.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG   -omf=coff -DXPRJ_default=$(CND_CONF)    -I".." -I"." -Wa,-MD,"${OBJECTDIR}/_ext/1472/A35997_assembly_functions.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,,-g,--no-relax,-g$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/A35997_assembly_functions.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/_ext/1472/ETM_DSP_FUNCTIONS.o: ../ETM_DSP_FUNCTIONS.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/ETM_DSP_FUNCTIONS.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ETM_DSP_FUNCTIONS.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../ETM_DSP_FUNCTIONS.s  -o ${OBJECTDIR}/_ext/1472/ETM_DSP_FUNCTIONS.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=coff -DXPRJ_default=$(CND_CONF)    -I".." -I"." -Wa,-MD,"${OBJECTDIR}/_ext/1472/ETM_DSP_FUNCTIONS.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ETM_DSP_FUNCTIONS.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1472/ETM_IO_PORTS.o: ../ETM_IO_PORTS.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/ETM_IO_PORTS.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ETM_IO_PORTS.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../ETM_IO_PORTS.s  -o ${OBJECTDIR}/_ext/1472/ETM_IO_PORTS.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=coff -DXPRJ_default=$(CND_CONF)    -I".." -I"." -Wa,-MD,"${OBJECTDIR}/_ext/1472/ETM_IO_PORTS.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ETM_IO_PORTS.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1472/A35997_assembly_functions.o: ../A35997_assembly_functions.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/A35997_assembly_functions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/A35997_assembly_functions.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../A35997_assembly_functions.s  -o ${OBJECTDIR}/_ext/1472/A35997_assembly_functions.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=coff -DXPRJ_default=$(CND_CONF)    -I".." -I"." -Wa,-MD,"${OBJECTDIR}/_ext/1472/A35997_assembly_functions.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/A35997_assembly_functions.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/A35997.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../libdsp-coff.a ../libdsp-elf.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/A35997.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    ..\libdsp-coff.a ..\libdsp-elf.a  -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG   -omf=coff -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,,,--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="..",--library-path=".",--no-force-link,--smart-io,-Map="${DISTDIR}/A35997.X.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/A35997.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../libdsp-coff.a ../libdsp-elf.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/A35997.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    ..\libdsp-coff.a ..\libdsp-elf.a  -mcpu=$(MP_PROCESSOR_OPTION)        -omf=coff -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="..",--library-path=".",--no-force-link,--smart-io,-Map="${DISTDIR}/A35997.X.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/A35997.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=coff  
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
