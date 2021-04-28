local_path := $(call GET_CUR_PATH)
$(call CLEAR_LOCAL_MODULE_VARS)
ZMODULE_NAME = capa_roadwidth
ZMODULE_TYPE = APP
ZMODULE_INTERFACE=
ZMODULE_SRC = \
	 common/configure/config.cpp \
	 src/road_width4.cpp \
	 receive/receive_manager.cpp \
	 main.cpp
ZMODULE_INCLUDE = include \
	common/configure \
	common/data \
	common \
	receive \
	receive/live_msg \
	receive/topic

ZMODULE_CPP_FLAGS = \
    -Dsoc
	-DNUMST=1   \
	-DNCSTATES=0  \
	-DHAVESTDIO  \
	-DTERMFCN=1 \
	-DONESTEPFCN=1 \
	-DMAT_FILE=0 \
	-DMULTI_INSTANCE_CODE=1 \
	-DINTEGER_CODE=0 \
	-DMT=0 \
	-DCLASSIC_INTERFACE=0 \
	-DALLOCATIONFCN=0 \
	-DTID01EQ=0 \
	-DPORTABLE_WORDSIZES=0
ZMODULE_SHARE_LIB =  common_transport core base  capa_node_transport live_msgs_oem_changan_s202da app opencv_core  opencv_imgproc ca_apa_node_transport\
                  protobuf std_msgs ros_deps zm_sdk boost_system pthread
ZMODULE_STATIC_LIB =
$(call ADD_MODULE)
