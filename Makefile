BASE = ../botop/rai
BASE2 = ../botop/src

#OBJS = main.o ../../bin/bot/bot.o

DEPEND = Core Algo Gui Geo Kin Optim KOMO Franka Control BotOp

OPENCV = 1

include $(BASE)/build/generic.mk
