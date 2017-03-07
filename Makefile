# replace this with the top of your X11 tree
# X11 = /exp/rcf/share/X11R5
X11 = /usr/X11R6

############## do not change below this line ####################

XINCDIR = $(X11)/include
XLIBDIR = $(X11)/lib 

XAWLIB = -lXaw
XMULIB = -lXmu
XTOOLLIB = -lXt
XLIB = -lX11
XEXTLIB = -lXext
MATHLIB = -lm

LIBS =  -L$(XLIBDIR) $(XAWLIB) $(XMULIB) $(XTOOLLIB) $(XLIB) $(XEXTLIB) \
	$(MATHLIB)

RM = rm -f
CC = gcc
#CCFLAGS = -c -O $(OPT) -I. -I$(XINCDIR)
CCFLAGS = -c -g -I. -I$(XINCDIR)

.SUFFIXES:	.c	.o

.c.o:	
	$(CC) $(CCFLAGS) $<

############## do not change above this line ####################

PROG1 = x
OFILES1 = xrobot.o \
         3Rarm.o \
         object.o \
	 4D_math.o \
	 control.o

XKWOFILES = Xkw/Canvas.o Xkw/Slider.o Xkw/Xkw.o

HFILES = 3Rarm.h

all:	$(PROG1) $(PROG2)

$(PROG1):	$(OFILES1)
	$(CC) -o $@ $(OFILES1) $(XKWOFILES) $(LIBS)

clean:
	$(RM) $(OFILES1) $(PROG1) *~

xrobot.o:	Xkw/Xkw.h 3Rarm.h

3Rarm.o:	3Rarm.h

object.o:	3Rarm.h

control.o:	3Rarm.h


