##
## @file		makefile
## @brief		GirafFE makefile.
## @author		Sigvald Marholm <sigvaldm@fys.uio.no>
##
## Copyright 2017 Sigvald Marholm <marholm@marebakken.com>
##
## This file is part of GirafFE.
##
## GirafFE is free software: you can redistribute it and/or modify it under
## the terms of the GNU General Public License as published by the Free Software
## Foundation, either version 3 of the License, or (at your option) any later
## version.
##
## GirafFE is distributed in the hope that it will be useful, but WITHOUT ANY
## WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
## FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
## details.
##
## You should have received a copy of the GNU General Public License along with
## GirafFE. If not, see <http://www.gnu.org/licenses/>.

CC		= g++
COPT	= -O3
CADD    =

EXEC    = giraffe

CFLAGS  = -std=c++17 -Wall -fPIC -I../voro++-0.4.6/src $(CADD)

SDIR	= src
ODIR	= src/obj
HDIR	= src
DDIR	= doc

HEAD_	= polyhedron.h GirafFE.h
SRC_	= polyhedron.cpp GirafFE.cpp io.cpp
OBJ_	= $(SRC_:.cpp=.o)
DOC_	= main.dox

objs_=cell.o common.o container.o unitcell.o v_compute.o c_loops.o \
     v_base.o wall.o pre_container.o container_prd.o
objs = $(patsubst %,../voro++-0.4.6/src/%,$(objs_))
src=$(patsubst %.o,%.cc,$(objs))

HEAD	= $(patsubst %,$(HDIR)/%,$(HEAD_))
SRC		= $(patsubst %,$(SDIR)/%,$(SRC_))
OBJ		= $(patsubst %,$(ODIR)/%,$(OBJ_))

all: version $(EXEC) $(EXEC) doc

$(EXEC): $(ODIR)/main.o $(OBJ)
	@echo "Linking GirafFE CLI tool"
	@$(CC) $^ $(objs) -o $@ $(CFLAGS)

$(EXEC).so: $(OBJ)
	@echo "Linking GirafFE library"
	@$(CC) $(OBJ) -shared -o $(EXEC).so

$(ODIR)/%.o: $(SDIR)/%.cpp $(HEAD)
	@echo "Compiling $<"
	@mkdir -p $(ODIR)
	@$(CC) -c $< -o $@ $(CFLAGS)

.phony: version
version:
	@echo "Embedding git version"
	@echo "#define VERSION \"$(shell git describe --abbrev=4 --dirty --always --tags)\"" > $(SDIR)/version.h

$(DDIR)/doxygen/doxyfile.inc: $(DDIR)/doxygen/doxyfile.mk $(DDIR)/doxygen/$(DOC_)
	@echo INPUT	= ../../$(SDIR) ../../$(HDIR) ../../$(TSDIR) ../../$(THDIR) ../../$(DDIR)/doxygen > $(DDIR)/doxygen/doxyfile.inc
	@echo FILE_PATTERNS	= $(HEAD_) $(SRC_) $(DOC_) test.h test.c  >> $(DDIR)/doxygen/doxyfile.inc

doc: $(HEAD) $(SRC) $(DDIR)/doxygen/doxyfile.inc
	@echo "Making documentation (run \"make pdf\" to get pdf)"
	@cd $(DDIR)/doxygen && doxygen doxyfile.mk > /dev/null 2>&1
	@ln -sf doc/html/index.html doc.html

pdf: doc
	@echo "Making PDF"
	cd $(DDIR)/latex && $(MAKE)	# Intentionally verbose to spot LaTeX errors

cleandoc:
	@echo "Cleaning documentation"
	@rm -f $(DDIR)/doxygen/doxyfile.inc
	@rm -fr $(DDIR)/html $(DDIR)/latex
	@rm -f doc.html

clean: cleandoc
	@echo "Cleaning compilation files"
	@rm -f *~ $(TODIR)/*.o $(ODIR)/*.o $(SDIR)/*.o $(SDIR)/*~ gmon.out
