##
## @file		makefile
## @brief		FEdensity makefile.
## @author		Sigvald Marholm <sigvaldm@fys.uio.no>
##
## Copyright 2017 Sigvald Marholm <marholm@marebakken.com>
##
## This file is part of FEdensity.
##
## FEdensity is free software: you can redistribute it and/or modify it under
## the terms of the GNU General Public License as published by the Free Software
## Foundation, either version 3 of the License, or (at your option) any later
## version.
##
## FEdensity is distributed in the hope that it will be useful, but WITHOUT ANY
## WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
## FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
## details.
##
## You should have received a copy of the GNU General Public License along with
## FEdensity. If not, see <http://www.gnu.org/licenses/>.

CC		= g++
COPT	= -O3
CADD    =

EXEC = fedensity

CFLAGS = -std=c++17 -Wall $(CADD)

SDIR	= src
ODIR	= src/obj
HDIR	= src
DDIR	= doc

HEAD_	= polyhedron.h FEdensity.h
SRC_	= polyhedron.cpp FEdensity.cpp io.cpp
OBJ_	= $(SRC_:.cpp=.o)
DOC_	= main.dox

HEAD	= $(patsubst %,$(HDIR)/%,$(HEAD_))
SRC		= $(patsubst %,$(SDIR)/%,$(SRC_))
OBJ		= $(patsubst %,$(ODIR)/%,$(OBJ_))

all: version $(EXEC) doc

$(EXEC): $(ODIR)/main.o $(OBJ)
	@echo "Linking FEdensity"
	@$(CC) $^ -o $@ $(CFLAGS)
	@echo "FEdensity is built"

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
