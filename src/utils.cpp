/**
 * @file		utils.cpp
 * @brief		Utilities for GirafFE
 * @author		Sigvald Marholm <sigvaldm@fys.uio.no>,
 */

/*
 * Copyright 2017 Sigvald Marholm <marholm@marebakken.com>
 *
 * This file is part of GirafFE.
 *
 * GirafFE is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * GirafFE is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * GirafFE. If not, see <http://www.gnu.org/licenses/>.
 */

#include "GirafFE.h"
#include "utils.h"
#include <cstring>
#include <sys/stat.h>

namespace gfe {

const char* argopt(int argc, const char *const *argv, char key){
	for(int i=1; i<argc; i++){
		const char *c = argv[i];
		if(*c!='-' || *(c+1)=='-') continue;
		while(*++c) if(*c==key) return argv[(i+1)%argc];
	}
	return 0;
}

const char* argoptl(int argc, const char *const *argv, const char *key){
	for(int i=1; i<argc; i++){
		const char *c = argv[i], *k = key;
		if(*c++!='-' || *c!='-') continue;
		while(*c){
			if(*++c!=*k++) break;
			if(!*c) return argv[(i+1)%argc];
		}
	}
	return 0;
}

bool fexist(const std::string& filename){
    struct stat buf;
	return stat(filename.c_str(), &buf) != -1;
}

void Progress::update(){

	unsigned int oldPos = pos_;
	pos_ = progress_*width_/end_;

	unsigned int oldFrac = frac_;
	frac_ = progress_*100/end_;

	if(pos_!=oldPos || (frac_!= oldFrac && percentage_)){

		out_ << "\r" << label_ << " [";
		out_ << std::string(pos_, '=');
		out_ << std::string(width_-pos_, ' ');
		out_ << "]";
		if(percentage_){
			out_ << " " << frac_ << "%";
		}
	}
	if(progress_>=end_) out_ << "\n";
	out_.flush();
}

} // namespace gfe
