/**
 * @file		main.cpp
 * @brief		Stand-alone GirafFE command line tool
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

#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>
#include <numeric>
#include <vector>
#include <map>
#include <algorithm>
#include "GirafFE.h"
#include "utils.h"
#include "polyhedron.h"
using std::cout;
using std::cerr;
using std::vector;
using std::string;
using std::map;
using poly::Point;
using namespace gfe;

#include <array>

int main(int argc, char *argv[]){

    cout << "GirafFE " << VERSION << ".\n";

	if(argopt(argc, argv, 'h')){

		cout << "Copyright: 2017 Sigvald Marholm <marholm@marebakken.com>\n";
		cout << "Licence: GPL 3\n";
		cout << "Usage: giraffe [options] input\n";

		cout << "\n";
		cout << "Input file formats supported:\n";
		cout << "    .msh    Gmsh\n";
		cout << "    .gfe    GirafFE\n";

		cout << "\n";
		cout << "Default output name: same as input name with .out appended.\n";

		cout << "\n";
		cout << "Options:\n";
		cout << "    -h  Help (this)\n";
		cout << "    -p  Test if input mesh is Pitteway\n";
		cout << "    -d  Test if input mesh is Delaunay\n";
		cout << "    -s  Output statistics about circumcenter propagation\n";
		cout << "    -x  Output data in lossless hexadecimal representation\n";
		cout << "    -o  Output file name (e.g. `-o volume.txt`)\n";
		cout << "    -w  Overwrite existing files\n";

		return 0;
	}

	/*
	 * GET INPUT FILE
	 */

	if(argc < 2){
        cerr << "Error: No input file sepcified. See giraffe -h.\n";
        return 1;
    }

	string ifname = argv[argc-1];
	std::ifstream infile(ifname);
	if(!infile.good()){
		cerr << "Error: Input file " << ifname << " not accessible.\n";
		return 1;
	}

	string ifext = ifname.substr(ifname.find_last_of(".")+1);
	Mesh mesh;
	if(ifext=="msh"){
		readGmsh(infile, mesh);
	} else if(ifext=="gfe"){
		readFE(infile, mesh);
	} else {
		cerr << "Input file extension ." << ifext << " not supported.\n";
		return 1;
	}

	/*
	 * ALTERNATIVE MODES
	 */

	if(argopt(argc, argv, 'x')){
 		cout << std::hexfloat;
 	}

	if(argopt(argc, argv, 'p')){
		cout << "Pitteway check not implemented yet\n";
		return 0;
	}

	if(argopt(argc, argv, 'd')){
		cout << "Delaunay check not implemented yet\n";
		return 0;
	}

	if(argopt(argc, argv, 's')){

		GraphNode<size_t> stat = mesh.computeInfluencersStatistics();

		vector<size_t> propDepth = propagationDepth(stat);
		map<size_t, int> count = countUnique(propDepth);

		cout << "Number of circumcenters with propagation depth...\n";
		cout << count << "\n";

		map<size_t, double> fraction;
		for(const auto& it : count){
			fraction[it.first] = (double)it.second / mesh.cells.size();
		}

		cout << "In fractions...\n";
		cout << fraction << "\n";

		count.clear();
		for(const auto& cell : mesh.cells){
			count[cell.influencers.size()]++;
		}

		cout << "Number of cells with number of influencers...\n";
		cout << count << "\n";

		fraction.clear();
		for(const auto& it : count){
			fraction[it.first] = (double)it.second / mesh.cells.size();
		}

		cout << "In fractions...\n";
		cout << fraction << "\n";

		std::vector<size_t> branching = propagationMaxBranching(stat);
		auto maxBranching = std::max_element(branching.begin(), branching.end());
		cout << "Highest degree of branching: " << *maxBranching << "\n\n";

		return 0;

	}

	/*
 	 * GET OUTPUT FILE
	 */

	string ofname;
	if(const char *f = argopt(argc, argv, 'o')){
		ofname = f;
	} else {
		ofname = ifname+".out";
	}

	if(fexist(ofname) && !argopt(argc, argv, 'w')){
		cerr << "Output file " << ofname << " already exists.\n";
		return 1;
	}
	std::ofstream outfile(ofname, std::ofstream::out);

	if(argopt(argc, argv, 'x')){
		outfile << std::hexfloat;
	}

	/*
	 * VORONOI COMPUTATION
	 */

	mesh.computeInfluencers();
    vector<double> volume = mesh.volume();
    double totalVolume = std::accumulate(volume.begin(), volume.end(), 0.0f);
    cout << "Total volume: " << totalVolume << "\n";

    writeVector(outfile, volume);

    cout << "GirafFE " << VERSION << " ended successfully.\n";
    return 0;
}
