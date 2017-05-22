/**
 * @file		utils.h
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

 #ifndef UTILS_H
 #define UTILS_H

#include "GirafFE.h"

namespace gfe {

template <typename T>
std::ostream& output(std::ostream& out, const GraphNode<T>& node, std::string acc=""){

    if(node.value!=(size_t)(-1))
        acc += " "+std::to_string(node.value);

    if(node.children.empty()){
        out << acc << "\n";
    } else {
        for(const auto& child : node.children)
            output(out, child, acc);
    }

    return out;
}

template <typename T>
size_t graphDepth(const GraphNode<T>& node){
	size_t childDepth = 0;
	for(const auto& child : node.children){
		childDepth = std::max(childDepth, graphDepth(child));
	}
	return childDepth+1;
}

template <typename T>
size_t graphMaxBranching(const GraphNode<T>& node){
	size_t branching = node.children.size();
	for(const auto& child : node.children){
		branching = std::max(branching, graphMaxBranching(child));
	}
	return branching;
}

template <typename T>
std::vector<size_t> propagationDepth(const GraphNode<T>& root){
	std::vector<size_t> depth;
	depth.reserve(root.children.size());

	for(const auto& child : root.children)
		depth.push_back(graphDepth(child)-1);

	return depth;
}

template <typename T>
std::vector<size_t> propagationMaxBranching(const GraphNode<T>& root){
	std::vector<size_t> branching;
	branching.reserve(root.children.size());

	for(const auto& child : root.children)
		branching.push_back(graphMaxBranching(child));

	return branching;
}

template <typename T>
std::map<T, int> countUnique(std::vector<T> &v){
	std::map<T, int> count;
	for(const auto& elem : v)
		count[elem]++;

	return count;
}

template <typename K, typename V>
std::ostream& operator<<(std::ostream& out, const std::map<K, V>& count){
	for(const auto& it : count)
		out << it.first << ": " << it.second << "\n";
	return out;
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const GraphNode<T>& node){
    return output(out, node);
}

// int argopt(int argc, const char *const *argv, const char *key);
// int argget(int argc, const char *const *argv, int index);

const char* argopt(int argc, const char *const *argv, char key);
const char* argoptl(int argc, const char *const *argv, const char *key);
bool fexist(const std::string& filename);


} // namespace gfe

#endif // UTILS_H
