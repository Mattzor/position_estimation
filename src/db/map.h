/*
Copyright (c) 2017, Robert Krook
Copyright (c) 2017, Erik Almblad
Copyright (c) 2017, Hawre Aziz
Copyright (c) 2017, Alexander Branzell
Copyright (c) 2017, Mattias Eriksson
Copyright (c) 2017, Carl Hjerpe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Chalmers University of Technology nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
// Needs to be compiled with flag -std=c++11 


#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

#define POLY_START      "BEGIN POLYGON"
#define POLY_END        "END POLYGON"
#define MARKING_START   "BEGIN MARKING"
#define MARKING_END     "END MARKING" 
#define COMMENT_SIGN    '#'

using namespace std;

struct Marking
{
    int id;
    int x, y;
};

struct Node
{
	int id;
    int x,y;
};

struct Polygon
{
    int numOfNodes;
	vector<Node> nodes;
};

class Map{
    public:
        vector<Polygon> polygons;
        vector<Marking> markings;
        void printPoly(Polygon *poly);
        void printMarking(Marking *marking);
        void printMap();
        void getMarkingPos(int id, int &x, int &y);
        bool isPosInPoly(Polygon *poly, int x, int y);
        Map();        
        
    private:
        void createPoly(ifstream &in, Polygon &poly);
        void createMarking(ifstream &in, Marking &marking);
        bool isCommentLine(string &str);
};




