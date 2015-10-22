/*
* The author of this software is Shane O'Sullivan.  
* Permission to use, copy, modify, and distribute this software for any
* purpose without fee is hereby granted, provided that this entire notice
* is included in all copies of any software which is or includes a copy
* or modification of this software and in all copies of the supporting
* documentation for such software.
* THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
* WARRANTY.  IN PARTICULAR, NEITHER THE AUTHORS NOR AT&T MAKE ANY
* REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY
* OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
*/


#
#include <stdio.h>
#include <search.h>
#include <malloc.h>
#include "VoronoiDiagramGenerator.h"



int main(int argc,char **argv) 
{	

	float xValues[4] = {-22, -17, 4,22};
	float yValues[4] = {-9, 31,13,-5};
	
	long count = 4;

	VoronoiDiagramGenerator vdg;
	vdg.generateVoronoi(xValues,yValues,count, -100,100,-100,100,3);

	vdg.resetIterator();

	float x1,y1,x2,y2;

	printf("\n-------------------------------\n");
	while(vdg.getNext(x1,y1,x2,y2))
	{
		printf("GOT Line (%f,%f)->(%f,%f)\n",x1,y1,x2, y2);
		
	}

	return 0;

	
}



