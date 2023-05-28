#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "stlfile.h"

// when given an edge array where edges loop around faces, returns an
// array where edges loop around vertices
int
dualedges(halfedge_t *enext, halfedge_t nedges, halfedge_t **vnextp)
{
	halfedge_t ei;
	vertex_t nverts;
	halfedge_t *vnext;

	vnext = malloc(nedges * sizeof vnext[0]);
	for(ei = 0; ei < nedges; ei++)
		vnext[ei] = ~(halfedge_t)0;

	nverts = 0;
	for(ei = 0; ei < nedges; ei++){
		halfedge_t edge;
		edge = ei;
		if(vnext[edge] == ~(halfedge_t)0){
			do {
				halfedge_t tmp;
				tmp = enext[edge^1]; // next on the opposite face.
				vnext[edge] = tmp;
				edge = tmp;
			} while(edge != ei);
			nverts++;
		}
	}

	fprintf(stderr, "dualedges: found %d loops\n", nverts);

	*vnextp = vnext;
	return nverts;
}
