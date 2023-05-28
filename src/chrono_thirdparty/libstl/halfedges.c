#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "hash64.h"
#include "stlfile.h"

static halfedge_t
edge(vertex_t *edges, halfedge_t nedges, halfedge_t *eht, halfedge_t ehtcap, vertex_t *e)
{
	halfedge_t *eip, ei;
	halfedge_t hash;
	halfedge_t i;

	hash = final64(e[0], e[1]);
	for(i = 0; i < ehtcap; i++){
		eip = eht + ((hash + i) & (ehtcap - 1));
		ei = *eip;
		if(ei == 0){
			*eip = nedges+1;
			return nedges;
		}
		ei--;
		if(cmp64(e, edges + 2*ei) == 0)
			return ei;
	}
	return ~(halfedge_t)0;
}

int
halfedges(vertex_t *triverts, triangle_t ntris, halfedge_t **nextp, vertex_t **vertp, halfedge_t *nedgep)
{
	vertex_t *vert, *edges;
	halfedge_t *next, *eht;
	halfedge_t nedges, ehtcap;
	triangle_t i, ti;

	nedges = 0;
	next = malloc(3*ntris * 2*sizeof next[0]);
	vert = malloc(3*ntris * 2*sizeof vert[0]);

	edges = malloc(3*ntris * 2*sizeof edges[0]);
	ehtcap = nextpow2(3*ntris);
	eht = malloc(ehtcap * sizeof eht[0]);
	memset(eht, 0, ehtcap * sizeof eht[0]);
	for(i = 0; i < ntris; i++){
		vertex_t vloop[3];
		halfedge_t eloop[3];
		for(ti = 0; ti < 3; ti++){
			vertex_t v0, v1, e[2];
			halfedge_t ei;
			v0 = triverts[3*i+ti];
			v1 = triverts[3*i+((1<<ti)&3)];
			e[0] = v0 < v1 ? v0 : v1; // min
			e[1] = v0 < v1 ? v1 : v0; // the other
			ei = edge(edges, nedges, eht, ehtcap, e);
			if(ei == ~(halfedge_t)0){
				fprintf(stderr, "halfedges: hash full at triangle %d/%d cap %d\n", i, ntris, ehtcap);
				goto exit_fail;
			}
			if(ei == nedges){
				// number of edges increased,
				// 'lazy' initialize entries to known bad indices.
				next[2*ei] = ~(halfedge_t)0;
				next[2*ei+1] = ~(halfedge_t)0;
				vert[2*ei] = ~(vertex_t)0;
				vert[2*ei+1] = ~(vertex_t)0;
				copy64(edges + 2*nedges, e);
				nedges++;
				eloop[ti] = 2*ei;
			} else {
				eloop[ti] = 2*ei+1;
			}
			vloop[ti] = v0;
		}
		for(ti = 0; ti < 3; ti++){
			next[eloop[ti]] = eloop[(1<<ti)&3];
			vert[eloop[ti]] = vloop[ti];
		}
	}

	// scan through, warn about orphan edges.
	for(i = 0; i < 2*nedges; i++){
		if(next[i] == ~(halfedge_t)0){
			fprintf(stderr, "halfedges: orphan edge");
			next[i] = i;
		}
	}

	free(eht);
	free(edges);
	next = realloc(next, nedges * 2*sizeof next[0]);
	vert = realloc(vert, nedges * 2*sizeof vert[0]);
	*nextp = next;
	*vertp = vert;
	*nedgep = 2*nedges;
	fprintf(stderr, "halfedges: found %d full edges\n", nedges);
	return 0;

exit_fail:
	free(eht);
	free(edges);
	free(next);
	free(vert);
	return -1;
}
