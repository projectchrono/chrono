
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "stlfile.h"
#include "hash96.h"

static uint32_t
get16(uint8_t *buf)
{
	return (uint32_t)buf[0] + ((uint32_t)buf[1]<<8);
}

static uint32_t
get32(uint8_t *buf)
{
	return (uint32_t)buf[0] + ((uint32_t)buf[1]<<8) + ((uint32_t)buf[2]<<16) + ((uint32_t)buf[3]<<24);
}

static vertex_t
vertex(uint32_t *verts, vertex_t nverts, vertex_t *vht, vertex_t vhtcap, uint32_t *vert)
{
	vertex_t *vip, vi;
	vertex_t hash;
	vertex_t i;

	hash = final96(vert[0], vert[1], vert[2]);
	for(i = 0; i < vhtcap; i++){
		vip = vht + ((hash + i) & (vhtcap - 1));
		vi = *vip;
		if(vi == 0){
			*vip = nverts+1;
			return nverts;
		}
		vi--;
		if(cmp96(vert, verts + 3*vi) == 0)
			return vi;

	}
	return ~(vertex_t)0;
}

int
loadstl(FILE *fp, char *comment, float **vertp, vertex_t *nvertp, vertex_t **trip, uint16_t **attrp, triangle_t *ntrip)
{
	uint8_t buf[128];
	triangle_t i, ti;
	vertex_t *tris;
	triangle_t ntris;
	vertex_t *vht, vi, nverts, vhtcap;
	uint32_t *verts;
	uint16_t *attrs;

	// the comment and triangle count
	if(fread(buf, 84, 1, fp) != 1){
		fprintf(stderr, "loadstl: short read at header\n");
		return -1;
	}

	if(comment != NULL)
		memcpy(comment, buf, 80);

	ntris = get32(buf+80);

	tris = malloc(ntris * 3*sizeof tris[0]);
	attrs = malloc(ntris * sizeof attrs[0]);
	verts = malloc(3*ntris * 3*sizeof verts[0]);

	vhtcap = nextpow2(4*ntris);
	vht = malloc(vhtcap * sizeof vht[0]);
	memset(vht, 0, vhtcap * sizeof vht[0]);

	fprintf(stderr, "loadstl: number of triangles: %u, vhtcap %d\n", ntris, vhtcap);

	nverts = 0;
	for(i = 0; i < ntris; i++){
		if(fread(buf, 50, 1, fp) != 1){
			fprintf(stderr, "loadstl: short read at triangle %d/%d\n", i, ntris);
			goto exit_fail;
		}
		// there's a normal vector at buf[0..11] which we are ignoring
		for(ti = 0; ti < 3; ti++){
			uint32_t vert[3];
			vert[0] = get32(buf+12 + 4*3*ti);
			vert[1] = get32(buf+12 + 4*3*ti+4);
			vert[2] = get32(buf+12 + 4*3*ti+8);
			vi = vertex(verts, nverts, vht, vhtcap, vert);
			if(vi == ~(uint32_t)0){
				fprintf(stderr, "loadstl: vertex hash full at triangle %d/%d\n", i, ntris);
				goto exit_fail;
			}
			if(vi == nverts){
				copy96(verts + 3*nverts, vert);
				nverts++;
			} else {
			}
			tris[3*i+ti] = vi;
		}
		attrs[i] = get16(buf + 48);
	}

	fprintf(stderr, "loadstl: number of verts: %u\n", nverts);
	free(vht);
	verts = realloc(verts, nverts * 3*sizeof verts[0]);
	*vertp = (float *)verts;
	*nvertp = nverts;
	*trip = tris;
	*attrp = attrs;
	*ntrip = ntris;
	return 0;

exit_fail:
	free(vht);
	free(verts);
	free(tris);
	free(attrs);
	return -1;
}
