
typedef uint32_t vertex_t;
typedef uint32_t triangle_t;
typedef uint32_t halfedge_t;

// load stl file, compute and return an indexed triangle mesh
int loadstl(FILE *fp, char *comment, float **vertp, vertex_t *nvertp, vertex_t **trip, uint16_t **attrp, triangle_t *ntrip);

/*
// compute halfedges from an indexed triangle mesh
int halfedges(vertex_t *triverts, triangle_t ntris, halfedge_t **nextp, vertex_t **evertp, halfedge_t *nedgep);

// compute dual edges from half edges (return vertex loops if passed face loops, vice versa)
int dualedges(halfedge_t *enext, halfedge_t nedges, halfedge_t **vnextp);
*/