/*
This file is part of ``kdtree'', a library for working with kd-trees.
Copyright (C) 2007-2009 John Tsiombikas <nuclear@siggraph.org>
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#ifndef _KDTREE_H_
#define _KDTREE_H_

struct vec3f {
    float data[3];
    float operator[](int idx) const { return data[idx]; }
};
struct kdhyperrect {
    float *min, *max;
};
struct kdnode {
    float* pos;
    int dir;
    void* data;

    struct kdnode *left, *right;
};
struct res_node {
    struct kdnode* item;
    float dist_sq;
    struct res_node* next;
};
struct kdtree {
    struct kdnode* root;
    struct kdhyperrect* rect;
    void (*destr)(void*);
};
struct kdres {
    struct kdtree* tree;
    struct res_node *rlist, *riter;
    int size;
};
kdtree* kd_create();
void kd_free(struct kdtree* tree);
void kd_clear(struct kdtree* tree);
void kd_data_destructor(struct kdtree* tree, void (*destr)(void*));
int kd_insert(struct kdtree* tree, const float* pos, void* data);
kdres* kd_nearest_range(struct kdtree* kd, const float* pos, float range);
kdres* kd_nearest(struct kdtree* tree, const float* pos);
void kd_res_free(struct kdres* set);
int kd_res_size(struct kdres* set);
void kd_res_rewind(struct kdres* set);
int kd_res_end(struct kdres* set);
int kd_res_next(struct kdres* set);
void* kd_res_item(struct kdres* set, float* pos);

#endif /* _KDTREE_H_ */