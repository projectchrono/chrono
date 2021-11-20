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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Kdtree.h"

#include <stdio.h>

// Disable MSVC C4706 warning "assignment within conditional expression"
#pragma warning(disable : 4706)

void clear_rec(struct kdnode* node, void (*destr)(void*));
int insert_rec(struct kdnode** node, const float* pos, void* data, int dir);
int rlist_insert(struct res_node* list, struct kdnode* item, float dist_sq);
void clear_results(struct kdres* set);
kdhyperrect* hyperrect_create(const float* min, const float* max);
void hyperrect_free(struct kdhyperrect* rect);
kdhyperrect* hyperrect_duplicate(const struct kdhyperrect* rect);
void hyperrect_extend(struct kdhyperrect* rect, const float* pos);
float hyperrect_dist_sq(struct kdhyperrect* rect, const float* pos);

struct kdtree* kd_create() {
    struct kdtree* tree;

    if (!(tree = new kdtree)) {
        return 0;
    }

    tree->root = 0;
    tree->destr = 0;
    tree->rect = 0;

    return tree;
}

void kd_free(struct kdtree* tree) {
    if (tree) {
        kd_clear(tree);
        delete tree;
    }
}

void clear_rec(struct kdnode* node, void (*destr)(void*)) {
    if (!node)
        return;

    clear_rec(node->left, destr);
    clear_rec(node->right, destr);

    if (destr) {
        destr(node->data);
    }
    delete[] node->pos;
    delete node;
}

void kd_clear(struct kdtree* tree) {
    clear_rec(tree->root, tree->destr);
    tree->root = 0;

    if (tree->rect) {
        hyperrect_free(tree->rect);
        tree->rect = 0;
    }
}

void kd_data_destructor(struct kdtree* tree, void (*destr)(void*)) {
    tree->destr = destr;
}

int insert_rec(struct kdnode** nptr, const float* pos, void* data, int dir) {
    int new_dir;
    struct kdnode* node;

    if (!*nptr) {
        if (!(node = new kdnode)) {
            return -1;
        }
        if (!(node->pos = new float[3])) {
            delete[] node;
            return -1;
        }
        memcpy(node->pos, pos, 3 * sizeof *node->pos);
        node->data = data;
        node->dir = dir;
        node->left = node->right = 0;
        *nptr = node;
        return 0;
    }

    node = *nptr;
    new_dir = (node->dir + 1) % 3;
    if (pos[node->dir] < node->pos[node->dir]) {
        return insert_rec(&(*nptr)->left, pos, data, new_dir);
    }
    return insert_rec(&(*nptr)->right, pos, data, new_dir);
}

int kd_insert(struct kdtree* tree, const float* pos, void* data) {
    if (insert_rec(&tree->root, pos, data, 0)) {
        return -1;
    }

    if (tree->rect == 0) {
        tree->rect = hyperrect_create(pos, pos);
    } else {
        hyperrect_extend(tree->rect, pos);
    }

    return 0;
}

int find_nearest(struct kdnode* node, const float* pos, float range, struct res_node* list, int ordered) {
    float dist_sq, dx;
    int i, ret, added_res = 0;

    if (!node)
        return 0;

    dist_sq = 0;
    for (i = 0; i < 3; i++) {
        dist_sq += (node->pos[i] - pos[i]) * (node->pos[i] - pos[i]);
    }
    if (dist_sq <= range * range) {
        if (rlist_insert(list, node, ordered ? dist_sq : -1.0) == -1) {
            return -1;
        }
        added_res = 1;
    }

    dx = pos[node->dir] - node->pos[node->dir];

    ret = find_nearest(dx <= 0.0 ? node->left : node->right, pos, range, list, ordered);
    if (ret >= 0 && fabs(dx) < range) {
        added_res += ret;
        ret = find_nearest(dx <= 0.0 ? node->right : node->left, pos, range, list, ordered);
    }
    if (ret == -1) {
        return -1;
    }
    added_res += ret;

    return added_res;
}

void kd_nearest_i(struct kdnode* node,
                  const float* pos,
                  struct kdnode** result,
                  float* result_dist_sq,
                  struct kdhyperrect* rect) {
    int dir = node->dir;
    int i;
    float dummy, dist_sq;
    struct kdnode *nearer_subtree, *farther_subtree;
    float *nearer_hyperrect_coord, *farther_hyperrect_coord;

    dummy = pos[dir] - node->pos[dir];
    if (dummy <= 0) {
        nearer_subtree = node->left;
        farther_subtree = node->right;
        nearer_hyperrect_coord = rect->max + dir;
        farther_hyperrect_coord = rect->min + dir;
    } else {
        nearer_subtree = node->right;
        farther_subtree = node->left;
        nearer_hyperrect_coord = rect->min + dir;
        farther_hyperrect_coord = rect->max + dir;
    }

    if (nearer_subtree) {
        dummy = *nearer_hyperrect_coord;
        *nearer_hyperrect_coord = node->pos[dir];
        kd_nearest_i(nearer_subtree, pos, result, result_dist_sq, rect);
        *nearer_hyperrect_coord = dummy;
    }

    dist_sq = 0;
    for (i = 0; i < 3; i++) {
        dist_sq += (node->pos[i] - pos[i]) * (node->pos[i] - pos[i]);
    }
    if (dist_sq < *result_dist_sq) {
        *result = node;
        *result_dist_sq = dist_sq;
    }

    if (farther_subtree) {
        dummy = *farther_hyperrect_coord;
        *farther_hyperrect_coord = node->pos[dir];

        if (hyperrect_dist_sq(rect, pos) < *result_dist_sq) {
            kd_nearest_i(farther_subtree, pos, result, result_dist_sq, rect);
        }
        *farther_hyperrect_coord = dummy;
    }
}

kdres* kd_nearest(struct kdtree* kd, const float* pos) {
    struct kdhyperrect* rect;
    struct kdnode* result;
    struct kdres* rset;
    float dist_sq;
    int i;

    if (!kd)
        return 0;
    if (!kd->rect)
        return 0;

    /* Allocate result set */
    if (!(rset = new kdres)) {
        return 0;
    }
    if (!(rset->rlist = new res_node)) {
        delete rset;
        return 0;
    }
    rset->rlist->next = 0;
    rset->tree = kd;

    /* Duplicate the bounding hyperrectangle, we will work on the copy */
    if (!(rect = hyperrect_duplicate(kd->rect))) {
        kd_res_free(rset);
        return 0;
    }

    result = kd->root;
    dist_sq = 0;
    for (i = 0; i < 3; i++)
        dist_sq += (result->pos[i] - pos[i]) * (result->pos[i] - pos[i]);

    kd_nearest_i(kd->root, pos, &result, &dist_sq, rect);

    hyperrect_free(rect);

    if (result) {
        if (rlist_insert(rset->rlist, result, -1.0) == -1) {
            kd_res_free(rset);
            return 0;
        }
        rset->size = 1;
        kd_res_rewind(rset);
        return rset;
    } else {
        kd_res_free(rset);
        return 0;
    }
}

kdres* kd_nearest_range(struct kdtree* kd, const float* pos, float range) {
    int ret;
    struct kdres* rset;

    if (!(rset = new kdres)) {
        return 0;
    }
    if (!(rset->rlist = new res_node)) {
        delete rset;
        return 0;
    }
    rset->rlist->next = 0;
    rset->tree = kd;

    if ((ret = find_nearest(kd->root, pos, range, rset->rlist, 0)) == -1) {
        kd_res_free(rset);
        return 0;
    }
    rset->size = ret;
    kd_res_rewind(rset);
    return rset;
}

void kd_res_free(struct kdres* rset) {
    clear_results(rset);
    delete rset->rlist;
    delete rset;
}

int kd_res_size(struct kdres* set) {
    return (set->size);
}

void kd_res_rewind(struct kdres* rset) {
    rset->riter = rset->rlist->next;
}

int kd_res_end(struct kdres* rset) {
    return rset->riter == 0;
}

int kd_res_next(struct kdres* rset) {
    rset->riter = rset->riter->next;
    return rset->riter != 0;
}

void* kd_res_item(struct kdres* rset, float* pos) {
    if (rset->riter) {
        if (pos) {
            memcpy(pos, rset->riter->item->pos, 3 * sizeof *pos);
        }
        return rset->riter->item->data;
    }
    return 0;
}

kdhyperrect* hyperrect_create(const float* min, const float* max) {
    size_t size = 3 * sizeof(float);
    struct kdhyperrect* rect = 0;

    if (!(rect = new kdhyperrect)) {
        return 0;
    }

    if (!(rect->min = new float[size])) {
        delete rect;
        return 0;
    }
    if (!(rect->max = new float[size])) {
        delete[] rect->min;
        delete rect;
        return 0;
    }
    memcpy(rect->min, min, size);
    memcpy(rect->max, max, size);

    return rect;
}

void hyperrect_free(struct kdhyperrect* rect) {
    delete[] rect->min;
    delete[] rect->max;
    delete rect;
}

kdhyperrect* hyperrect_duplicate(const struct kdhyperrect* rect) {
    return hyperrect_create(rect->min, rect->max);
}

void hyperrect_extend(struct kdhyperrect* rect, const float* pos) {
    int i;

    for (i = 0; i < 3; i++) {
        if (pos[i] < rect->min[i]) {
            rect->min[i] = pos[i];
        }
        if (pos[i] > rect->max[i]) {
            rect->max[i] = pos[i];
        }
    }
}

float hyperrect_dist_sq(struct kdhyperrect* rect, const float* pos) {
    int i;
    float result = 0;

    for (i = 0; i < 3; i++) {
        if (pos[i] < rect->min[i]) {
            result += (rect->min[i] - pos[i]) * (rect->min[i] - pos[i]);
        } else if (pos[i] > rect->max[i]) {
            result += (rect->max[i] - pos[i]) * (rect->max[i] - pos[i]);
        }
    }

    return result;
}

/* TODO make the ordering code use heapsort (Han 5/18/2021)*/
int rlist_insert(struct res_node* list, struct kdnode* item, float dist_sq) {
    struct res_node* rnode;

    if (!(rnode = new res_node)) {
        return -1;
    }
    rnode->item = item;
    rnode->dist_sq = dist_sq;

    if (dist_sq >= 0.0) {
        while (list->next && list->next->dist_sq < dist_sq) {
            list = list->next;
        }
    }
    rnode->next = list->next;
    list->next = rnode;
    return 0;
}

void clear_results(struct kdres* rset) {
    struct res_node *tmp, *node = rset->rlist->next;

    while (node) {
        tmp = node;
        node = node->next;
        delete tmp;
    }

    rset->rlist->next = 0;
}