#ifndef _KDTREE_H_
#define _KDTREE_H_

struct vec3f{
    float data[3];
    float operator[](int idx) const {return data[idx];}
};
struct kdhyperrect {
    float *min, *max;
};
struct kdnode {
    float *pos;
    int dir;
    void *data;

    struct kdnode *left, *right;
};
struct res_node {
    struct kdnode *item;
    float dist_sq;
    struct res_node *next;
};
struct kdtree {
    struct kdnode *root;
    struct kdhyperrect *rect;
    void (*destr)(void*);
};
struct kdres {
    struct kdtree *tree;
    struct res_node *rlist, *riter;
    int size;
};
kdtree *kd_create();
void kd_free(struct kdtree *tree);
void kd_clear(struct kdtree *tree);
void kd_data_destructor(struct kdtree *tree, void (*destr)(void*));
int kd_insert(struct kdtree *tree, const float *pos, void *data);
kdres *kd_nearest_range(struct kdtree *kd, const float *pos, float range);
kdres *kd_nearest(struct kdtree *tree, const float *pos);
void kd_res_free(struct kdres *set);
int kd_res_size(struct kdres *set);
void kd_res_rewind(struct kdres *set);
int kd_res_end(struct kdres *set);
int kd_res_next(struct kdres *set);
void *kd_res_item(struct kdres *set, float *pos);



#endif  /* _KDTREE_H_ */