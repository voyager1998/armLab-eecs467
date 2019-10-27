// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "mbot_command_t.h"

static int __mbot_command_t_hash_computed;
static uint64_t __mbot_command_t_hash;

uint64_t __mbot_command_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __mbot_command_t_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = __mbot_command_t_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0x75344374abf3c6b9LL
         + __int64_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __pose_xyt_t_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
         + __pose_xyt_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __mbot_command_t_get_hash(void)
{
    if (!__mbot_command_t_hash_computed) {
        __mbot_command_t_hash = (int64_t)__mbot_command_t_hash_recursive(NULL);
        __mbot_command_t_hash_computed = 1;
    }

    return __mbot_command_t_hash;
}

int __mbot_command_t_encode_array(void *buf, int offset, int maxlen, const mbot_command_t *p, int elements)
{
    int pos = 0, element;
    int thislen;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].is_mode_spin), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].state), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __pose_xyt_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].goal_pose), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].num_obstacles), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __pose_xyt_t_encode_array(buf, offset + pos, maxlen - pos, p[element].obstacle_poses, p[element].num_obstacles);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int mbot_command_t_encode(void *buf, int offset, int maxlen, const mbot_command_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __mbot_command_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __mbot_command_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __mbot_command_t_encoded_array_size(const mbot_command_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __int64_t_encoded_array_size(&(p[element].utime), 1);

        size += __int8_t_encoded_array_size(&(p[element].is_mode_spin), 1);

        size += __int8_t_encoded_array_size(&(p[element].state), 1);

        size += __pose_xyt_t_encoded_array_size(&(p[element].goal_pose), 1);

        size += __int32_t_encoded_array_size(&(p[element].num_obstacles), 1);

        size += __pose_xyt_t_encoded_array_size(p[element].obstacle_poses, p[element].num_obstacles);

    }
    return size;
}

int mbot_command_t_encoded_size(const mbot_command_t *p)
{
    return 8 + __mbot_command_t_encoded_array_size(p, 1);
}

size_t mbot_command_t_struct_size(void)
{
    return sizeof(mbot_command_t);
}

int mbot_command_t_num_fields(void)
{
    return 6;
}

int mbot_command_t_get_field(const mbot_command_t *p, int i, lcm_field_t *f)
{
    if (0 > i || i >= mbot_command_t_num_fields())
        return 1;
    
    switch (i) {
    
        case 0: {
            f->name = "utime";
            f->type = LCM_FIELD_INT64_T;
            f->typestr = "int64_t";
            f->num_dim = 0;
            f->data = (void *) &p->utime;
            return 0;
        }
        
        case 1: {
            f->name = "is_mode_spin";
            f->type = LCM_FIELD_INT8_T;
            f->typestr = "int8_t";
            f->num_dim = 0;
            f->data = (void *) &p->is_mode_spin;
            return 0;
        }
        
        case 2: {
            f->name = "state";
            f->type = LCM_FIELD_INT8_T;
            f->typestr = "int8_t";
            f->num_dim = 0;
            f->data = (void *) &p->state;
            return 0;
        }
        
        case 3: {
            /* pose_xyt_t */
            f->name = "goal_pose";
            f->type = LCM_FIELD_USER_TYPE;
            f->typestr = "pose_xyt_t";
            f->num_dim = 0;
            f->data = (void *) &p->goal_pose;
            return 0;
        }
        
        case 4: {
            f->name = "num_obstacles";
            f->type = LCM_FIELD_INT32_T;
            f->typestr = "int32_t";
            f->num_dim = 0;
            f->data = (void *) &p->num_obstacles;
            return 0;
        }
        
        case 5: {
            /* pose_xyt_t */
            f->name = "obstacle_poses";
            f->type = LCM_FIELD_USER_TYPE;
            f->typestr = "pose_xyt_t";
            f->num_dim = 1;
            f->dim_size[0] = p->num_obstacles;
            f->dim_is_variable[0] = 1;
            f->data = (void *) &p->obstacle_poses;
            return 0;
        }
        
        default:
            return 1;
    }
}

const lcm_type_info_t *mbot_command_t_get_type_info(void)
{
    static int init = 0;
    static lcm_type_info_t typeinfo;
    if (!init) {
        typeinfo.encode         = (lcm_encode_t) mbot_command_t_encode;
        typeinfo.decode         = (lcm_decode_t) mbot_command_t_decode;
        typeinfo.decode_cleanup = (lcm_decode_cleanup_t) mbot_command_t_decode_cleanup;
        typeinfo.encoded_size   = (lcm_encoded_size_t) mbot_command_t_encoded_size;
        typeinfo.struct_size    = (lcm_struct_size_t)  mbot_command_t_struct_size;
        typeinfo.num_fields     = (lcm_num_fields_t) mbot_command_t_num_fields;
        typeinfo.get_field      = (lcm_get_field_t) mbot_command_t_get_field;
        typeinfo.get_hash       = (lcm_get_hash_t) __mbot_command_t_get_hash;
    }
    
    return &typeinfo;
}
int __mbot_command_t_decode_array(const void *buf, int offset, int maxlen, mbot_command_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].is_mode_spin), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].state), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __pose_xyt_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].goal_pose), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].num_obstacles), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        p[element].obstacle_poses = (pose_xyt_t*) lcm_malloc(sizeof(pose_xyt_t) * p[element].num_obstacles);
        thislen = __pose_xyt_t_decode_array(buf, offset + pos, maxlen - pos, p[element].obstacle_poses, p[element].num_obstacles);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __mbot_command_t_decode_array_cleanup(mbot_command_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_decode_array_cleanup(&(p[element].utime), 1);

        __int8_t_decode_array_cleanup(&(p[element].is_mode_spin), 1);

        __int8_t_decode_array_cleanup(&(p[element].state), 1);

        __pose_xyt_t_decode_array_cleanup(&(p[element].goal_pose), 1);

        __int32_t_decode_array_cleanup(&(p[element].num_obstacles), 1);

        __pose_xyt_t_decode_array_cleanup(p[element].obstacle_poses, p[element].num_obstacles);
        if (p[element].obstacle_poses) free(p[element].obstacle_poses);

    }
    return 0;
}

int mbot_command_t_decode(const void *buf, int offset, int maxlen, mbot_command_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __mbot_command_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __mbot_command_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int mbot_command_t_decode_cleanup(mbot_command_t *p)
{
    return __mbot_command_t_decode_array_cleanup(p, 1);
}

int __mbot_command_t_clone_array(const mbot_command_t *p, mbot_command_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_clone_array(&(p[element].utime), &(q[element].utime), 1);

        __int8_t_clone_array(&(p[element].is_mode_spin), &(q[element].is_mode_spin), 1);

        __int8_t_clone_array(&(p[element].state), &(q[element].state), 1);

        __pose_xyt_t_clone_array(&(p[element].goal_pose), &(q[element].goal_pose), 1);

        __int32_t_clone_array(&(p[element].num_obstacles), &(q[element].num_obstacles), 1);

        q[element].obstacle_poses = (pose_xyt_t*) lcm_malloc(sizeof(pose_xyt_t) * q[element].num_obstacles);
        __pose_xyt_t_clone_array(p[element].obstacle_poses, q[element].obstacle_poses, p[element].num_obstacles);

    }
    return 0;
}

mbot_command_t *mbot_command_t_copy(const mbot_command_t *p)
{
    mbot_command_t *q = (mbot_command_t*) malloc(sizeof(mbot_command_t));
    __mbot_command_t_clone_array(p, q, 1);
    return q;
}

void mbot_command_t_destroy(mbot_command_t *p)
{
    __mbot_command_t_decode_array_cleanup(p, 1);
    free(p);
}

int mbot_command_t_publish(lcm_t *lc, const char *channel, const mbot_command_t *p)
{
      int max_data_size = mbot_command_t_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = mbot_command_t_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _mbot_command_t_subscription_t {
    mbot_command_t_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void mbot_command_t_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    mbot_command_t p;
    memset(&p, 0, sizeof(mbot_command_t));
    status = mbot_command_t_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding mbot_command_t!!!\n", status);
        return;
    }

    mbot_command_t_subscription_t *h = (mbot_command_t_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    mbot_command_t_decode_cleanup (&p);
}

mbot_command_t_subscription_t* mbot_command_t_subscribe (lcm_t *lcm,
                    const char *channel,
                    mbot_command_t_handler_t f, void *userdata)
{
    mbot_command_t_subscription_t *n = (mbot_command_t_subscription_t*)
                       malloc(sizeof(mbot_command_t_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 mbot_command_t_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg mbot_command_t LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int mbot_command_t_subscription_set_queue_capacity (mbot_command_t_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int mbot_command_t_unsubscribe(lcm_t *lcm, mbot_command_t_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe mbot_command_t_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

