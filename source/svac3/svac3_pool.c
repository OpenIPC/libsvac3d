/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 */
#include "svac3_pool.h"

#include <pthread.h>
#include <stdlib.h>

struct svac3_pool {
    pthread_mutex_t lock;
    pthread_cond_t work, done;
    pthread_t *thread;
    int threads;                 /* workers + the caller */
    int quit;

    /* the job being run */
    svac3_pool_fn fn;
    void *arg;
    int n, next, finished;
    unsigned generation;
};

/* Take and run items of the current job until none are left */
static void drain(svac3_pool_t *p)
{
    for (;;) {
        if (p->next >= p->n)
            return;
        const int i = p->next++;
        svac3_pool_fn fn = p->fn;
        void *arg = p->arg;
        pthread_mutex_unlock(&p->lock);
        fn(arg, i);
        pthread_mutex_lock(&p->lock);
        if (++p->finished == p->n)
            pthread_cond_broadcast(&p->done);
    }
}

static void *worker(void *opaque)
{
    svac3_pool_t *p = opaque;
    unsigned seen = 0;
    pthread_mutex_lock(&p->lock);
    for (;;) {
        while (!p->quit && p->generation == seen)
            pthread_cond_wait(&p->work, &p->lock);
        if (p->quit)
            break;
        seen = p->generation;
        drain(p);
    }
    pthread_mutex_unlock(&p->lock);
    return NULL;
}

svac3_pool_t *svac3_pool_create(int threads)
{
    if (threads < 2)
        return NULL;
    if (threads > SVAC3_MAX_THREADS)
        threads = SVAC3_MAX_THREADS;
    svac3_pool_t *p = calloc(1, sizeof(*p));
    if (!p)
        return NULL;
    p->thread = calloc((size_t)threads - 1, sizeof(*p->thread));
    if (!p->thread || pthread_mutex_init(&p->lock, NULL)) {
        free(p->thread);
        free(p);
        return NULL;
    }
    pthread_cond_init(&p->work, NULL);
    pthread_cond_init(&p->done, NULL);
    p->threads = 1;
    for (int i = 0; i < threads - 1; i++) {
        if (pthread_create(&p->thread[i], NULL, worker, p))
            break;
        p->threads++;
    }
    if (p->threads < 2) {
        svac3_pool_destroy(p);
        return NULL;
    }
    return p;
}

void svac3_pool_destroy(svac3_pool_t *p)
{
    if (!p)
        return;
    pthread_mutex_lock(&p->lock);
    p->quit = 1;
    pthread_cond_broadcast(&p->work);
    pthread_mutex_unlock(&p->lock);
    for (int i = 0; i < p->threads - 1; i++)
        pthread_join(p->thread[i], NULL);
    pthread_cond_destroy(&p->work);
    pthread_cond_destroy(&p->done);
    pthread_mutex_destroy(&p->lock);
    free(p->thread);
    free(p);
}

int svac3_pool_threads(const svac3_pool_t *p)
{
    return p ? p->threads : 1;
}

void svac3_pool_run(svac3_pool_t *p, svac3_pool_fn fn, void *arg, int n)
{
    if (!p || n < 2) {
        for (int i = 0; i < n; i++)
            fn(arg, i);
        return;
    }
    pthread_mutex_lock(&p->lock);
    p->fn = fn;
    p->arg = arg;
    p->n = n;
    p->next = 0;
    p->finished = 0;
    p->generation++;
    pthread_cond_broadcast(&p->work);
    drain(p);                                   /* the caller works too */
    while (p->finished < p->n)
        pthread_cond_wait(&p->done, &p->lock);
    pthread_mutex_unlock(&p->lock);
}
