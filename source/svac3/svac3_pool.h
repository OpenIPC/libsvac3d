/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * A small fixed thread pool: svac3_pool_run() runs fn(arg, i) for i in
 * [0, n) across the pool's threads and the caller's, and returns when all
 * are done. A NULL pool runs them on the caller, in order.
 */
#ifndef SVAC3_POOL_H
#define SVAC3_POOL_H

typedef struct svac3_pool svac3_pool_t;
typedef void (*svac3_pool_fn)(void *arg, int i);

/* threads in all, the caller's included; NULL when threads < 2 or on
 * failure (callers then run serially) */
svac3_pool_t *svac3_pool_create(int threads);
void svac3_pool_destroy(svac3_pool_t *pool);
int svac3_pool_threads(const svac3_pool_t *pool);

void svac3_pool_run(svac3_pool_t *pool, svac3_pool_fn fn, void *arg, int n);

#endif /* SVAC3_POOL_H */
