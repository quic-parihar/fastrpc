// Copyright (c) 2024, Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef VERIFY_PRINT_ERROR
#define VERIFY_PRINT_ERROR
#endif
#define FARF_ERROR 1
#define FARF_HIGH 0
#define FARF_MEDIUM 0

#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>


#include "dspsignal.h"
#include "fastrpc_common.h"
#include "fastrpc_internal.h"
#include "remote.h"
#include "verify.h"
#include "AEEStdErr.h"
#include "HAP_farf.h"

struct dspsignal_domain_signals {
  int dev;
  /* Adds "int domain;" (hash key) + "UT_hash_handle hh;". */
  ADD_DOMAIN_HASH();
};

DECLARE_HASH_TABLE(dspsignal, struct dspsignal_domain_signals)

struct dspsignal_process_signals {
  pthread_mutex_t mutex;
};

static struct dspsignal_process_signals *signals;
static pthread_once_t signals_once = PTHREAD_ONCE_INIT;

// Initialize process static signal structure. This should realistically never
// fail.
static void init_process_signals_once(void) {

  signals = calloc(1, sizeof(*signals));
  if (signals == NULL) {
    FARF(ERROR, "Out of memory");
    return;
  }
  if (pthread_mutex_init(&signals->mutex, NULL) != 0) {
    FARF(ERROR, "Mutex init failed");
    free(signals);
    signals = NULL;
    return;
  }
  HASH_TABLE_INIT(struct dspsignal_domain_signals);
}

void deinit_process_signals() {
  if (signals) {
    HASH_TABLE_CLEANUP(struct dspsignal_domain_signals);
    pthread_mutex_destroy(&signals->mutex);
    free(signals);
    signals = NULL;
  }
}

/* Lookup-only accessor: NULL if "signals" was never allocated or this
 * domain has no signal bookkeeping yet. */
static struct dspsignal_domain_signals *lookup_domain_signals(int domain) {
  struct dspsignal_domain_signals *ds = NULL;

  if (!signals)
    return NULL;
  GET_HASH_NODE(struct dspsignal_domain_signals, domain, ds);
  return ds;
}

// Dynamically initialize process signals structure.
static AEEResult init_domain_signals(int domain) {

  AEEResult nErr = AEE_SUCCESS;
  struct dspsignal_domain_signals *ds = NULL;

  VERIFY(IS_VALID_EFFECTIVE_DOMAIN_ID(domain));

  // Initialize process-level structure
  if ((pthread_once(&signals_once, init_process_signals_once) != 0) ||
      (signals == NULL)) {
    FARF(ERROR, "dspsignal init failed");
    return AEE_ERPC;
  }

  pthread_mutex_lock(&signals->mutex);
  GET_HASH_NODE(struct dspsignal_domain_signals, domain, ds);
  if (ds != NULL) {
    // Already initialized
    goto bail;
  }

  VERIFYC((ds = calloc(1, sizeof(*ds))) != NULL, AEE_ENOMEMORY);
  ds->domain = domain;

  VERIFY(AEE_SUCCESS == (nErr = fastrpc_session_dev(domain, &ds->dev)));
  VERIFYC(-1 != ds->dev, AEE_ERPC);

  /* Publish only once fully populated; signals->mutex serializes writers. */
  HASH_ADD_INT(info.tbl, domain, ds);

bail:
  pthread_mutex_unlock(&signals->mutex);
  if (nErr != AEE_SUCCESS) {
    /* The entry was never published, so nobody else can reference it. */
    free(ds);
    FARF(ERROR, "Error 0x%x: %s failed (domain %d) errno %s", nErr, __func__,
         domain, strerror(errno));
  }
  return nErr;
}

static int get_domain(int domain) {
  if (domain == -1) {
    domain = get_current_domain();
  }
  return domain;
}

void dspsignal_domain_deinit(int domain) {
  AEEResult nErr = AEE_SUCCESS;
  struct dspsignal_domain_signals *ds = NULL;

  if (!signals)
    return;
  VERIFYC(IS_VALID_EFFECTIVE_DOMAIN_ID(domain), AEE_EBADPARM);
  pthread_mutex_lock(&signals->mutex);
  GET_HASH_NODE(struct dspsignal_domain_signals, domain, ds);
  if (ds) {
    HASH_DEL(info.tbl, ds);
    free(ds);
  }
  pthread_mutex_unlock(&signals->mutex);
  FARF(ALWAYS, "%s done for domain %d", __func__, domain);

bail:
  if (nErr != AEE_SUCCESS) {
    FARF(ERROR, "Error 0x%x: %s failed (domain %d)", nErr, __func__, domain);
  }
  return;
}

AEEResult dspsignal_create(int domain, uint32_t id, uint32_t flags) {

  AEEResult nErr = AEE_SUCCESS;
  struct dspsignal_domain_signals *ds = NULL;

  VERIFYC(id < DSPSIGNAL_NUM_SIGNALS, AEE_EBADPARM);
  domain = get_domain(domain);
  VERIFYC(flags == 0, AEE_EBADPARM);
  VERIFYC(IS_VALID_EFFECTIVE_DOMAIN_ID(domain), AEE_EBADPARM);
  VERIFY((nErr = init_domain_signals(domain)) == 0);
  VERIFYC((ds = lookup_domain_signals(domain)) != NULL, AEE_EBADSTATE);
  errno = 0;
  nErr = ioctl_signal_create(ds->dev, id, flags);
  if (nErr) {
    if (errno == ENOTTY) {
      FARF(HIGH, "dspsignal support not present in the FastRPC driver");
      nErr = AEE_EUNSUPPORTED;
    } else {
      nErr = convert_kernel_to_user_error(nErr, errno);
    }
    goto bail;
  }
  FARF(HIGH, "%s: Signal %u created", __func__, id);

bail:
  if (nErr != AEE_SUCCESS) {
    FARF(ERROR, "Error 0x%x: %s failed (domain %d, ID %u, flags 0x%x) errno %s",
         nErr, __func__, domain, id, flags, strerror(errno));
  }
  return nErr;
}

AEEResult dspsignal_destroy(int domain, uint32_t id) {

  AEEResult nErr = AEE_SUCCESS;
  struct dspsignal_domain_signals *ds = NULL;

  VERIFYC(id < DSPSIGNAL_NUM_SIGNALS, AEE_EBADPARM);
  domain = get_domain(domain);
  VERIFYC(IS_VALID_EFFECTIVE_DOMAIN_ID(domain), AEE_EBADPARM);
  VERIFYC((ds = lookup_domain_signals(domain)) != NULL, AEE_EBADSTATE);
  errno = 0;
  nErr = ioctl_signal_destroy(ds->dev, id);
  if (nErr) {
    nErr = convert_kernel_to_user_error(nErr, errno);
    goto bail;
  }
  FARF(HIGH, "%s: Signal %u destroyed", __func__, id);

bail:
  if (nErr != AEE_SUCCESS) {
    FARF(ERROR, "Error 0x%x: %s failed (domain %d, ID %u) errno %s", nErr,
         __func__, domain, id, strerror(errno));
  }
  return nErr;
}

AEEResult dspsignal_signal(int domain, uint32_t id) {

  AEEResult nErr = AEE_SUCCESS;
  struct dspsignal_domain_signals *ds = NULL;

  VERIFYC(id < DSPSIGNAL_NUM_SIGNALS, AEE_EBADPARM);
  domain = get_domain(domain);
  VERIFYC(IS_VALID_EFFECTIVE_DOMAIN_ID(domain), AEE_EBADPARM);
  VERIFYC((ds = lookup_domain_signals(domain)) != NULL, AEE_EBADSTATE);

  FARF(MEDIUM, "%s: Send signal %u", __func__, id);
  errno = 0;
  nErr = ioctl_signal_signal(ds->dev, id);
  if (nErr) {
    nErr = convert_kernel_to_user_error(nErr, errno);
    goto bail;
  }

bail:
  if (nErr != AEE_SUCCESS) {
    FARF(ERROR, "Error 0x%x: %s failed (domain %d, ID %u) errno %s", nErr,
         __func__, domain, id, strerror(errno));
  }
  return nErr;
}

AEEResult dspsignal_wait(int domain, uint32_t id, uint32_t timeout_usec) {

  AEEResult nErr = AEE_SUCCESS;
  struct dspsignal_domain_signals *ds = NULL;

  VERIFYC(id < DSPSIGNAL_NUM_SIGNALS, AEE_EBADPARM);
  domain = get_domain(domain);
  VERIFYC(IS_VALID_EFFECTIVE_DOMAIN_ID(domain), AEE_EBADPARM);
  VERIFYC((ds = lookup_domain_signals(domain)) != NULL, AEE_EBADSTATE);

  FARF(MEDIUM, "%s: Wait signal %u timeout %u", __func__, id, timeout_usec);
  errno = 0;
  nErr = ioctl_signal_wait(ds->dev, id, timeout_usec);
  if (nErr) {
    if (errno == ETIMEDOUT) {
      FARF(MEDIUM, "%s: Signal %u timed out", __func__, id);
      return AEE_EEXPIRED;
    } else if (errno == EINTR) {
      FARF(MEDIUM, "%s: Signal %u canceled", __func__, id);
      return AEE_EINTERRUPTED;
    } else {
      nErr = convert_kernel_to_user_error(nErr, errno);
      goto bail;
    }
  }

bail:
  if (nErr != AEE_SUCCESS) {
    FARF(ERROR, "Error 0x%x: %s failed (domain %d, ID %u, timeout %u) errno %s",
         nErr, __func__, domain, id, timeout_usec, strerror(errno));
  }
  return nErr;
}

AEEResult dspsignal_cancel_wait(int domain, uint32_t id) {

  AEEResult nErr = AEE_SUCCESS;
  struct dspsignal_domain_signals *ds = NULL;

  VERIFYC(id < DSPSIGNAL_NUM_SIGNALS, AEE_EBADPARM);
  domain = get_domain(domain);
  VERIFYC(IS_VALID_EFFECTIVE_DOMAIN_ID(domain), AEE_EBADPARM);
  VERIFYC((ds = lookup_domain_signals(domain)) != NULL, AEE_EBADSTATE);

  FARF(MEDIUM, "%s: Cancel wait signal %u", __func__, id);
  errno = 0;
  nErr = ioctl_signal_cancel_wait(ds->dev, id);
  if (nErr) {
    nErr = convert_kernel_to_user_error(nErr, errno);
    goto bail;
  }

bail:
  if (nErr != AEE_SUCCESS) {
    FARF(ERROR, "Error 0x%x: %s failed (domain %d, ID %u) errno %s", nErr,
         __func__, domain, id, strerror(errno));
  }
  return nErr;
}
