#ifndef TEST_TKERNEL_H
#define TEST_TKERNEL_H

#include <stdint.h>

typedef int32_t ID;
typedef int32_t TMO;
typedef uint32_t UINT;
typedef struct {
    uint32_t mtxatr;
} T_CMTX;

#define TA_TFIFO 1U
#define TA_INHERIT 2U
#define TMO_FEVR (-1)
#define TMO_POL 0
#define E_OK 0

ID tk_cre_mtx(const T_CMTX *config);
int32_t tk_loc_mtx(ID mutex, TMO timeout);
int32_t tk_unl_mtx(ID mutex);
int32_t tk_set_flg(ID flag, UINT pattern);

#endif
