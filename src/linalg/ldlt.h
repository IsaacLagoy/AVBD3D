#ifndef LDLT_H
#define LDLT_H

#include "debug_utils/print.h"
#include "mat6x6.h"

vec6 solve(const mat6x6& lhs, const vec6& rhs);

#endif