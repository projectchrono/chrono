#include<stdio.h>
#include <stdlib.h>

#pragma once

#define NOT_IMPLEMENTED_YET                                                    \
  {                                                                            \
    printf(__func__);                                                          \
    printf(": FUNCTION NOT IMPLEMENTED YET. EXITTING.\n");                     \
    exit(1);                                                                   \
  }

#define GRANULAR_ERROR(msg)                     \
    {                                           \
        printf(msg);                            \
        printf(__func__);                       \
        printf("\n: EXITTING GRANULAR SIM.\n"); \
        exit(1);                                \
    }
