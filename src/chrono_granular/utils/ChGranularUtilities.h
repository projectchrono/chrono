#include<stdio.h>
#include <stdlib.h>

#pragma once

#define NOT_IMPLEMENTED_YET                                                    \
  {                                                                            \
    printf(__func__);                                                          \
    printf(": FUNCTION NOT IMPLEMENTED YET. EXITTING.\n");                     \
    exit(1);                                                                   \
  }
