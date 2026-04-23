/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @log
 */

#pragma once

#include <glog/logging.h>
#include <iostream>

#define LINFO LOG(INFO)
#define LWARN LOG(WARNING)
#define LERROR LOG(ERROR)
#define LFATAL LOG(FATAL)

#define LINFO_IF(cond) LOG_IF(INFO, cond)
#define LWARN_IF(cond) LOG_IF(WARNING, cond)
#define LERROR_IF(cond) LOG_IF(ERROR, cond)

#define LCHECK(cond) CHECK(cond)

#define LINFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
#define LWARN_EVERY(freq) LOG_EVERY_N(WARNING, freq)
#define LERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

#define RETURN_IF_NULL(ptr)          \
  if (ptr == nullptr) {              \
    LWARN << #ptr << " is nullptr."; \
    return;                          \
  }

#define RETURN_VAL_IF_NULL(ptr, val) \
  if (ptr == nullptr) {              \
    LWARN << #ptr << " is nullptr."; \
    return val;                      \
  }

#define RETURN_IF(condition)               \
  if (condition) {                         \
    LWARN << #condition << " is not met."; \
    return;                                \
  }

#define RETURN_VAL_IF(condition, val)      \
  if (condition) {                         \
    LWARN << #condition << " is not met."; \
    return val;                            \
  }
