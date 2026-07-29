#pragma once

#if __has_include("config_local.h")
#include "config_local.h"
#else
#warning "当前使用示例配置；上车前必须复制并校准 config_local.h"
#include "config_local.example.h"
#endif

