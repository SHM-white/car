#pragma once

#if __has_include("config_local.h")
#include "config_local.h"
#else
#warning "当前使用示例网络配置；实机必须提供 config_local.h"
#include "config_local.example.h"
#endif
