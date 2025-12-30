//
// Created by Canjia Huang <canjia7@gmail.com> on 2025/11/28.
//

#ifndef POLYGEN_THROW_ERROR_H
#define POLYGEN_THROW_ERROR_H

#include <sstream>
#include <stdexcept>
#include <string>
#include "utils/log.h"

/* The rationale for using macro definition is to enable the output of the file name and line number where an error
 * occurred when logging the error */
#define THROW_RUNTIME_ERROR(msg) \
do { \
LOG::ERROR("{}", msg); \
throw std::runtime_error(msg); \
} while(0)

#endif //POLYGEN_THROW_ERROR_H