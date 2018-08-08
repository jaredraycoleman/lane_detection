#ifndef HELPERS_H
#define HELPERS_H

#include <string>
#include <algorithm>

std::string rem_whitespace(std::string path);

std::string get_dir(std::string path);

std::string abs_path(std::string path, std::string relative_to);
#endif