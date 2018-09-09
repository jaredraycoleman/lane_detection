#include "helpers.h"

/**
 * Removes the whitespace in a path
 * @param path path string to remove whitespace from
 * @returns path without whitespace
 */
std::string rem_whitespace(std::string path) {
    path.erase(std::remove_if(path.begin(), path.end(), ::isspace), path.end());
    return path;
}

/**
 * Gets the directory of a path
 * @param path Path to get directoy of
 * @returns directory of path
 */
std::string get_dir(std::string path) {
    path = rem_whitespace(path);

    std::string dir(".");
    if (path.find('/') != std::string::npos) {
        dir = path.substr(0, path.find_last_of('/'));
    }
    return dir;
}

/**
 * Gets the absolute path relative to another path
 * @param path path to convert to absolute
 * @param relative_to relative path
 * @returns absolute path
 */
std::string abs_path(std::string path, std::string relative_to) {
    path = rem_whitespace(path);
    relative_to = rem_whitespace(relative_to);

    if (path[0] != '/') {
        path = std::string(relative_to + '/' + path);
    }

    return path;
}