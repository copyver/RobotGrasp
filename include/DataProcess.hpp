#pragma once

#include <iostream>
#include <string>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cwchar>
#include <vector>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <sstream>
#include <iomanip>

namespace fs = std::filesystem;

namespace DataUtils {
    inline bool folderExists(const std::string &path) {
        struct stat info{};
        if (stat(path.c_str(), &info) != 0)
            return false;
        return (info.st_mode & S_IFDIR) != 0;
    }

    inline bool createFolder(const std::string &path) {
        #ifdef _WIN32
                int status = _mkdir(path.c_str());
        #else
                int status = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        #endif
                return status == 0;
    }

    inline std::string GetCurrentExecutableFolder() {
        char path[PATH_MAX];
        ssize_t count = readlink("/proc/self/exe", path, PATH_MAX);
        if (count == -1) {
            throw std::runtime_error("Error reading symbolic link /proc/self/exe");
        }
        std::string exePath(path, count);
        std::string::size_type pos = exePath.find_last_of("/");

        return exePath.substr(0, pos);
    }

    inline std::string GetParentFolder() {
        char path[PATH_MAX];
        ssize_t count = readlink("/proc/self/exe", path, PATH_MAX);
        if (count == -1) {
            throw std::runtime_error("Error reading symbolic link /proc/self/exe");
        }

        // 获取当前可执行文件所在目录
        std::string exePath(path, count);
        std::string::size_type pos = exePath.find_last_of("/");
        if (pos == std::string::npos) {
            throw std::runtime_error("Unable to determine executable folder");
        }

        // 截取上级目录
        std::string currentFolder = exePath.substr(0, pos);
        pos = currentFolder.find_last_of("/");
        if (pos == std::string::npos) {
            throw std::runtime_error("Unable to determine parent folder");
        }

        return currentFolder.substr(0, pos);
    }

    inline bool clearFolder(const std::string &folderPath) {
        DIR *dir = opendir(folderPath.c_str());
        if (dir == nullptr) {
            return false;  // 无法打开文件夹
        }

        struct dirent *entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string fileName = entry->d_name;
            if (fileName != "." && fileName != "..") {
                std::string filePath = folderPath;
                filePath += "/";
                filePath += fileName;
                struct stat fileInfo{};
                if (stat(filePath.c_str(), &fileInfo) == 0) {
                    if (S_ISDIR(fileInfo.st_mode)) {
                        // 如果是文件夹，可以递归删除文件夹及其内容
                        clearFolder(filePath);
                        rmdir(filePath.c_str());
                    } else {
                        // 如果是文件，删除文件
                        remove(filePath.c_str());
                    }
                }
            }
        }
        closedir(dir);
        return true;
    }

    inline std::wstring stringToWString(const std::string &str) {
        std::mbstate_t state = std::mbstate_t();
        const char *src = str.data();
        size_t len = std::mbsrtowcs(nullptr, &src, 0, &state) + 1; // +1 for null terminator
        std::vector<wchar_t> buffer(len);
        std::mbsrtowcs(buffer.data(), &src, buffer.size(), &state);
        return {buffer.data()};
    }

    inline void saveImageWithIndex(const std::string &folderPath, const cv::Mat &image) {
        if (!fs::exists(folderPath)) {
            fs::create_directories(folderPath);
        }

        // 统计文件夹中已有的 PNG 图片数量
        int imageCount = 0;
        for (const auto &entry: fs::directory_iterator(folderPath)) {
            if (entry.path().extension() == ".png") {
                imageCount++;
            }
        }

        // 生成新的文件名，格式为 image_000001.png
        std::ostringstream filenameStream;
        filenameStream << "image_" << std::setw(6) << std::setfill('0') << imageCount << ".png";
        std::string newImageName = filenameStream.str();
        std::string newImagePath = (fs::path(folderPath) / newImageName).string();

        if (cv::imwrite(newImagePath, image)) {
            std::cout << "图片已保存为: " << newImagePath << std::endl;
        } else {
            std::cerr << "保存图片失败！" << std::endl;
        }
    }
}
