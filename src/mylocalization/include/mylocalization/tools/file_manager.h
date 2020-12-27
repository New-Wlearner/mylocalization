//
// Created by dongxiao on 2020/12/13.
//

#ifndef MYLOCALIZATION_FILE_MANAGER_H
#define MYLOCALIZATION_FILE_MANAGER_H
#include <cstring>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include "glog/logging.h"

namespace mylocalization{
    class FileManager{
    public:
        static bool CreatFile(std::ofstream& ofs,std::string file_path);
        static bool CreatDirectory(std::string director_path);
    };
}
#endif //MYLOCALIZATION_FILE_MANAGER_H
