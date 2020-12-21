//
// Created by dongxiao on 2020/12/13.
//
#include "mylocalization/tools/file_manager.h"
namespace mylocalization{
    bool FileManager::CreatDirectory(std::string director_path) {
        if(!boost::filesystem::is_directory(director_path))
            boost::filesystem::create_directory(director_path);
        if(!boost::filesystem::is_directory(director_path)){
            LOG(WARNING) << "Can't Creat Directory!"<<director_path;
            return false;
        }
        return true;
    }
    bool FileManager::CreatFile(std::ofstream &ofs, std::string file_path) {
        ofs.open(file_path.c_str(),std::ios::app);
        if(!ofs){
            LOG(WARNING) << "Can't Creat file!" << file_path;
            return false;
        }
        return true;
    }
}
