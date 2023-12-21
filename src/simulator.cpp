#include <iostream>
#include <fstream>
#include <string>

#include <json/json.h>

#include "simulator_app.hpp"


int main(int argc, char* argv[])
{
    std::string scenesFilePath;
    std::string sceneName;
    {    
        if(argc != 2 && argc != 3)
        {
            std::cout << "sim: simulate a scene." << std::endl
                    << "Usage: ./sim path_to_scenes_file scene_name" << std::endl
                    << "\t path_to_scenes_file: path to the json describing at least one scene" << std::endl
                    << "\t scene_name: name of the scene to simulate. Defaults to the first one alphabetically" << std::endl;

            return EXIT_FAILURE;
        }

        scenesFilePath = std::string(argv[1]);
        sceneName = (argc == 3)? std::string(argv[2]) : "";
    }
    
    Json::Value sceneRoot;
    {
        Json::Value scenesJson;
        std::ifstream scenesFile(scenesFilePath);
        if(!scenesFile.is_open())
        {
            std::cerr << "Cannot read scenes file \"" << scenesFilePath << "\"" << std::endl;
            return EXIT_FAILURE;
        }

        try
        {
            scenesFile >> scenesJson;
        }
        catch(const Json::Exception& e)
        {
            std::cerr << "Exception parsing scenes file: " << e.what() << std::endl;
            return EXIT_FAILURE;
        }

        if(scenesJson.isNull() || !scenesJson.isObject() || scenesJson.size() == 0)
        {
            std::cerr << "No scenes found" << std::endl;
            std::cout << scenesJson.isNull() << scenesJson.isObject() << scenesJson.size() << std::endl;
            return EXIT_FAILURE;
        }

        if(sceneName.empty())
        {
            sceneName = scenesJson.getMemberNames()[0];
            sceneRoot = scenesJson[sceneName];
        }
        else
        {
            sceneRoot = scenesJson[sceneName];
            if(sceneRoot.isNull())
            {
                std::cerr << "No scene named \"" << sceneName << "\"" << std::endl;
                return EXIT_FAILURE;
            }
        }
    }
    
    SimulatorApp simApp(sceneName, sceneRoot);
    simApp.run();

    return EXIT_SUCCESS;
}