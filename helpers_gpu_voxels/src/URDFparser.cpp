// C++ stl
#include <iostream>
#include <string>

// boost
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <urdf/model.h>

// namespace and function declaration
using namespace std;
string getFullPath(string filename);

int main(int argc, char **argv)
{
    // init
    ros::init(argc,argv, "URDF_parser");
    ros::NodeHandle n("~"); //private -> for private parameters (urdf path)

    // urdf name, mesh filename
    string urdf_file;
    string filename;

    // count
    int meshNum = 0;
    int noMeshNum = 0;

    // urdf model, links and other link properties
    urdf::Model robot;
    vector<urdf::LinkSharedPtr> links;
    urdf::Link *link;
    urdf::MeshConstSharedPtr mesh;
    urdf::Vector3 linkScale;

    //get FULL urdf path
    if(!n.getParam("urdf", urdf_file)) 
    {
        cout << "Insert ONE URDF file!" << endl;
        return 8;
    } 

    // build tree from URDF
    if (!robot.initFile(urdf_file)) 
    {
        cout << "Couldn't load: " << urdf_file << endl;
        return 24;
    } 
    else 
    {
        cout << "URDF: " << urdf_file << " loaded properly!" << endl;
    }

    // get all robot links
    robot.getLinks(links);
    cout << "Robot " << robot.getName() << " has " << links.size() << " links:" << endl;
    cout << "-------------------------------------------------------------------------------------------------" <<
            "-------------------------------------------------------------------------------------------------" <<
            "-----------------------------------------" << endl;

    // get mesh path for all links
    for(auto i = links.cbegin(); i != links.cend(); i++)
    {
        // get link
        link = i->get();

        // check if all properties exists
        if (link->visual == nullptr) 
        {
            cout << link->name << ": visual dosen't exists\n" << endl;
            noMeshNum++;

        } 
        else if(link->visual->geometry == nullptr) 
        {
            cout << link->name << ": geometry dosen't exists\n" << endl;
            noMeshNum++;

        
        }// cast urdf::GeometrySharedPtr to const urdf::Mesh to get all mesh data (downcasting)
        else if ((mesh = boost::dynamic_pointer_cast<const urdf::Mesh>(link->visual->geometry)) == nullptr)
        {
            cout << link->name << ": mesh dosen't exists\n" << endl;
            noMeshNum++;
            
        } 
        else 
        { 
            // get mesh relative path (from package) and scale
            filename = boost::trim_copy(mesh->filename);
            linkScale = mesh->scale;

            // if mesh file is not in .stl, check collision mesh format
            if(boost::algorithm::to_lower_copy(boost::filesystem::extension(filename)) != ".stl")
                if(link->collision != nullptr) 
                    if(link->collision->geometry != nullptr)
                        if((mesh = boost::dynamic_pointer_cast<const urdf::Mesh>(link->collision->geometry)) != nullptr)
                            if(boost::algorithm::to_lower_copy(boost::filesystem::extension(boost::trim_copy(mesh->filename))) == ".stl") 
                            {
                                filename = boost::trim_copy(mesh->filename);
                            }

            // print relative, absolute path and scale of mesh
            cout << "Link name: " << link->name << endl;
            //cout << "   Mesh: " << filename << endl;
            cout << "Full mesh path: " << getFullPath(filename) << endl;
            cout << "Scale: " << linkScale.x << " " << linkScale.y << " " << linkScale.z << "\n" << endl;
            meshNum++;  
        }
        
    }
    cout << "Number of links with mesh file: " << meshNum << endl;
    cout << "Number of links without mesh file: " << noMeshNum;

    return 0;
}

string getFullPath(string filename) 
{
    // mesh package and path
    string package, sub = "package://", file = "file://";
    string path;

    if(filename.find(sub) != string::npos) 
    {
        // remove "package://"
        filename.erase(0, sub.size());
        //cout << filename << endl;

        // get package name and remove package name from filename
        boost::trim_left(filename);
        package = filename.substr(0, filename.find("/"));
        filename.erase(0, filename.find("/"));
        //cout<<"Package: "<<package<<endl;

        // find package path
        path = ros::package::getPath(package);

        if(path.empty()) 
        {
            return "Visual exist, but ROS package: " + package + " which contains meshes is not installed!";
        }
        // return full path
        return path + filename;

    } 
    else if(filename.find(file) != string::npos) 
    {
        // remove "file://" and return absolute path to mesh file
        filename.erase(0, file.size());
        return boost::trim_left_copy(filename);

    } 
    else
    {
        return filename;
    }
}