/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * This application is used for converting the ORBvoc file to different format(xml/txt/binary). // by JadeCong
*/

#include "ORBVocabulary.h"
#include <time.h>
#include <string.h>


using namespace std;

void load_from_xml(ORB_SLAM3::ORBVocabulary* voc, const std::string infile)
{
    clock_t tStart = clock();
    voc->load(infile);
    printf("Loading from xml: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void load_from_binary(ORB_SLAM3::ORBVocabulary* voc, const std::string infile)
{
    clock_t tStart = clock();
    voc->loadFromBinaryFile(infile);
    printf("Loading from binary: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

bool load_from_text(ORB_SLAM3::ORBVocabulary* voc, const std::string infile)
{
    clock_t tStart = clock();
    bool res = voc->loadFromTextFile(infile);
    printf("Loading from text: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_xml(ORB_SLAM3::ORBVocabulary* voc, const std::string outfile)
{
    clock_t tStart = clock();
    voc->save(outfile);
    printf("Saving as xml: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_text(ORB_SLAM3::ORBVocabulary* voc, const std::string outfile)
{
    clock_t tStart = clock();
    voc->saveToTextFile(outfile);
    printf("Saving as text: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_binary(ORB_SLAM3::ORBVocabulary* voc, const std::string outfile)
{
    clock_t tStart = clock();
    voc->saveToBinaryFile(outfile);
    printf("Saving as binary: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}


int main(int argc, char **argv)
{
    cout << "VocabularyConverter using BoW load/save benchmark..." << endl;
    string infile, outfile;
    ORB_SLAM3::ORBVocabulary* voc = new ORB_SLAM3::ORBVocabulary();
    
    // loading file
    cout << "Please input the loading file(xml/txt/bin) path: ./Vocabulary/ORBvoc.txt(e.g.)" << endl;
    cin >> infile;
    if(infile.substr(infile.length()-3, 3) == "xml")
    {
        load_from_xml(voc, infile);
    }
    else if(infile.substr(infile.length()-3, 3) == "txt")
    {
        load_from_text(voc, infile);
    }
    else if(infile.substr(infile.length()-3, 3) == "bin")
    {
        load_from_binary(voc, infile);
    }
    else
    {
        cout << "Wrong file path or format, please try to run the application again！" << endl;
        return 0;
    }
    
    // saving file
    cout << "Please input the saving file(xml/txt/bin) path: ./Vocabulary/ORBvoc.bin(e.g.)" << endl;
    cin >> outfile;
    if(infile.substr(infile.length()-3, 3) == outfile.substr(outfile.length()-3, 3)) // check whether outfile format is same with infile format
    {
        cout << "Same format for loading and saving file, please try to text different saving file format again！" << endl;
        cin >> outfile;
    }
    if(outfile.substr(outfile.length()-3, 3) == "xml")
    {
        save_as_xml(voc, outfile);
    }
    else if(outfile.substr(outfile.length()-3, 3) == "txt")
    {
        save_as_text(voc, outfile);
    }
    else if(outfile.substr(outfile.length()-3, 3) == "bin")
    {
        save_as_binary(voc, outfile);
    }
    else
    {
        cout << "Wrong file path or format, please try to run the application again！" << endl;
        return 0;
    }
    
    // finishing converting
    cout << "Finished converting " << infile << " to " << outfile << endl;
    
    return 0;
}
