#include <iostream>
#include <fstream>
#include <vector>
#include <string>
using std::cout;
using std::vector;
using std::ifstream;
using std::string;

void ReadFile(string path) {
    ifstream myfile (path);
    // vector<vector<int>> pad = {{}};
    if (myfile){
        string line;
        while (getline(myfile, line)){
            cout << line << "\n";
        }
    }
}

/* void PrintBoardFile(const vector<vector<int>> board){
    for (auto i:board){
        for (int j:i){
            cout << j;
        }
        cout <<"\n";
    }
} */

int main(){
    ReadFile("/C++ study/1.Board");
    // PrintBoardFile()
}