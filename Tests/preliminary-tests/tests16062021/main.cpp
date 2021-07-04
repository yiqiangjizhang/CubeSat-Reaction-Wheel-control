#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
using namespace std;

int main(void) {

    fstream file;
    fstream data;
    fstream time;
    file.open("input04.txt");
    data.open("data04.txt");
    time.open("time04.txt");

    if(file.is_open() && data.is_open() && time.is_open()) {
        string line;
    	while (getline(file, line)) {
            time << line.substr(0,12) << endl;
            data << line.substr(19,line.length()-1) << endl;
        }
    }

    file.close();
    data.close();
    time.close();
}
