#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
using namespace std;

int main(void)
{

    fstream file;
    fstream data_gyro;
    fstream data_yaw;
    fstream time;

    file.open("input1010.txt");
    data_gyro.open("data_gyro1010.txt");
    data_yaw.open("data_yaw1010.txt");
    time.open("time1010.txt");

    if (file.is_open() && data_gyro.is_open() && data_yaw.is_open() && time.is_open())
    {
        string line;
        while (getline(file, line))
        {
            time << line.substr(0, 12) << endl;
            data_gyro << line.substr(13, 20) << endl;
            data_yaw << line.substr(20, line.length() - 1) << endl;
        }
        cout << "Done!" << endl;
        file.close();
        data_gyro.close();
        data_yaw.close();
        time.close();
    }
}