#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
using namespace std;

int main(void)
{

    fstream file;
    fstream data;
    fstream time;
    fstream time2;
    fstream pid_output;
    file.open("input10008.txt");
    data.open("data10008.txt");
    time.open("time10008.txt");
    time2.open("time210008.txt");
    pid_output.open("pid_output10008.txt");

    int i = 1;
    int j = 1;

    if (file.is_open() && data.is_open() && time.is_open() && time2.is_open() && pid_output.is_open())
    {

        string line;
        while (getline(file, line))
        {
            if (j < 236) // When changes from coarse to fine
            {
                time << line.substr(0, 12) << endl;
                data << line.substr(13, line.length() - 1) << endl;
                j++;
            }
            else
            {
                if (i % 2 != 0)
                {
                    time << line.substr(0, 12) << endl;
                    data << line.substr(13, line.length() - 1) << endl;
                }
                else
                {
                    time2 << line.substr(0, 12) << endl;
                    pid_output << line.substr(25, line.length() - 1) << endl;
                }
                i++;
            }
        }
    }
    cout << "Done!" << endl;

    file.close();
    data.close();
    time.close();
    time2.close();
    pid_output.close();
}
