#include <iostream>

#define RED "\x1b[31m"
#define GREEN "\x1b[32m"
#define RESET "\x1b[m"

using std::cout;
using std::endl;

int main(int argc, char* args[]) {
    for(int i= 0; i < argc; i++) {
        if(i > 0 && atoi(args[i])%2==0)
            cout << RED << args[i] << RESET << " ";
        else
            cout << GREEN << args[i] << RESET << " ";
    }
    cout << endl;

    if(argc > 2)
        cout << atoi(args[1]) +  atoi(args[2]) << endl;
    
    return 0;
}