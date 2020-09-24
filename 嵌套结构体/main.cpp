#include<iostream>
using namespace std;

struct aaa {
    int a;
};

struct bbb: public aaa {
    int b;
};

int main()
{
    bbb b1;
    b1.a = 1;
    b1.b = 2;
    std::cout <<b1.a<<endl;
    std::cout <<b1.b<< endl;
    return 0;
}