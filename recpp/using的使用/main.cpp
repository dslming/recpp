#include <iostream>
#include <string>
using namespace std;

class Person
{
protected:
  int age;
};

class HeighPerson : public Person
{
public:
  using Person::age;
};

int main()
{
  cout << "pStr1 引用计数：" << pStr1.use_count() << endl;
  cout << "pStr2 引用计数：" << pStr2.use_count() << endl;
  return 0;
}
