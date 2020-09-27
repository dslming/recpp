#include <iostream>
#include <string>
#include <memory>
using namespace std;

class Test
{
public:
    Test(string name)
    {
        name_ = name;
        cout << this->name_ << "  constructor" << endl;
    }
    ~Test()
    {
        cout << this->name_ << "  destructor" << endl;
    }

    string name_;
};


int main()
{
    /* 类对象 原生指针构造 */
    shared_ptr<Test> pStr1(new Test("object"));
    cout << (*pStr1).name_ << endl;
    
    /* use_count()检查引用计数 */
    cout << "pStr1 引用计数：" << pStr1.use_count() << endl;

    shared_ptr<Test> pStr2 = pStr1;
    cout << (*pStr2).name_ << endl;
    cout << "pStr1 引用计数：" << pStr1.use_count() << endl;
    cout << "pStr2 引用计数：" << pStr2.use_count() << endl;
    return 0;
}