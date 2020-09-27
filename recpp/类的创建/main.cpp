#include <iostream>

using namespace std;

class Box
{
   public:
      double height;   // 高度
      double width;   // 高度

   double getHeight(void){
      // this 是实例的指针
      return this->height;
   };
};

int main( )
{
   // 方法1,隐式创建,在栈中分配内存
   Box b1;
   b1.height = 5.0;
   cout<< b1.getHeight()<<endl;

   // 方法2,显示创建
   Box b2 = Box();
   b2.height = 5.0;
   cout<< b2.height<<endl;

  // 方法3,显示new 创建,在堆中分配内存，需要手动释放
  Box * b3 = new Box();
  b3->height = 5.0;
  cout<< b3->height<<endl;
  delete b3;
  b3 = NULL;
  return 0;
}
