#include <iostream>
#include <utility>


template <typename T>
class Check{
public:
  virtual void f1() = 0;
  virtual void f2(){};
};

class Simple: public Check<int>{
public:
  void f1() override {
    std::cout << "I did it!" << std::endl;
  }
  // void f2() override {
  //   std::cout << "I did it!" << std::endl;
//  }
};


struct imu_image
{
    std::string imu;
    std::string image;
};

int main()
{
  auto p = std::make_pair(1, 2);
  // std::cout<<p.first;
  auto ch = Simple{};
  ch.f1();

  return 0;
}
