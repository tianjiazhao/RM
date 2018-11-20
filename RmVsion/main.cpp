#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include "serial.h"
#include "Imageconsprod.h"
#include "Armordetector.h"
#include <thread>
int main()
{
//    ArmorDetector a;
//    a.text();
      int fd2car;
//      fd2car = OpenPort("/dev/ttyUSB0");
//      configurePort(fd2car);
      ImageConsProd image_cons_prod(fd2car);
      std::thread t1(&ImageConsProd::ImageConsumer,image_cons_prod);
      std::thread t2(&ImageConsProd::ImageProducer,image_cons_prod);
      t1.join();
      t2.join();


}
