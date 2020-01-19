#include "iplanner/engine.h"

#include <iostream>

int main()
{
  iplanner::Engine engine;

  try
  {
    engine.Run();
  }
  catch (const std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }

  return 0;
}
