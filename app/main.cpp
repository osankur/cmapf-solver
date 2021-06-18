/* Copyright (c) 2021 Arthur Queffelec
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 */

#include <Window.hpp>
#include <iostream>
#include <functional>

int main() {
  std::cout << "Hello CMake." << std::endl;
  return 0;
}

int uimain(std::function<int()> run) {
  sciter::archive::instance().open(
      aux::elements_of(resources));
  sciter::om::hasset<Frame> pwin = new Frame();
  pwin->load(WSTR("this://app/index.htm"));

  pwin->expand();

  return run();
}