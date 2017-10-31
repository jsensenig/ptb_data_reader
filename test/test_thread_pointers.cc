/*
 * test_thread_pointers.cc
 *
 *  Created on: Oct 31, 2017
 *      Author: nbarros
 */

#include <iostream>
#include <thread>

using std::cout;
using std::endl;

bool keep_running = true;
void func_join() {
  cout.sync_with_stdio(true);
  cout << "func_join::Running thread " << std::this_thread::get_id() << endl;
  for (size_t i = 0; i < 20; i++) {

    std::this_thread::sleep_for(std::chrono::seconds(1));
    cout << "func_join:: iteration " << i << endl;
  }
  cout << "func_join:: joining back." << endl;
}

void func_destroy() {
  cout.sync_with_stdio(true);
  cout << "func_destroy::Running thread " << std::this_thread::get_id() << endl;
  for (size_t i = 0; i < 50; i++) {
    if (!keep_running) {
      cout << "func_destroy:: leaving..." << endl;
      return;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    cout << "func_destroy:: iteration " << i << endl;
  }
  cout << "func_destroy:: joining back." << endl;
}

int main() {
  cout.sync_with_stdio(true);

  std::thread *t1 = nullptr;
  std::thread *t2 = nullptr;
  cout << "Starting the threads..." << endl;
  t1 = new std::thread(func_join);
  cout << "Starting the second thread..." << endl;
  t2 = new std::thread(func_destroy);
  cout << "Status of the threads::" << endl;
  cout << "T1 : " <<  t1->get_id() << " " << t1->joinable() << endl;
  cout << "T2 : " <<  t2->get_id() << " " << t2->joinable() << endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));
  cout << "Destroying T2 : " << t2 << endl;
  //delete t2;
  keep_running = false;
  std::this_thread::sleep_for(std::chrono::seconds(1));

  cout << "Status of t2 : " << t2 << endl;
  t2 = nullptr;
  cout << "Status of t2 : " << t2 << endl;
  cout << "Joining t1..." << endl;
  t1->join();
  cout << "All ready ... t1=" << t1 << endl;
  delete t1;
  cout << "All ready ... t1=" << t1 << endl;
  t1 = nullptr;
  return 0;

}



