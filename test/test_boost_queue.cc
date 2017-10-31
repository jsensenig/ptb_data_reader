#include <iostream>
#include <boost/lockfree/spsc_queue.hpp>

using std::cout;
using std::endl;

typedef struct buffer_t {
    int       handle;
    size_t    len;
} buffer_t;

int main() {

const size_t num_buffs_ = 1024;
//boost::lockfree::spsc_queue<buffer_t> buffer_queue_{1024};
boost::lockfree::spsc_queue<buffer_t, boost::lockfree::capacity<num_buffs_> >buffer_queue_;
buffer_t buf;

for (size_t i = 0; i < 10; i++) {
  buf.len = i;
  buf.handle = i+3;
  buffer_queue_.push(buf);
}

buffer_t bufother;

for (size_t i = 0; i < 10; i++) {
  if (!buffer_queue_.pop(bufother)) {
    cout << "Failed..." << endl;
  }
  cout << "len " << bufother.len << " handle " << bufother.handle << endl;
}


  return 0;
}