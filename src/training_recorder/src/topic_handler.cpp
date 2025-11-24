
#include "training_recorder/topic_handler.hpp"

namespace training_recorder {
BufferPool::BufferPool(size_t buf_size, size_t pool_size)
    : buf_size_(buf_size) {
  for (size_t i = 0; i < pool_size; ++i) {
    auto ptr = new uint8_t[buf_size_];
    free_list_.push_back(ptr);
  }
}

BufferPool::~BufferPool() {
  for (auto p : free_list_)
    delete[] p;
  for (auto p : leased_)
    delete[] p;
}

uint8_t *BufferPool::acquire() {
  std::lock_guard<std::mutex> lk(m_);
  if (!free_list_.empty()) {
    uint8_t *p = free_list_.back();
    free_list_.pop_back();
    leased_.push_back(p);
    return p;
  }
  // pool exhausted - allocate fallback (should be avoided by sizing)
  uint8_t *p = new uint8_t[buf_size_];
  leased_.push_back(p);
  return p;
}

void BufferPool::release(uint8_t *p) {
  std::lock_guard<std::mutex> lk(m_);
  // put back into free list if it belongs to original pool
  free_list_.push_back(p);
  // note: leaked allocated buffers are not removed; in production
  // you'd track which pointers came from pool vs fallback
}

size_t BufferPool::buffer_capacity() const { return buf_size_; }

// Lock-protected queue of SerializedBuffer shared pointers
MessageQueue::MessageQueue(size_t capacity) : capacity_(capacity) {}

bool MessageQueue::push(std::shared_ptr<SerializedBuffer> msg) {
  std::unique_lock<std::mutex> lk(m_);
  if (queue_.size() >= capacity_) {
    // queue full
    return false;
  }
  queue_.push_back(msg);
  lk.unlock();
  cv_.notify_one();
  return true;
}
// blocking pop with shutdown awareness
std::shared_ptr<SerializedBuffer>
MessageQueue::pop_or_wait(bool &shutdown_flag) {
  std::unique_lock<std::mutex> lk(m_);
  cv_.wait(lk, [&]() { return !queue_.empty() || shutdown_flag; });
  if (!queue_.empty()) {
    auto item = queue_.front();
    queue_.pop_front();
    return item;
  }
  return nullptr;
}
// attempt non-blocking pop
std::shared_ptr<SerializedBuffer> MessageQueue::try_pop() {
  std::unique_lock<std::mutex> lk(m_);
  if (queue_.empty())
    return nullptr;
  auto item = queue_.front();
  queue_.pop_front();
  return item;
}
size_t MessageQueue::size() const {
  std::lock_guard<std::mutex> lk(m_);
  return queue_.size();
}
} // namespace training_recorder
