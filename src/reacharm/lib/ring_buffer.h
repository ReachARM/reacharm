//
// Created by mojo on 03/10/15.
//

#ifndef REACHARM_RING_BUFFER_H
#define REACHARM_RING_BUFFER_H

#include <vector>
#include <algorithm>

template<class T>
class ring_buffer {

 public :

  using value_type = T;

  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  ring_buffer(const int capacity) noexcept : capacity_(capacity){}
  ~ring_buffer() noexcept {}

  //==========================================================================
  // P U B L I C   M E T H O D S

  value_type operator [](const int index) const {
   if(index > vals_.size())
    return 0;
   return vals_[index];
  }

  value_type& operator [](const int index){
   if(index > vals_.size())
    return nullptr;
   return vals_[index];
  }

  void add(value_type& val) noexcept {
   if(vals_.size() > capacity_)
    vals_.push_back(val);
   else
    vals_[index_] = val;
   index_ += 1;
   if( index_ > capacity_)
    index_ = 0;
  }

  typedef typename std::vector<T>::const_iterator iterator;
  iterator begin() const noexcept { return vals_.begin(); }
  iterator end() const noexcept  { return vals_.end(); }

  int size() const noexcept { return vals_.size(); }

  std::vector<value_type> sort() const noexcept {
   std::vector<value_type> temp(vals_);
   std::sort(temp.begin(),temp.end());
   return temp;
  }

 private :

  std::vector<value_type> vals_;

  int index_;

  const int capacity_;

};


#endif //REACHARM_RING_BUFFER_H
