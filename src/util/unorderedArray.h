#ifndef UNORDEREDARRAY_H
#define UNORDEREDARRAY_H

#include "includes.h"

template <typename T, size_t N>
class UnorderedArray {
    std::array<T, N> data;
    size_t _size = 0;

    public:
    void add(const T& element) {
        if (_size == N) return;
        data[_size++] = element;
    }

    void remove(size_t index) {
        if (index >= _size) return;
        if (index < _size - 1) std::swap(data[index], data[_size - 1]);
        _size--;
    }

    T& operator[](size_t index) { return data[index]; }
    const T& operator[](size_t index) const { return data[index]; }

    void clear() { _size = 0; }
    size_t size() const { return _size; }
};

#endif