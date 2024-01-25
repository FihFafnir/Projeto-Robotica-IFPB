#pragma once

template <typename T, size_t S>
class StaticStack {
    T m_data[S];
    size_t m_capacity;
    size_t m_top;

    public:
        StaticStack();

        size_t size();
        bool isEmpty();
        bool isFull();
        
        void push(T element);
        void pop();
        
        T& operator[](int index);
        T& get(int index);
        T& top();
};

template <typename T, size_t S>
StaticStack<T, S>::StaticStack() : m_capacity(S), m_top(0) {}

template <typename T, size_t S>
size_t StaticStack<T, S>::size() {
    return m_top;
}

template <typename T, size_t S>
bool StaticStack<T, S>::isEmpty() {
    return !m_top;
}

template <typename T, size_t S>
bool StaticStack<T, S>::isFull() {
    return m_top == m_capacity;
}

template <typename T, size_t S>
void StaticStack<T, S>::push(T element) {
    if (!isFull())
        m_data[m_top++] = element;
}

template <typename T, size_t S>
void StaticStack<T, S>::pop() {
    if (!isEmpty())
        m_top--;
}

template <typename T, size_t S>
T& StaticStack<T, S>::operator[](int index) {
    return m_data[index < 0 ? m_top + index : index];
}

template <typename T, size_t S>
T& StaticStack<T, S>::get(int index) {
    return m_data[index < 0 ? m_top + index : index];
}

template <typename T, size_t S>
T& StaticStack<T, S>::top() {
    return m_data[m_top - 1];
}
