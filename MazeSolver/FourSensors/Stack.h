#ifndef STACK_H_INCLUDED
#define STACK_H_INCLUDED

class Stack {
    size_t maxLength;
    size_t top;
    int* arr;

    public:
        Stack(int maxLength);
        int size();
        int get(int index);
        void push(int newElement);
        void pop();
        bool isEmpty();
        bool isFull();
};

Stack::Stack(int maxLength) : maxLength(maxLength) {
    arr = new int[maxLength];
    top = 0;
}

void Stack::push(int newElement) {
    if (!isFull())
        arr[top++] = newElement;
}

void Stack::pop() {
    if (!isEmpty())
        top--;
}

int Stack::get(int index) {
    if (size() > index && index >= 0)
        return arr[index];
    else if (index < 0)
        return arr[size() - index];
}

int Stack::size() {
    return top;
}

bool Stack::isEmpty() {
    return top == 0;
}

bool Stack::isFull() {
    return top == maxLength;
}


#endif