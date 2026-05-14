#ifndef SIZEDSTACK_H
#define SIZEDSTACK_H

#include<vector>
template <typename T>
class sizedStack
{
public:
    explicit sizedStack()=default;
    explicit sizedStack(int s) {
        maxSize = s;
        container = new std::vector<T>();
        container->reserve(maxSize); // 预分配空间
    }// 指定大小
    ~sizedStack(){
        delete container;
    } // 析构函数
    std::vector<T>* container = nullptr;
    int maxSize = 10; // 改名为maxSize更清晰
    int getsize(){
        return container->size();
    }
    void setSize(int size){
        maxSize = size;
        // 可能需要截断现有元素
        if (container->size() > maxSize) {
            container->erase(container->begin(), container->end() - maxSize);
        }
    }
    void push(T obj){
        if (container->size() >= maxSize) {
            // 达到最大值，移除最老的元素
            container->erase(container->begin());
        }
        container->push_back(obj);// 进栈
    }
    void pop(){
        if (!container->empty()) {
            container->pop_back();
        }
    } // 出栈一个
    T* getTop(){
        if (!container->empty())
            return &(container->back()); // 返回引用更合理
        else
            return nullptr;
    } // 栈顶元素
    void clear(){
        container->clear();
    }


};

#endif // SIZEDSTACK_H
