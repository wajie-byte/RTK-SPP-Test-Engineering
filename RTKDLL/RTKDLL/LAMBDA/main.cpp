/*

题目：
输入为一棵二叉树的层次遍历序列，其中空结点用特殊字符 `'#'` 表示。
例如，输入 "ABC#D#E##F" 表示如下二叉树
               A
             /   \
            B     C
             \     \
              D     E
             /
            F

下列函数，大家根据题目要求和初始代码的上下文，自行设计其参数与返回值

1. 实现函数buildtree(), 生成该二叉树的链式存储

2. 实现函数thread(), 将该二叉树按中序遍历线索化

3. 实现函数inorderTraversal(), 对线索二叉树进行中序遍历，输出遍历结点序列。
   当使用某节点的线索指针进行遍历时，输出"Thread_in_node_X", 这里X为该结点对应字符
   
4. 实现函数getSuccessorInPreorder(), 在线索化二叉树中寻找某个结点在前序遍历中的后继结点。
   在main()函数中生成中序遍历结点序列，顺次输出序列中每个结点的在前序遍历中的后继结点
   
5. 实现函数getPredecessorInPostorder()，在该线索化二叉树中寻找某个结点在后序遍历中的前驱结点。
   在main()函数中生成中序遍历结点序列，顺次输出序列中每个结点的在后序遍历中的前驱结点


输入格式：
字符串

输出格式
字符1 字符2 ... 字符n-1 字符n
字符1 字符2 ... 字符n-1
字符1 字符2 ... 字符n-1

例如:
输入 : ABC#D#E##F

输出 : 
B F Thread_in_node_F D Thread_in_node_D A C E
D C F B E
D F C E B

*/

#include <iostream>
#include <stack>
#include <queue>
#include <vector>
#include <string>
using namespace std;

struct node
{
    char data;
    char ltag, rtag;
    node* lchild, * rchild;
};

// 实现 buildtree()函数---------------------------------------





// ---------------buildtree()  end---------------------------
// 实现 thread()函数---------------------------------------





// ---------------thread()  end---------------------------

// 实现 inorderTraversal()函数---------------------------------------





// ---------------inorderTraversal()  end---------------------------

// 实现 getSuccessorInPreorder()函数---------------------------------------





// ---------------getSuccessorInPreorder()  end---------------------------

// 实现 getPredecessorInPostorder()函数---------------------------------------





// ---------------getPredecessorInPostorder()  end---------------------------



int main()
{

    string str;
    cin >> str;
    node* root = buildtree(str.c_str());

    //不要修改main()函数中以上部分的内容---------------------------


    //1、调用thread()函数线索化二叉树



    //2、调用inorderTraversal()遍历输出二叉树



    //3、编写程序, 使用getSuccessorInPreorder()
    //输出二叉树中各个结点在前序序列中的后继结点



    //4、编写程序, 使用getPredecessorInPostorder()
    //输出二叉树中各个结点在后序序列中的前驱结点





    //不要修改main()函数中以下部分的内容---------------------------


    return 0;
}