#ifndef __Linked_List_H__
#define __Linked_List_H__
#include <stdio.h>
typedef struct _Node{
	int value;
	struct _Node *prior;
	struct _Node *next;
}Node;
typedef struct _List{
	Node *head;
	Node *tail;
}List;
void add(List *plist,int number);
void Print(List *plist);
void cut(List *plist,int number);
void Free(List *list);
void Link(List list1,List list2,List *New_plist);
#endif
