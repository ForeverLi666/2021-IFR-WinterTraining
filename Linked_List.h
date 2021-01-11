#ifndef Linked_List
#define Linked_List
typedef struct _Node{
	int value;
	struct _Node *next;
}Node;
typedef struct _List{
	Node *head;
	Node *tail;
}List;
void add(List *plist,int number);
void find(List *plist);
void cut(List *plist,int number);
void Free(List *list);
#endif
