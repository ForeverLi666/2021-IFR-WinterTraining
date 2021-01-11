#include <stdio.h>
#include <stdlib.h>
#include "Linked_List.h"
void add(List *plist,int number)
{
	Node *p=(Node*)malloc(sizeof(Node));
	p->value=number;
	p->next=NULL;
	Node *last=plist->head;
	if(last)
	{
		while (last->next)
		{
			last=last->next;
		}
		last->next=p; 
	}else
	{
		plist->head=p;
	}
}
void find(List *plist)
{
	Node *p=NULL;
	for(p=plist->head;p;p=p->next)
	{
		printf("%d\t",p->value);
	}
	printf("\n");
}
void cut(List *plist,int number)
{
	Node *p=NULL;
	Node *q=NULL;
	for(q,p=plist->head;p;q=p,p=p->next) 
	{
		if(p->value==number)
		{
			if(q)
			{
				q->next=p->next;
			}else
			{
				plist->head=p->next;
			}
			free(p);
			break;
		}
	}
}
void Free(List *list)
{
	Node *p=NULL;
	Node *q=NULL;
	for(p=list->head;p;p=q)
	{
		q=p->next;
		free(p);
	}
}
