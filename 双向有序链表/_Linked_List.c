#include <stdlib.h>
#include "_Linked_List.h"
int t=0; ;
void add(List *plist,int number)
{
	Node *p=(Node*)malloc(sizeof(Node));
	Node *q=NULL;
	Node *q1=NULL;
	p->value=number;
	p->prior=NULL;
	p->next=NULL;
	if(plist->tail)
	{
		if(p->value>plist->tail->value)
		{
			plist->tail->next=p;
			p->prior=plist->tail;
			plist->tail=p;
		}
		else if(p->value<plist->tail->value)
		{
			for(q1,q=plist->head;q;q1=q,q=q->next) 
			{
				if(p->value==q->value)
				{
					break; 
				}
				if(p->value<q->value)
				{
					if(q1)
					{
						q1->next=p;
						p->prior=q1;
						p->next=q;
						q->prior=p;
					}
					else
					{
						q->prior=p;
						p->next=q;
						plist->head=p;
					}
					break;
				}
			}
		}
	}
	else
	{
		plist->head=plist->tail=p;
	}
}
void Print(List *plist)
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
	Node *p1=NULL;
	Node *q1=NULL;
	Node *p2=NULL;
	Node *q2=NULL;
	for(q1,p1=plist->head,q2,p2=plist->tail;p1!=p2;q1=p1,p1=p1->next,q2=p2,p2=p2->prior) 
	{
		if(p1->value==number)
		{
			if(q1)
			{
				q1->next=p1->next;
				p1->next->prior=q1;
			}
			else
			{
				plist->head=p1->next;
				p1->next->prior=NULL;
			}
			free(p1);
			break;
		}
		if(p2->value==number)
		{
			if(q2)
			{
				q2->prior=p2->prior;
				p2->prior->next=q2;
			}
			else
			{
				plist->tail=p2->prior;
				p2->prior->next=NULL;
			}
			free(p2);
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
void Link(List *plist1,List *plist2,List *New_plist)
{
	if(plist1->head->value<=plist2->head->value)
	{
		Node *p1=plist1->head;
		Node *p2=plist2->head;
		for(p1;p1;p1=p1->next)
		{
			add(New_plist,p1->value);
		}
		for(p2;p2;p2=p2->next)
		{
			add(New_plist,p2->value);
		}
	}
	else if(plist1->head->value>plist2->head->value)
	{
		Node *p1=plist1->head;
		Node *p2=plist2->head;
		for(p2;p2;p2=p2->next)
		{
			add(New_plist,p2->value);
		}
		for(p1;p1;p1=p1->next)
		{
			add(New_plist,p1->value);
		}
	}
}
