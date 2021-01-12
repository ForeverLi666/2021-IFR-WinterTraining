#include <stdio.h>
#include <stdlib.h>
#include "_Linked_List.h"
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
		if(p->value>=plist->tail->value)
		{
			plist->tail->next=p;
			p->prior=plist->tail;
			plist->tail=p;
		}
		else
		{
			for(q1,q=plist->head;q;q1=q,q=q->next) 
			{
				if(p->value<=q->value)
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
void Link(List list1,List list2,List *New_plist)
{
	if(list1.head->value<=list2.head->value)
	{
		New_plist->head=list1.head;
		Node *p=list1.head;
		Node *p1=list1.head->next;
		Node *p2=list2.head;
		while(p1&&p2)
		{
			if(p1->value<p2->value)
			{
				p->next=p1;
				p1->prior=p;
				p=p1;
				p1=p1->next;
			}
			else if(p1->value>p2->value)
			{
				p->next=p2;
				p2->prior=p;
				p=p2;
				p2=p2->next;
			}
			else if(p1->value=p2->value)
			{
				p->next=p1;
				p1->prior=p;
				p=p1;
				p1=p1->next;
				p2=p2->next;
			}
		}
		while(p1==NULL&&p2!=NULL)
		{
			p->next=p2;
			p2->prior=p;
			p=p2;
			p2=p2->next;
			New_plist->tail=p2;		
		}
		while(p2==NULL&&p1!=NULL)
		{
			p->next=p1;
			p1->prior=p;
			p=p1;
			p1=p1->next;
			New_plist->tail=p1;
		}
	}
	else
	{
		New_plist->head=list2.head;
		Node *p=list2.head;
		Node *p2=list2.head->next;
		Node *p1=list1.head;
		while(p1&&p2)
		{
			if(p1->value<p2->value)
			{
				p->next=p1;
				p1->prior=p;
				p=p1;
				p1=p1->next;
			}
			else if(p1->value>p2->value)
			{
				p->next=p2;
				p2->prior=p;
				p=p2;
				p2=p2->next;
			}
			else if(p1->value=p2->value)
			{
				p->next=p1;
				p1->prior=p;
				p=p1;
				p1=p1->next;
				p2=p2->next;
			}
		}
		while(p1==NULL&&p2!=NULL)
		{
			p->next=p2;
			p2->prior=p;
			p=p2;
			p2=p2->next;
			New_plist->tail=p2;		
		}
		while(p2==NULL&&p1!=NULL)
		{
			p->next=p1;
			p1->prior=p;
			p=p1;
			p1=p1->next;
			New_plist->tail=p1;
		}
	}
} 
