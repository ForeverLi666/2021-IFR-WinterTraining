#include <stdio.h> 
#include <stdlib.h>
#include "_Linked_List.h"
int main()
{
	List list1;
	List list2;
	list1.head=list1.tail=NULL;
	list2.head=list2.tail=NULL;
	List New_list;
	New_list.head=New_list.tail=NULL;
	int number=0;
	int Number=0;
	while(number!=6)
	{
		printf("-----------------------------------链表1-------------------------------------\n");
		Print(&list1);
		printf("-----------------------------------链表2-------------------------------------\n");
		Print(&list2);
		printf("----------------------------------合并链表-----------------------------------\n");
		Print(&New_list);
		printf("请选择您要做的：1.插入节点至链表1。    2.插入节点至链表2。   3.删除链表1中的节点。\n"); 
		printf("                4.删除链表2中的节点。  5.合并链表1和链表2。  6.退出程序。\n\n");
		printf("其中1、2、3、4操作方法为:操作序号->Enter->数据。\n");
		printf("例如插入节点(节点中数据为3)至链表1：1->Enter->3。\n");
		scanf("%d",&number);
		switch(number) 
		{
			case 1:
				scanf("%d",&Number);
				add(&list1,Number);
				system("cls");
				break;
			case 2:
				scanf("%d",&Number);
				add(&list2,Number);
				system("cls");
				break;
			case 3:
				scanf("%d",&Number);
				cut(&list1,Number);
				system("cls");
				break;
			case 4:
				scanf("%d",&Number);
				cut(&list2,Number);
				system("cls");
				break;
			case 5:
				Link(list1,list2,&New_list);
				system("cls");
				break;
		}
	}
	return 0;
}
