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
		printf("-----------------------------------����1-------------------------------------\n");
		Print(&list1);
		printf("-----------------------------------����2-------------------------------------\n");
		Print(&list2);
		printf("----------------------------------�ϲ�����-----------------------------------\n");
		Print(&New_list);
		printf("��ѡ����Ҫ���ģ�1.����ڵ�������1��    2.����ڵ�������2��   3.ɾ������1�еĽڵ㡣\n"); 
		printf("                4.ɾ������2�еĽڵ㡣  5.�ϲ�����1������2��  6.�˳�����\n\n");
		printf("����1��2��3��4��������Ϊ:�������->Enter->���ݡ�\n");
		printf("�������ڵ�(�ڵ�������Ϊ3)������1��1->Enter->3��\n");
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
