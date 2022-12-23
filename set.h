#pragma once
#include <set>
#include <iostream>


using namespace std;

template<typename T>
class Set
{
private:
	std::set<T> s;
	typename std::set<T>::iterator iter;

public:
	size_t len()const;
	size_t size()const;  //������Ԫ�ص���Ŀ
	bool is_empty()const;//�������Ϊ�գ�����true

	void _init_set(const T* p, size_t len);

	bool is_exist(const T v);
	bool is_exist(std::set<T> s1, const T v);

	Set();
	Set(const Set& a);
	Set(const T* p, const size_t len);

	void operator=(const Set& a);

	Set operator+(const Set& B); //���ϲ���
	Set operator-(const Set& B); //���ϲ

	void add(const T value);  //Ϊ�������Ԫ��
	void remove(const T v);      //�Ƴ�ָ��Ԫ��
	void erase(const T v);
	void clear();              //�������Ԫ��
	void copy(const Set a);


	// A-B=empty˵��A��B���Ӽ�
	Set intersection(const Set B, bool updatabase = false); //ȡ���Ͻ���

	//�ж����������Ƿ������ͬ��Ԫ�أ����û�з��� True�����򷵻� False
	bool isdisjoint(const Set& B);

	//�ж�ָ�������Ƿ�Ϊ�÷����������ϵ��Ӽ���
	bool issubset(const Set father, bool updatabase = false); //�Ӽ�

	//�жϸ÷����Ĳ��������Ƿ�Ϊָ�����ϵ��Ӽ�
	bool issuperset(const Set son, bool updatabase = false);  //����

	Set union_(const Set B, bool updatabase = false);       //ȡ���ϲ���

	Set difference(const Set B, bool updatabase = false);   //ȡ���ϲ

	Set symmetric_difference(const Set B, bool updatabase = false); //ȡ���϶ԳƲ-�������������в��ظ���Ԫ�ؼ���

	void setdata(const T* arr, const size_t len);

	void print(const char* name = "", Set* pSet = NULL);
};

template<typename T>
size_t Set<T>::len()const { return s.max_size(); }

template<typename T>
size_t Set<T>::size()const { return s.size(); }   //������Ԫ�ص���Ŀ

template<typename T>
bool Set<T>::is_empty()const { return s.empty(); }// �������Ϊ�գ�����true

template<typename T>
void Set<T>::_init_set(const T* p, size_t len) {
	for (size_t i = 0; i < len; i++) s.insert(p[i]);
}

template<typename T>
bool Set<T>::is_exist(const T v) {
	//set<T>::iterator iter;
	iter = s.find(v);
	return (iter == s.end()) ? false : true;
}

template<typename T>
bool Set<T>::is_exist(std::set<T> s1, const T v) {
	//set<T>::iterator iter;
	iter = s1.find(v);
	return (iter == s1.end()) ? false : true;
}

template<typename T>
Set<T>::Set() {}

template<typename T>
Set<T>::Set(const Set& a) { s = a.s; iter = a.iter; }

template<typename T>
Set<T>::Set(const T* p, const size_t len) { _init_set(p, len); }

template<typename T>
void Set<T>::operator=(const Set& a) { s = a.s; iter = a.iter; }

template<typename T>
Set<T> Set<T>::operator+(const Set& B)
{
	return union_(B, false);
}//���ϲ���

template<typename T>
Set<T> Set<T>::operator -(const Set& B)
{
	return difference(B, false);
}//���ϲ

template<typename T>
void Set<T>::add(const T value) //Ϊ�������Ԫ��
{
	s.insert(value);
}

template<typename T>
void Set<T>::remove(const T v)
{
	if (is_exist(v))
		s.erase(v);
}

template<typename T>
void Set<T>::erase(const T v)
{
	remove(v);
}

template<typename T>
void Set<T>::clear() //�������Ԫ��
{
	s.clear();
}

template<typename T>
void Set<T>::copy(const Set a)
{
	s = a.s;
}

// A-B=empty˵��A��B���Ӽ�
template<typename T>
Set<T> Set<T>::intersection(const Set B, bool updatabase) //ȡ���Ͻ���
{
	static Set a;
	std::set<T> tmp;
	set_intersection(s.begin(), s.end(), B.s.begin(), B.s.end(), inserter(tmp, tmp.begin()));
	//���������ǵ�1/2����ͷβ;����5������A��Bȡ�ϼ���Ľ�����뼯��C��
	a.s = tmp;
	if (updatabase)
		s = tmp;
	return a;
}

template<typename T>
//�ж����������Ƿ������ͬ��Ԫ�أ����û�з��� True�����򷵻� False
bool Set<T>::isdisjoint(const Set& B)
{
	std::set<T> tmp;
	set_difference(s.begin(), s.end(), B.s.begin(), B.s.end(), inserter(tmp, tmp.begin()));
	return (tmp.size() != s.size()) ? true : false;
}

template<typename T>
//�ж�ָ�������Ƿ�Ϊ�÷����������ϵ��Ӽ���
bool Set<T>::issubset(const Set father, bool updatabase) //�Ӽ�
{
	std::set<T> tmp;
	set_difference(s.begin(), s.end(), father.s.begin(), father.s.end(), inserter(tmp, tmp.begin()));
	return (tmp.empty()) ? true : false;
}

template<typename T>
//�жϸ÷����Ĳ��������Ƿ�Ϊָ�����ϵ��Ӽ�
bool Set<T>::issuperset(const Set son, bool updatabase) //����
{
	std::set<T> tmp;
	set_difference(son.s.begin(), son.s.end(), s.begin(), s.end(), inserter(tmp, tmp.begin()));
	return (tmp.empty()) ? true : false;
}

template<typename T>
Set<T> Set<T>::union_(const Set B, bool updatabase) //ȡ���ϲ���
{
	static Set a;
	std::set<T> tmp;
	set_union(s.begin(), s.end(), B.s.begin(), B.s.end(), inserter(tmp, tmp.begin()));

	a.s = tmp;
	if (updatabase)
		s = tmp;
	return a;
}

template<typename T>
Set<T> Set<T>::difference(const Set B, bool updatabase) //ȡ���ϲ
{
	static Set a;
	std::set<T> tmp;
	set_difference(s.begin(), s.end(), B.s.begin(), B.s.end(), inserter(tmp, tmp.begin()));

	a.s = tmp;
	if (updatabase)
		s = tmp;
	return a;
}

template<typename T>
Set<T> Set<T>::symmetric_difference(const Set B, bool updatabase) //ȡ���϶ԳƲ-�������������в��ظ���Ԫ�ؼ���
{
	static Set a;
	std::set<T> tmp;
	set_symmetric_difference(s.begin(), s.end(), B.s.begin(), B.s.end(), inserter(tmp, tmp.begin()));

	a.s = tmp;
	if (updatabase)
		s = tmp;
	return a;
}

template<typename T>
void Set<T>::setdata(const T* arr, const size_t len)
{
	s.clear();
	for (size_t i = 0; i < len; i++)
		s.insert(arr[i]);
}

template<typename T>
void Set<T>::print(const char* name, Set* pSet) {
	std::set<T> set = (pSet == NULL) ? s : pSet->s;
	cout << endl;
	if (set.empty()) {
		cout << name << "{None}"; return;
	}
	cout << name << "{";
	for (iter = set.begin(); iter != set.end(); iter++)
		cout << *iter << ",";
	cout << "}" << endl;
}

/*
int main(void)
{
	double arr[] = { 1,2,3,4 }, arr1[] = { 3, 4, 5, 6 }, arr2[] = { 7, 8 }, arr3[] = { 1,2 };
	Set<double> s1, s2(arr, 4), s3(s2), s4, s5;
	s1.print("s1=");
	s2.print("s2=");
	s3.print("s3=");
	s4.print("s4=");

	s1.add(22.22);
	s1.add(22.23);
	s1.add(22.24);
	s1.print("s1=");
	s1.erase(22.22);
	s1.print("s1=");
	s1.clear();
	s1.print("s1=");

	s4 = s2;
	s4.print("s4=");
	s4.clear();
	s4.print("s4=");
	s4.copy(s2);
	s4.print("s4=");
	s1.setdata(arr, 4);
	s2.setdata(arr1, 4);
	s3.setdata(arr2, 2);
	s4.setdata(arr3, 2);
	s1.print("s1=");
	s5 = s1 - s2;
	s5.print("s1-s2=");
	s5 = s1 + s2;
	s5.print("s1+s2=");
	s5.add(1);
	s5.print("s5=");
	s5 = s1.symmetric_difference(s2);
	s5.print("s5=");
	std::cout << s4.issubset(s1) << std::endl;
	std::cout << s1.issuperset(s4) << std::endl;
	std::cout << s1.isdisjoint(s4) << std::endl;
	std::cout << s1.isdisjoint(s3) << std::endl;

	*/
	//set<T> s, s1,s2,s3,s4;
	//set<T>::iterator iter;

	//cout << s.size() << endl;
	//cout << s.max_size() << endl;
	//s.insert(2);
	//s.insert(10);
	//s.insert(11);
	//s.insert(12);
	//s.insert(1);
	//s1 = s;  T d1 = 11.22;
	//try 
		//{
		// iter = s.find(d1);
		// if (iter==s.end())
		//  cout << "null" << endl;
		// else
		// cout << "find=" << *s.find(d1) << endl;
		//}
	//	catch (exception e)
			///*{
			// cout <<e.what() << endl;
			//}*/
			//cout << "====" << s1.size() << endl;
			//for (iter = s1.begin(); iter != s1.end(); iter++)
			//{
			// cout << *iter << endl;
			//}
			//printf("adr=%p;%p", &s1, &s);
			//set_union(s.begin(), s.end(), s1.begin(), s1.end(), inserter(s3, s3.begin()));
			//
			//for (iter = s4.begin(); iter != s4.end(); iter++)
			// cout << "xx=" << *iter << endl;

			///*for (iter = s.begin(); iter != s.end(); ++iter)
			// 15     {
			// 16         cout << *iter << " ";
			// 17     }*/

	//}
	

//=============
