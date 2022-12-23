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
	size_t size()const;  //集合中元素的数目
	bool is_empty()const;//如果集合为空，返回true

	void _init_set(const T* p, size_t len);

	bool is_exist(const T v);
	bool is_exist(std::set<T> s1, const T v);

	Set();
	Set(const Set& a);
	Set(const T* p, const size_t len);

	void operator=(const Set& a);

	Set operator+(const Set& B); //集合并集
	Set operator-(const Set& B); //集合差集

	void add(const T value);  //为集合添加元素
	void remove(const T v);      //移除指定元素
	void erase(const T v);
	void clear();              //清除所有元素
	void copy(const Set a);


	// A-B=empty说明A是B的子集
	Set intersection(const Set B, bool updatabase = false); //取集合交集

	//判断两个集合是否包含相同的元素，如果没有返回 True，否则返回 False
	bool isdisjoint(const Set& B);

	//判断指定集合是否为该方法参数集合的子集。
	bool issubset(const Set father, bool updatabase = false); //子集

	//判断该方法的参数集合是否为指定集合的子集
	bool issuperset(const Set son, bool updatabase = false);  //父集

	Set union_(const Set B, bool updatabase = false);       //取集合并集

	Set difference(const Set B, bool updatabase = false);   //取集合差集

	Set symmetric_difference(const Set B, bool updatabase = false); //取集合对称差集-返回两个集合中不重复的元素集合

	void setdata(const T* arr, const size_t len);

	void print(const char* name = "", Set* pSet = NULL);
};

template<typename T>
size_t Set<T>::len()const { return s.max_size(); }

template<typename T>
size_t Set<T>::size()const { return s.size(); }   //集合中元素的数目

template<typename T>
bool Set<T>::is_empty()const { return s.empty(); }// 如果集合为空，返回true

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
}//集合并集

template<typename T>
Set<T> Set<T>::operator -(const Set& B)
{
	return difference(B, false);
}//集合差集

template<typename T>
void Set<T>::add(const T value) //为集合添加元素
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
void Set<T>::clear() //清除所有元素
{
	s.clear();
}

template<typename T>
void Set<T>::copy(const Set a)
{
	s = a.s;
}

// A-B=empty说明A是B的子集
template<typename T>
Set<T> Set<T>::intersection(const Set B, bool updatabase) //取集合交集
{
	static Set a;
	std::set<T> tmp;
	set_intersection(s.begin(), s.end(), B.s.begin(), B.s.end(), inserter(tmp, tmp.begin()));
	//参数依次是第1/2集合头尾;参数5将集合A、B取合集后的结果存入集合C中
	a.s = tmp;
	if (updatabase)
		s = tmp;
	return a;
}

template<typename T>
//判断两个集合是否包含相同的元素，如果没有返回 True，否则返回 False
bool Set<T>::isdisjoint(const Set& B)
{
	std::set<T> tmp;
	set_difference(s.begin(), s.end(), B.s.begin(), B.s.end(), inserter(tmp, tmp.begin()));
	return (tmp.size() != s.size()) ? true : false;
}

template<typename T>
//判断指定集合是否为该方法参数集合的子集。
bool Set<T>::issubset(const Set father, bool updatabase) //子集
{
	std::set<T> tmp;
	set_difference(s.begin(), s.end(), father.s.begin(), father.s.end(), inserter(tmp, tmp.begin()));
	return (tmp.empty()) ? true : false;
}

template<typename T>
//判断该方法的参数集合是否为指定集合的子集
bool Set<T>::issuperset(const Set son, bool updatabase) //父集
{
	std::set<T> tmp;
	set_difference(son.s.begin(), son.s.end(), s.begin(), s.end(), inserter(tmp, tmp.begin()));
	return (tmp.empty()) ? true : false;
}

template<typename T>
Set<T> Set<T>::union_(const Set B, bool updatabase) //取集合并集
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
Set<T> Set<T>::difference(const Set B, bool updatabase) //取集合差集
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
Set<T> Set<T>::symmetric_difference(const Set B, bool updatabase) //取集合对称差集-返回两个集合中不重复的元素集合
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
