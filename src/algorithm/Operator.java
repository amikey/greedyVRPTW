package algorithm;

import java.sql.Time;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.function.Function;

/**
 * 用于输入已知解后，产生各个新解所进行的各种操作算子
 * 包括Two-Exchange、Or-Exchange、Relocation、Exchange、Crossover算子 来源于A Two-Stage
 * Hybrid Local Search for the Vehicle Routing Problem with Time
 * Windows论文用到的这些算子 计划加入的算子：2-opt、k-opt等算子
 * 
 * @author Intgrp
 *
 */
public class Operator<T> {

	/**
	 * 对当前路径的i,j之间的点进行反转，包括i和j这两个点
	 * 
	 * @param a
	 * @param i
	 * @param j
	 * @return
	 */
	public List<T> two_Exchange(List<T> a, int i, int j) {
		// 将i和j包括端点，里面的点做反转
		for (int m = i; m <= (j - i) / 2 + i; m++) {
			T tmp = a.get(m);
			a.set(m, a.get(j - m + i));
			a.set(j - m + i, tmp);
		}
		return a;
	}

	public List<List<T>> groupTwo_Exchange(List<List<T>> routes) {
		// 要产生一个新的操作，就要先对原始路径做拷贝，防止操作后得出的结果比原来更差的试试有保留原来的路径可以继续试验
		List<List<T>> copyRoutes = new ArrayList<List<T>>();
		Iterator<List<T>> iterator = routes.iterator();
		while (iterator.hasNext()) {
			List<T> eachRoute = iterator.next();
			Iterator<T> it = eachRoute.iterator();
			List<T> tmpRoute = new ArrayList<T>();
			while (it.hasNext()) {
				tmpRoute.add(it.next());
			}
			copyRoutes.add(tmpRoute);
		}
		
		
		
		// 随机选择出需要做操作算子的是一条路径
		Random random = new Random();
		int routeIndex = random.nextInt(copyRoutes.size());
		// 获得需要操作的那条路径
		List<T> r1 = copyRoutes.get(routeIndex);
		while (r1.size()<=1) {
			routeIndex = random.nextInt(copyRoutes.size());
			// 获得需要操作的那条路径
			r1 = copyRoutes.get(routeIndex);
		}
		
		int idx1 = random.nextInt(r1.size());
		int idx2 = random.nextInt(r1.size());
//		System.out.println("i="+idx1+"，\t j="+idx2);
		if (idx1>idx2) {
			int tmp = idx1;
			idx1 = idx2;
			idx2 =tmp;
		}
		while (idx2-idx1<=1 || idx2==idx1) {
			idx1 = random.nextInt(r1.size());
			idx2 = random.nextInt(r1.size());
//			System.out.println("r1 size="+r1.size());
//			System.out.println("i="+idx1+"，\t j="+idx2);
		}
//		System.out.println("r1="+r1);
		r1 = two_Exchange(r1, idx1,idx2);
		copyRoutes.set(routeIndex, r1);
//		System.out.println("r1="+r1);
		return copyRoutes;
	}

	/**
	 * 将a路径中删除1或2或3个连续点，然后移动到b路径中，这里只考虑在a,b中较短长度的范围内移来移去
	 * 
	 * @param a
	 *            数组（染色体）a
	 * @param b
	 *            数组（染色体）b
	 * @return
	 */
	public List<List<T>> or_Exchange(T[] a, T[] b) {
		Random random = new Random();
		int moveNum = random.nextInt(3) + 1;// 要删除移动的数量
		int index = random.nextInt((Math.min(a.length, b.length))) + 1;// 从哪个位置开始
		// 如果位置后的数量不足moveNum，则重新再随机
		while (index + moveNum >= a.length) {
			index = random.nextInt() % (Math.min(a.length, b.length));
		}
		System.out.println("index=" + index);
		System.out.println("moveNum=" + moveNum);
		// 对a中，删除index后面moveNum个元素
		List<List<T>> out = new ArrayList<>();
		List<T> tmp = new ArrayList<>();
		for (int i = 0; i < a.length; i++) {
			if (i >= index && i < index + moveNum)
				continue;
			else
				tmp.add(a[i]);
		}
		out.add(tmp);
		tmp = new ArrayList<>();
		for (int i = 0; i < index + moveNum; i++) {
			if (i >= index && i < index + moveNum)
				tmp.add(a[i]);
			else
				tmp.add(b[i]);
		}
		for (int i = index; i < b.length; i++)
			tmp.add(b[i]);
		out.add(tmp);
		return out;
	}

	/**
	 * 将a路径中删除1或2或3个连续点，然后移动到b路径中，这里只考虑在a,b中较短长度的范围内移来移去
	 * 
	 * @param routes
	 *            所有车辆行走路径组成的集合
	 * @return 返回一个新的routes，里面的两条路径产生了函数or_Exchange的效果
	 */
	public List<List<T>> or_Exchange(List<List<T>> routes) {
		// 要产生一个新的操作，就要先对原始路径做拷贝，防止操作后得出的结果比原来更差的试试有保留原来的路径可以继续试验
		List<List<T>> copyRoutes = new ArrayList<List<T>>(routes);
		System.out.println("copyRoutes size=" + copyRoutes.size());
		// 随机选择出需要做操作算子的是两条路径
		Random random = new Random();
		int idx1 = random.nextInt(copyRoutes.size());
		int idx2 = random.nextInt(copyRoutes.size());
		while (idx1 == idx2) {
			idx2 = random.nextInt(copyRoutes.size());
		}
		System.out.println("idx1 = " + idx1 + "，\t" + "idx2=" + idx2);
		// 获得需要操作的那两个路径
		List<T> r1 = copyRoutes.get(idx1);
		List<T> r2 = copyRoutes.get(idx2);
		System.out.println("r1 size =" + r1.size() + "，\t" + "r2 size=" + r2.size());
		// 随机出需要删除的数量和位置，然后才能够其中一个路径中删除，加入另一个路径中
		int moveNum = random.nextInt(3) + 1;// 要删除移动的数量
		// 如果本身路径只有2-3个点，这时候要移去的数目就会超过已有点数量，这时候就不移动了，直接返回原始的数据
		int index = random.nextInt((Math.min(r1.size(), r2.size()))) + 1;// 从哪个位置开始
		System.out.println("前moveNum=" + moveNum + "，\t" + "index=" + index);
		// 如果位置后的数量不足moveNum，则重新再随机
		if ((index + moveNum >= r1.size()) || (index + moveNum >= r2.size())) {
			index = 0;
			moveNum = 1;
		}
		System.out.println("后moveNum=" + moveNum + ",\t" + "index=" + index);
		// 对a中，删除index后面moveNum个元素，并记录下这些元素信息，整体插入到r2中index位置
		List<T> tmp = new ArrayList<T>();
		for (int i = index; i < index + moveNum; i++) {
			tmp.add(r1.get(i));
		}
		// 移除原始序列中index位置之后的moveNum个数字
		int cnt = moveNum;
		while (cnt > 0) {
			r1.remove(index);
			cnt--;
		}
		r2.addAll(index, tmp);
		return copyRoutes;

	}

	/**
	 * a数组（染色体）随机交换其中的两个点，在本身数组里面直接交换，不产生新数组
	 * 这是针对单个List的，对于List<List<T>>这个情况，则用另一个函数来调用这个来实现
	 * 
	 * @param a
	 *            数组（染色体
	 */
	public List<T> Swap(List<T> a) {
		Random random = new Random();
		int r1 = random.nextInt(a.size());
		int r2 = random.nextInt(a.size());
		while (r1 == r2) {
			r2 = random.nextInt(a.size());
		}
		T tmp = a.get(r1);
		a.set(r1, a.get(r2));
		a.set(r2, tmp);
		return a;
	}

	public static void print(Object[] o) {
		for (int i = 0; i < o.length; i++) {
			System.out.print(o[i].toString() + " ");
		}
		System.out.println();
	}

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		Integer[] num1 = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
		Integer[] num2 = { 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };
		Operator<Integer> opera = new Operator<Integer>();
		// 测试two_Exchange
		// Integer[] out = opera.two_Exchange(num1, 2, 8);
		// print(out);
		// 测试or_Exchange
		// List<List<Integer>> mList ;
		// mList = opera.or_Exchange(num1, num2);
		// for (List<Integer> ml:mList) {
		// for (Integer tmp:ml) {
		// System.out.print(tmp+" ");
		// }
		// System.out.println();
		// }
		// 测试Swap
		// opera.Swap(num1);
		// for (Integer t:num1) {
		// System.out.print(t+" ");
		// }
		// System.out.println();
	}

}
